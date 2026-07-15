#!/usr/bin/env python3
"""CANavigator experiment runner, ported off Gazebo/ROS2 onto PyBullet.

Per attempt: generate an arena + target, run each strategy (APE1/APE2/APE3/CA)
on one PyBullet body teleported back to start between strategies (cheaper than
a full sim restart), discard the attempt if any strategy misses the target,
else record one CSV row per strategy. `gui` switches between a real-time
visual demo and headless batch mode (p.DIRECT, no sleep pacing) -- both share
the same drive loop and SimClock-based determinism.

Entrypoint: run.py (Hydra-driven). See docs/CONFIGURATION.md.
"""
import logging
import math
import time
from typing import Dict, Optional

import pybullet as p

from config import TeleopConfig
from sim.arena import (
    ArenaCfg, PerlinArenaCfg, generate_buildings_perlin, generate_city_layout,
    pick_target, write_nofly_meta, write_target_meta,
)
from sim.lidar import Lidar2D
from sim.teleop import PyTeleop
from sim.sim_adapters import SimClock, PoseProvider, StaticPoseProvider, ScanProvider, CloudProvider, EventQueue
from sim.event_source import EventEmitter, EventCfg
from sim.violations import ViolationMonitor
from sim.energy_monitor import EnergyMonitor
from nav.algorithm import LidarTargetNavigatorCA
from viz.hud_overlay import MetricsOverlay
from viz.hud_text import format_hud_text
from viz.world_builder import (
    TARGET_RADIUS, build_arena_bodies, render_city_ground_detail, make_drone,
    make_target, make_start_marker, make_event_markers, hide_event_markers,
    update_event_marker, _prefer_nvidia_gpu,
)
from experiment.results_io import init_csv, flush_csv
from applog.async_logger import setup_async_logger, AsyncLoggerCfg
from analysis.statistics_analyzer import run_analysis

# Explicit, not __name__: stays "main" whether run as __main__ or imported --
# analysis/log_transformer.py's log reconstruction keys off this exact name.
_main_logger = logging.getLogger("main")

N_LAYERS = 5
VERTICAL_ANGLE_MIN = -0.0872665
VERTICAL_ANGLE_INCREMENT = 0.0436332


def _attempt_seed(cfg: TeleopConfig, attempt_idx: int) -> int:
    return cfg.world_gen_seed_offset + (0 if cfg.fixed_seed else ((attempt_idx - 1) % 100) + 1)


_trail_ids: list = []


def run_strategy(strategy: str, cfg: TeleopConfig, drone_body: int, start_xy, target_xy,
                  lidar: Lidar2D, sim_clock: SimClock, phys_dt: float, gui: bool,
                  event_markers: Optional[Dict[str, int]] = None,
                  overlay: Optional[MetricsOverlay] = None):
    """Run one strategy from a teleported-to-start drone body through to
    reached/timeout. Returns the nav 8-tuple plus violation/energy summaries."""
    p.resetBasePositionAndOrientation(drone_body, [start_xy[0], start_xy[1], cfg.flight_z], [0, 0, 0, 1])
    p.resetBaseVelocity(drone_body, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])
    if gui:
        # Fresh trail per strategy -- else successive teleported-back runs
        # pile trails on top of each other across a long GUI session.
        for tid in _trail_ids:
            p.removeUserDebugItem(tid)
        _trail_ids.clear()
        if event_markers is not None:
            hide_event_markers(event_markers)

    drone_pose = PoseProvider(drone_body)
    target_pose = StaticPoseProvider((target_xy[0], target_xy[1], cfg.flight_z))
    scan_provider = ScanProvider(lidar, drone_pose, sim_clock)
    cloud_provider = CloudProvider(lidar, drone_pose, sim_clock, n_layers=N_LAYERS,
                                    vertical_angle_min=VERTICAL_ANGLE_MIN,
                                    vertical_angle_increment=VERTICAL_ANGLE_INCREMENT)
    events = EventQueue()
    emitter = EventEmitter(events, sim_clock, EventCfg(
        seed=cfg.event_seed, event_deterministic=cfg.event_deterministic,
        dt_min_s=cfg.event_dt_min_s,
        dt_max_s=cfg.event_dt_max_s,
        deadline_alpha=cfg.deadline_alpha,
        deadline_min_s=cfg.deadline_min_s,
        deadline_max_s=cfg.deadline_max_s,
        mix_enemy=cfg.event_mix_enemy,
        mix_obstacle=cfg.event_mix_obstacle,
        mix_lane=cfg.event_mix_lane,
        log_csv_path=cfg.event_log_csv_path,
    ))
    violations = ViolationMonitor(sim_clock, meta_path=cfg.nofly_meta_path)
    energy = EnergyMonitor(sim_clock)

    teleop = PyTeleop(drone_body, cfg)
    nav = LidarTargetNavigatorCA(teleop, cfg, strategy, drone_pose, target_pose,
                                  scan_provider, cloud_provider, events, sim_clock)

    nav.begin_mission()
    violations.mark_run_start(strategy)
    energy.mark_run_start(strategy)

    nav_period = 1.0 / nav._gc.rate_hz
    t_start = sim_clock.now()
    next_nav_t = t_start
    status = "running"
    print(f"    -> {strategy}: start=({start_xy[0]:.1f},{start_xy[1]:.1f}) "
          f"target=({target_xy[0]:.1f},{target_xy[1]:.1f}) "
          f"straight_line_dist={math.hypot(target_xy[0]-start_xy[0], target_xy[1]-start_xy[1]):.1f}m")

    # GUI-only: a metrics overlay window (viz/hud_overlay.py, a separate
    # OS-level window) updated at the ~2s print cadence below, plus a
    # sparse (~2Hz) flight-path trail.
    if overlay is not None:
        overlay.update(f"{strategy}\nstatus: running")
    trail_last_pos = (start_xy[0], start_xy[1], cfg.flight_z)
    cam_yaw_deg = 90.0  # faces +Y at drone yaw=0; smoothed below so wind/control wobble doesn't shake the view
    last_evt_seq = 0
    evt_hide_at = [float("inf")]

    step = 0
    while True:
        scan_provider.update()
        cloud_provider.update()
        emitter.step()

        pos, orn = p.getBasePositionAndOrientation(drone_body)
        violations.push_pose(pos[0], pos[1], pos[2], sim_clock.now())
        energy.push_pose(pos[0], pos[1], pos[2], sim_clock.now())

        if gui and event_markers is not None:
            # Must run every iteration, not throttled: events can arrive as
            # fast as phys_dt (0.0167s) < event_dt_min_s (0.02s default), so
            # throttling could silently skip a marker.
            drone_yaw = p.getEulerFromQuaternion(orn)[2]
            last_evt_seq = update_event_marker(event_markers, emitter, last_evt_seq,
                                                pos, drone_yaw, sim_clock.now(), evt_hide_at)

        if sim_clock.now() >= next_nav_t:
            status = nav.tick((target_xy[0], target_xy[1], cfg.flight_z))
            next_nav_t += nav_period
            if status == "reached":
                if overlay is not None:
                    dist = math.hypot(pos[0] - target_xy[0], pos[1] - target_xy[1])
                    overlay.update(format_hud_text(strategy, "reached", dist, sim_clock.now(),
                                                    nav, violations, energy))
                break

        teleop.physics_step(phys_dt)
        p.stepSimulation()
        sim_clock.tick()
        step += 1

        if gui:
            # Chase cam tracks heading instead of sitting at a fixed angle.
            # cameraYaw=270-heading puts the camera behind the nose
            # (verified empirically by rendering the nose marker). Heading
            # is smoothed (shortest-angle lerp) so wind/control wobble
            # doesn't judder the view.
            heading_deg = math.degrees(p.getEulerFromQuaternion(orn)[2])
            target_cam_yaw = 270.0 - heading_deg
            d = (target_cam_yaw - cam_yaw_deg + 180.0) % 360.0 - 180.0
            cam_yaw_deg += 0.15 * d
            p.resetDebugVisualizerCamera(cameraDistance=22, cameraYaw=cam_yaw_deg, cameraPitch=-50,
                                          cameraTargetPosition=[pos[0], pos[1], 0])
            if step % 30 == 0:  # ~2Hz trail sampling -- sparse so segments don't pile up
                _trail_ids.append(p.addUserDebugLine(trail_last_pos, pos, lineColorRGB=[0.1, 0.9, 0.9],
                                                       lineWidth=2.0, lifeTime=0))
                trail_last_pos = pos
            time.sleep(phys_dt)

        if step % 120 == 0:  # ~every 2 sim-seconds, both GUI and headless
            dist = math.hypot(pos[0] - target_xy[0], pos[1] - target_xy[1])
            print(f"       t={sim_clock.now():6.1f}s pos=({pos[0]:7.2f},{pos[1]:7.2f}) "
                  f"dist_to_target={dist:6.2f} status={status}")
            if overlay is not None:
                overlay.update(format_hud_text(strategy, status, dist, sim_clock.now(),
                                                nav, violations, energy))

        if (sim_clock.now() - t_start) > cfg.simulation_timeout:
            break

    teleop.stop()
    nav_result = nav.end_mission(status == "reached")
    violation_summary = violations.log_and_reset(strategy, include_boxes=True)
    energy_summary = energy.log_and_reset(strategy)
    emitter.close()
    return nav_result, violation_summary, energy_summary


def run_attempt(cfg: TeleopConfig, attempt_idx: int, gui: bool, strategies,
                 overlay: Optional[MetricsOverlay] = None):
    p.resetSimulation()
    p.setGravity(0, 0, 0)
    phys_dt = 1.0 / 60.0
    p.setTimeStep(phys_dt)
    sim_clock = SimClock(phys_dt)

    seed = _attempt_seed(cfg, attempt_idx)

    city_layout = None
    if cfg.simulation_world_style == "perlin":
        arena_cfg = PerlinArenaCfg(seed=seed)
        buildings = generate_buildings_perlin(arena_cfg)
    else:
        arena_cfg = ArenaCfg(seed=seed)
        # generate_city_layout() (vs. generate_buildings()) also returns
        # cosmetic road/pavement/grass detail, needed below.
        city_layout = generate_city_layout(arena_cfg)
        buildings = city_layout.buildings

    building_ids = build_arena_bodies(buildings, arena_cfg)
    if gui and city_layout is not None:
        render_city_ground_detail(city_layout)
    cfg.nofly_rects_xywh = [(b.cx, b.cy, 2 * b.hx, 2 * b.hy) for b in buildings]
    write_nofly_meta(cfg.nofly_meta_path, arena_cfg.x_min, arena_cfg.x_max,
                      arena_cfg.y_min, arena_cfg.y_max, buildings, seed, cfg.simulation_world_style)

    # Fixed spawn point, no collision search needed: the collision-filter
    # pairs set up below make buildings non-solid to the drone.
    start_xy = (cfg.start_x, cfg.start_y)
    target_seed = seed + 1
    target_xy = pick_target(arena_cfg.x_min, arena_cfg.x_max, arena_cfg.y_min, arena_cfg.y_max,
                             buildings, start_xy, seed=target_seed, min_dist_start=cfg.target_distance)
    write_target_meta(cfg.target_json_path, target_xy[0], target_xy[1], cfg.flight_z,
                       TARGET_RADIUS, cfg.nofly_meta_path, start_xy, cfg.target_distance)

    drone_body = make_drone(start_xy, cfg.flight_z, cfg.drone_radius_m)
    # Buildings are pass-through (a nav miss is a soft zone_violation, not a
    # bounce): disable collision *response* per pair, but keep the shape
    # intact so sim/lidar.py's rayTestBatch still detects them.
    for bid in building_ids:
        p.setCollisionFilterPair(drone_body, bid, -1, -1, enableCollision=0)
    make_target(target_xy, cfg.flight_z, gui=gui)
    event_markers = None
    if gui:
        make_start_marker(start_xy, cfg.flight_z)
        event_markers = make_event_markers()
    lidar = Lidar2D(num_rays=48, max_range=15.0, fov_deg=300.0, draw_debug=gui)

    all_reached = True
    buffered = []
    for strategy in strategies:
        (reached, elapsed, latency_us, compute_energy_j, events_handled,
         events_violated, events_violated_deadline, events_violated_preemptive), \
            violation_summary, energy_summary = run_strategy(
                strategy, cfg, drone_body, start_xy, target_xy, lidar, sim_clock, phys_dt, gui,
                event_markers=event_markers, overlay=overlay,
            )

        buffered.append({
            "strategy": strategy, "elapsed": elapsed,
            "violations": violation_summary.get("total_violations"),
            "compute_latency_us": latency_us, "compute_energy_j": compute_energy_j,
            "energy_j": energy_summary.get("energy_j"),
            "mean_power_w": energy_summary.get("mean_power_w"),
            "events_handled": events_handled, "events_violated": events_violated,
            "events_violated_deadline": events_violated_deadline,
            "events_violated_preemptive": events_violated_preemptive,
        })

        # Terminator record for analysis/log_transformer.py's offline
        # reconstruction of experiment_summary.csv from run_logs.json --
        # redundant with the flush_csv write below, kept for verification.
        _main_logger.info({
            "reached": reached,
            "elapsed": elapsed,
            "violations": violation_summary.get("total_violations"),
            "energy_j": energy_summary.get("energy_j"),
            "mean_power_w": energy_summary.get("mean_power_w"),
            "compute_latency_us": latency_us,
        }, extra={"strategy": strategy})

        status_icon = "OK" if reached else "FAIL"
        print(f"  [{status_icon}] {strategy}: elapsed={elapsed:.1f}s "
              f"violations={violation_summary.get('total_violations')} "
              f"energy_j={energy_summary.get('energy_j'):.0f} "
              f"events={events_handled}/{events_violated} violated")

        if not reached:
            all_reached = False
            break

    return all_reached, buffered


def main(gui: bool = True, strategies=None, simulation_runs: int = None,
         cfg: Optional[TeleopConfig] = None) -> None:
    if cfg is None:
        cfg = TeleopConfig()
    if simulation_runs is not None:
        cfg.simulation_runs = simulation_runs
    strategies = strategies or cfg.analyzer_strategies

    logcfg = AsyncLoggerCfg(
        logfile=cfg.log_path, max_bytes=0, queue_maxsize=8000,
        drop_on_full=False, console=False, level=logging.INFO, json_format=True,
    )
    log_handle = setup_async_logger(logcfg)

    init_csv(cfg.results_csv_path)

    if gui:
        _prefer_nvidia_gpu()
        # Sky-blue background; no-ops harmlessly on builds that ignore it.
        p.connect(p.GUI, options="--background_color_red=0.53 "
                                  "--background_color_green=0.81 --background_color_blue=0.92")
    else:
        p.connect(p.DIRECT)
    if gui:
        # Shadows read scene depth better than flat boxes; hide default
        # side panels so they don't shrink the arena view into a corner.
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        # Off-angle elevated light casts long shadows across the street
        # grid, vs. PyBullet's default near-overhead light pooling straight down.
        p.configureDebugVisualizer(lightPosition=[60, -80, 100])

    # One overlay window per process, persists across attempts/strategies.
    overlay = MetricsOverlay() if gui else None

    good_runs = 0
    attempt_idx = 0
    t_wall_start = time.time()
    try:
        while good_runs < cfg.simulation_runs:
            attempt_idx += 1
            run_idx = good_runs + 1
            print(f"\n=== Attempt {attempt_idx} (run {run_idx}/{cfg.simulation_runs}, "
                  f"{cfg.simulation_world_style} arena) ===")

            all_reached, buffered = run_attempt(cfg, attempt_idx, gui, strategies, overlay=overlay)

            if all_reached:
                good_runs += 1
                flush_csv(cfg.results_csv_path, run_idx, buffered)
                print(f"Run {run_idx}: all strategies reached ({good_runs}/{cfg.simulation_runs} good runs)")
            else:
                print(f"Attempt {attempt_idx} discarded: a strategy failed to reach target")
    finally:
        p.disconnect()
        log_handle.stop()
        if overlay is not None:
            overlay.close()

    print(f"\nTotal wall time: {time.time() - t_wall_start:.1f}s for {good_runs} good runs "
          f"({attempt_idx} attempts)")
    run_analysis(zone_metric="mean", cfg=cfg)
