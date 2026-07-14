#!/usr/bin/env python3
"""CANavigator experiment runner, ported off Gazebo/ROS2 onto PyBullet.

Mirrors ca_navigator/main.py's structure: generate a fresh arena + target
per attempt, run each strategy (APE1/APE2/APE3/CA) against it sharing one
PyBullet body (teleported back to start between strategies instead of a
full sim restart -- even cheaper here than the original's Gazebo teleport
since there's no separate process at all), discard the whole attempt if
any strategy fails to reach the target, and record one CSV row per
strategy for each kept ("good") run. Supports both a GUI demo (visual,
real-time-paced) and headless batch mode (p.DIRECT, no sleep pacing, for
fast iteration over many runs) via the `gui` flag -- both share the same
drive loop and SimClock-based determinism.

Run:
    ./venv/bin/python main.py            # GUI demo
    ./venv/bin/python main.py --headless # batch mode, cfg.simulation_runs runs
"""
import csv
import logging
import math
import os
import random
import shutil
import subprocess
import sys
import time
from typing import Dict, Optional

import pybullet as p
import pybullet_data
from PIL import Image, ImageDraw

from arena import (
    ArenaCfg, PerlinArenaCfg, generate_buildings_perlin, generate_city_layout,
    pick_target, write_nofly_meta, write_target_meta,
)
from hud_overlay import MetricsOverlay
from lidar import Lidar2D
from config import TeleopConfig
from teleop import PyTeleop
from sim_adapters import SimClock, PoseProvider, StaticPoseProvider, ScanProvider, CloudProvider, EventQueue
from nav_algorithm import LidarTargetNavigatorCA

# Explicit (not __name__) so this stays "main" whether run as __main__ or
# imported -- analysis/log_transformer.py's offline JSON-log reconstruction
# path keys off this exact logger name for its terminator record.
_main_logger = logging.getLogger("main")
from mcu_cycle_model import latency_to_energy_j
from event_source import EventEmitter, EventCfg
from violations import ViolationMonitor
from energy_monitor import EnergyMonitor
from applog.async_logger import setup_async_logger, AsyncLoggerCfg
from analysis.statistics_analyzer import run_analysis

DRONE_VIS_RGBA = [0.1, 0.4, 1.0, 1.0]
TARGET_RADIUS = 0.6
N_LAYERS = 5
VERTICAL_ANGLE_MIN = -0.0872665
VERTICAL_ANGLE_INCREMENT = 0.0436332

CSV_FIELDNAMES = [
    "run", "strategy", "elapse_time", "zone_violations",
    "compute_latency_us", "compute_energy_j",
    "propulsion_energy_j", "propulsion_mean_power_w",
    "events_handled", "event_violated", "event_violated_deadline",
    "event_violated_preemptive", "event_violation_rate",
]


def _prefer_nvidia_gpu() -> None:
    """On a hybrid AMD/NVIDIA (PRIME) laptop or desktop, PyBullet's GUI
    window otherwise renders through Mesa on the integrated/CPU path,
    which is far slower than the discrete NVIDIA GPU sitting idle. Request
    PRIME render-offload so the GL context PyBullet opens (at p.connect
    time, which reads these env vars lazily) lands on the NVIDIA GPU
    instead. No-op if there's no NVIDIA driver, or if the user has already
    set these vars themselves."""
    if "__GLX_VENDOR_LIBRARY_NAME" in os.environ:
        return
    if not shutil.which("nvidia-smi"):
        return
    try:
        subprocess.run(["nvidia-smi", "-L"], capture_output=True, timeout=2, check=True)
    except Exception:
        return
    os.environ["__NV_PRIME_RENDER_OFFLOAD"] = "1"
    os.environ["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"


WALL_RGBA = [0.15, 0.55, 0.9, 0.08]  # faint translucent tint, matching airport_world.sdf's bounds_visible

# Muted urban palette for building color variety -- kept in a narrow,
# coherent range (grays/tans/blue-grays) rather than pure random RGB, so
# the skyline reads as a city, not a rainbow.
BUILDING_PALETTE = [
    [0.55, 0.55, 0.60, 1.0],
    [0.62, 0.58, 0.52, 1.0],
    [0.48, 0.50, 0.55, 1.0],
    [0.58, 0.54, 0.46, 1.0],
    [0.45, 0.48, 0.52, 1.0],
    [0.60, 0.60, 0.58, 1.0],
]

_PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
_WINDOW_TEX_PATH = os.path.join(_PROJECT_ROOT, "models", "generated", "window_tex.png")
_GROUND_TEX_PATH = os.path.join(_PROJECT_ROOT, "models", "generated", "ground_tex.png")


def _ensure_window_texture() -> str:
    """Procedurally generate (once, cached to disk) a small tileable
    window-grid texture so buildings read as buildings instead of plain
    flat boxes. A base wall tone with a grid of slightly lit/dark squares
    for windows."""
    if not os.path.exists(_WINDOW_TEX_PATH):
        os.makedirs(os.path.dirname(_WINDOW_TEX_PATH), exist_ok=True)
        size = 128
        img = Image.new("RGB", (size, size), (235, 235, 232))
        draw = ImageDraw.Draw(img)
        rng = random.Random(7)
        cell = 16
        margin = 3
        for gy in range(0, size, cell):
            for gx in range(0, size, cell):
                lit = rng.random() < 0.35
                color = (255, 244, 190) if lit else (70, 78, 92)
                draw.rectangle(
                    [gx + margin, gy + margin, gx + cell - margin, gy + cell - margin],
                    fill=color,
                )
        img.save(_WINDOW_TEX_PATH)
    return _WINDOW_TEX_PATH


def _ensure_ground_texture() -> str:
    """Procedurally generate (once, cached to disk) a subtle speckled
    asphalt/concrete-look ground texture, replacing PyBullet's default
    blue/white checkerboard."""
    if not os.path.exists(_GROUND_TEX_PATH):
        os.makedirs(os.path.dirname(_GROUND_TEX_PATH), exist_ok=True)
        size = 256
        base = (112, 110, 106)
        img = Image.new("RGB", (size, size), base)
        rng = random.Random(11)
        px = img.load()
        for _ in range(size * size // 6):
            x, y = rng.randrange(size), rng.randrange(size)
            jitter = rng.randint(-14, 14)
            r, g, b = base
            px[x, y] = (max(0, min(255, r + jitter)), max(0, min(255, g + jitter)), max(0, min(255, b + jitter)))
        img.save(_GROUND_TEX_PATH)
    return _GROUND_TEX_PATH


def build_arena_bodies(buildings, arena_cfg):
    """Ground + solid boundary walls (matching CANavigator's
    worlds/airport_world.sdf 'bounds_visible' model exactly) plus the
    generated NFZ buildings.

    Buildings get a REAL PyBullet collision shape, because unlike
    Ignition Gazebo's GPU LiDAR (which raycasts the rendering scene's
    visual meshes independently of physics collision), PyBullet's
    rayTestBatch only ever intersects collision geometry -- with no
    collision shape at all, this port's LiDAR would be blind to them,
    which is not how the original behaved (the original's `<visual>` is
    written unconditionally regardless of pass_through=True; only
    `<collision>` is what pass_through skips). What pass_through=True
    *does* mean is no rigid-body contact force -- the drone is not
    physically stopped by a building, a nav failure is a soft
    zone_violation, not a bounce. Since PyBullet ties raycasting to
    collision shapes, that's reproduced here via a collision-filter pair
    that disables contact resolution between the drone and each building
    while leaving both bodies' collision geometry intact for
    rayTestBatch. See run_attempt(), which wires this filter pair up
    once the drone body exists.
    """
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    ground_id = p.loadURDF("plane.urdf")
    ground_tex = p.loadTexture(_ensure_ground_texture())
    p.changeVisualShape(ground_id, -1, textureUniqueId=ground_tex)

    window_tex = p.loadTexture(_ensure_window_texture())
    palette_rng = random.Random(arena_cfg.seed)
    building_ids = []
    for b in buildings:
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[b.hx, b.hy, b.height / 2])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[b.hx, b.hy, b.height / 2],
                                   rgbaColor=palette_rng.choice(BUILDING_PALETTE))
        bid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                                 basePosition=[b.cx, b.cy, b.height / 2])
        p.changeVisualShape(bid, -1, textureUniqueId=window_tex)
        building_ids.append(bid)

    thickness, height, z_pose = 1.0, 50.0, 5.0
    x_min, x_max, y_min, y_max = arena_cfg.x_min, arena_cfg.x_max, arena_cfg.y_min, arena_cfg.y_max
    x_span, y_span = x_max - x_min, y_max - y_min
    walls = [
        ([x_span / 2, thickness / 2, height / 2], [0, y_max, z_pose]),   # north
        ([x_span / 2, thickness / 2, height / 2], [0, y_min, z_pose]),   # south
        ([thickness / 2, y_span / 2, height / 2], [x_max, 0, z_pose]),   # east
        ([thickness / 2, y_span / 2, height / 2], [x_min, 0, z_pose]),   # west
        ([x_span / 2, y_span / 2, thickness / 2], [0, 0, 30.0]),         # ceiling
    ]
    for half_extents, pos in walls:
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=WALL_RGBA)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                           basePosition=pos)

    return building_ids


ROAD_RGBA = [0.12, 0.12, 0.13, 1.0]
PAVEMENT_RGBA = [0.72, 0.70, 0.65, 1.0]
GRASS_RGBA = [0.22, 0.50, 0.18, 1.0]


def render_city_ground_detail(layout) -> None:
    """Flat, no-collision ground decals for the road grid, developed-block
    pavement, and grass on undeveloped lots (arena.CityLayout). Purely
    cosmetic: no collisionShapeIndex at all, so this can never affect
    LiDAR/rayTestBatch or the drone-building collision-filter setup.
    Stacked at increasing z (road < pavement < grass) so grass and
    pavement patches render on top of the road layer without z-fighting.
    """
    def _flat_rect(cx, cy, w, h, z, rgba):
        half = [max(w / 2.0, 0.05), max(h / 2.0, 0.05), 0.01]
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba)
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis, basePosition=[cx, cy, z])

    for (cx, cy, w, h) in layout.road_rects:
        _flat_rect(cx, cy, w, h, 0.01, ROAD_RGBA)
    for (cx, cy, w, h) in layout.pavement_rects:
        _flat_rect(cx, cy, w, h, 0.02, PAVEMENT_RGBA)
    for (cx, cy, w, h) in layout.grass_rects:
        rng = random.Random(int(cx * 1000) + int(cy * 1000))
        jitter = rng.uniform(-0.05, 0.05)
        rgba = [max(0.0, min(1.0, c + jitter)) for c in GRASS_RGBA[:3]] + [1.0]
        _flat_rect(cx, cy, w, h, 0.03, rgba)


NOSE_RGBA = [1.0, 0.15, 0.1, 1.0]


def make_drone(start_xy, flight_z, radius):
    col = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    # Compound visual (sphere body + a small forward-offset "nose" box) so
    # heading/yaw is visible at a glance -- a plain sphere gives no visual
    # cue which way the drone is facing. The nose is visual-only (kept out
    # of the collision shape) and rides along with the body for free since
    # it's baked into the same multibody's visual frame, no manual per-tick
    # pose sync required.
    vis = p.createVisualShapeArray(
        shapeTypes=[p.GEOM_SPHERE, p.GEOM_BOX],
        radii=[radius, 0.0],
        halfExtents=[[0, 0, 0], [radius * 0.4, radius * 0.22, radius * 0.22]],
        visualFramePositions=[[0, 0, 0], [radius * 0.85, 0, 0]],
        rgbaColors=[DRONE_VIS_RGBA, NOSE_RGBA],
    )
    body = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                              basePosition=[start_xy[0], start_xy[1], flight_z])
    p.changeDynamics(body, -1, linearDamping=0.0, angularDamping=0.0)
    return body


BEACON_RGB = [1.0, 0.1, 0.7]
START_MARKER_RGB = [0.15, 1.0, 0.3]


def make_target(target_xy, flight_z, gui: bool = False):
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=TARGET_RADIUS, rgbaColor=[1.0, 0.85, 0.0, 1.0])
    body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis,
                              basePosition=[target_xy[0], target_xy[1], flight_z])
    if gui:
        # The 0.6m target sphere is easy to lose track of across a
        # 200x100m arena -- a tall beacon line is visible from anywhere.
        p.addUserDebugLine([target_xy[0], target_xy[1], flight_z], [target_xy[0], target_xy[1], 40.0],
                            lineColorRGB=BEACON_RGB, lineWidth=3.0, lifeTime=0)
    return body


def make_start_marker(start_xy, flight_z):
    """A flat ring on the ground plus a short pole at the drone's fixed
    spawn point -- one per attempt (the drone teleports back to this same
    point for every strategy within an attempt, so no need to redraw it
    per-strategy)."""
    n = 24
    cx, cy = start_xy
    ring = [(cx + 1.5 * math.cos(2 * math.pi * i / n), cy + 1.5 * math.sin(2 * math.pi * i / n), 0.05)
            for i in range(n + 1)]
    for a, b in zip(ring, ring[1:]):
        p.addUserDebugLine(a, b, lineColorRGB=START_MARKER_RGB, lineWidth=3.0, lifeTime=0)
    p.addUserDebugLine([cx, cy, 0.0], [cx, cy, flight_z + 2.0],
                        lineColorRGB=START_MARKER_RGB, lineWidth=2.0, lifeTime=0)


# Event markers are visual-only timing/kind cues, NOT literal obstacles the
# nav logic is computed to avoid: in this codebase (as in the original),
# a deterministic-mode event carries no world position at all --
# EventEmitter._emit() only ever produces {kind, t_emit, deadline_s,
# meta: {i, kind}} (event_source.py). nav_algorithm.py's reaction to an
# event is a timing-pressure race (APE1/2/3 re-plan against whatever the
# LiDAR *already* sees), not a reaction to a new object at a specific
# spot. These markers exist so an event's kind/timing is visible and
# correlates with the drone's reaction -- they spawn a plausible stand-in
# distance ahead of the drone, not a location the nav actually used.
EVENT_MARKER_SPECS = {
    # kind -> (shapeType, shape_kwargs, rgb)
    "ENEMY": (p.GEOM_SPHERE, {"radius": 0.9}, [0.95, 0.1, 0.1]),
    "SUDDEN_OBSTACLE": (p.GEOM_CYLINDER, {"radius": 0.7, "length": 2.2}, [1.0, 0.55, 0.0]),
    "LANE_BLOCK": (p.GEOM_BOX, {"halfExtents": [3.5, 0.3, 1.0]}, [0.95, 0.9, 0.1]),
}
EVENT_MARKER_AHEAD_M = 7.0       # plausible stand-in distance ahead of the drone
EVENT_MARKER_BEARING_JITTER_DEG = 25.0
EVENT_MARKER_VISIBLE_S = 1.5     # auto-hide if nothing supersedes it


def make_event_markers() -> Dict[str, int]:
    """Pre-create one body per event kind, once per attempt, starting
    fully transparent (alpha 0) and with no collision shape at all --
    "initially not physically interacting with the world." Only alpha and
    position are ever touched afterward (update_event_marker), so there's
    no per-event createMultiBody/removeBody churn -- the same reasoning
    already applied to the LiDAR debug-ray fix earlier this session
    (avoid unthrottled per-tick object churn)."""
    markers: Dict[str, int] = {}
    for kind, (shape_type, kwargs, rgb) in EVENT_MARKER_SPECS.items():
        vis = p.createVisualShape(shape_type, rgbaColor=rgb + [0.0], **kwargs)
        markers[kind] = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis, basePosition=[0, 0, -50])
    return markers


def hide_event_markers(markers: Dict[str, int]) -> None:
    for kind, body in markers.items():
        rgb = EVENT_MARKER_SPECS[kind][2]
        p.changeVisualShape(body, -1, rgbaColor=rgb + [0.0])


def update_event_marker(markers: Dict[str, int], emitter, last_seen_seq: int,
                         drone_pos, drone_yaw: float, sim_now: float, hide_at: list) -> int:
    """Call every loop iteration (not throttled -- period must stay <=
    EventCfg.dt_min_s so a fast burst of events never skips a visible
    marker). hide_at is a 1-element list used as a mutable float cell.
    Returns the new last_seen_seq."""
    if emitter.last_emit_seq > last_seen_seq:
        last_seen_seq = emitter.last_emit_seq
        evt = emitter.last_event
        kind = evt["kind"] if evt else None
        if kind in markers:
            bearing = drone_yaw + math.radians(random.uniform(-EVENT_MARKER_BEARING_JITTER_DEG,
                                                                EVENT_MARKER_BEARING_JITTER_DEG))
            mx = drone_pos[0] + EVENT_MARKER_AHEAD_M * math.cos(bearing)
            my = drone_pos[1] + EVENT_MARKER_AHEAD_M * math.sin(bearing)
            for k, body in markers.items():
                rgb = EVENT_MARKER_SPECS[k][2]
                if k == kind:
                    # LANE_BLOCK's long axis (local X) needs to span
                    # across the drone's path, not sit parallel to it --
                    # orient perpendicular to the drone's heading (not
                    # the jittered bearing the marker is placed along) so
                    # it consistently reads as a barrier crossing dead
                    # ahead. The sphere/cylinder markers are rotationally
                    # symmetric about Z, so orientation is a no-op for them.
                    orn = p.getQuaternionFromEuler([0, 0, drone_yaw + math.pi / 2.0]) \
                        if k == "LANE_BLOCK" else [0, 0, 0, 1]
                    p.resetBasePositionAndOrientation(body, [mx, my, drone_pos[2]], orn)
                    p.changeVisualShape(body, -1, rgbaColor=rgb + [0.85])
                else:
                    p.changeVisualShape(body, -1, rgbaColor=rgb + [0.0])
            hide_at[0] = sim_now + EVENT_MARKER_VISIBLE_S

    if sim_now > hide_at[0]:
        hide_event_markers(markers)
        hide_at[0] = float("inf")

    return last_seen_seq


def _format_hud_text(strategy: str, status: str, dist: float, sim_now: float,
                      nav, violations, energy) -> str:
    """All the metrics of the current run, for the OS-level overlay window
    (hud_overlay.MetricsOverlay) -- not embedded in the 3D world at all
    (an earlier version anchored debug text to a fixed screen position by
    inverting the chase-cam's view matrix each tick; geometrically correct
    but looked bad in practice, so the panel now lives in its own window).
    Reads live private counters directly off nav/violations/energy
    (nav._events_handled etc., violations._total_violations,
    energy._energy_j) -- an established pattern in this port already
    (e.g. main.py already reads nav._gc.rate_hz elsewhere).

    Compute energy uses the exact same formula nav_algorithm.py's
    end_mission() uses (nav_algorithm.py:606-607) -- McuCycleMeter.end()
    is a pure getter (no reset; only begin_mission()/reset() clear it), so
    calling it mid-run is safe and gives a live running total, not just a
    final one.
    """
    handled = nav._events_handled
    violated = nav._events_violated
    deadline_v = nav._events_violated_deadline
    preempt_v = nav._events_violated_preemptive
    zone_v = violations._total_violations
    energy_j = energy._energy_j
    total_latency_us, _ = nav._cycle_meter.end()
    compute_energy_j = latency_to_energy_j(total_latency_us, max(1e-6, sim_now - nav._t_start))
    return (
        f"{strategy}\n"
        f"status: {status}\n"
        f"dist: {dist:6.1f} m\n"
        f"elapsed: {sim_now:6.1f} s\n"
        f"events: {handled}  violated: {violated}\n"
        f"  deadline: {deadline_v}  preempt: {preempt_v}\n"
        f"zone violations: {zone_v}\n"
        f"energy: {energy_j:8.0f} J\n"
        f"compute energy: {compute_energy_j:6.2f} J"
    )


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
        # Fresh trail per strategy -- otherwise successive strategies (same
        # start/target, teleported back each time) pile trails on top of
        # each other indefinitely across a long GUI session.
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

    # GUI-only visual aids: a metrics overlay window (hud_overlay.py, a
    # separate OS-level window, not embedded in the 3D world) updated at
    # the ~2s print cadence below, and a sparse flight-path trail
    # (throttled to ~2Hz).
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
            # Every iteration, not throttled: must run at least as often
            # as events can arrive (period ~phys_dt=0.0167s <= cfg's
            # event_dt_min_s=0.02s default) so a fast burst of events
            # never silently skips a visible marker.
            drone_yaw = p.getEulerFromQuaternion(orn)[2]
            last_evt_seq = update_event_marker(event_markers, emitter, last_evt_seq,
                                                pos, drone_yaw, sim_clock.now(), evt_hide_at)

        if sim_clock.now() >= next_nav_t:
            status = nav.tick((target_xy[0], target_xy[1], cfg.flight_z))
            next_nav_t += nav_period
            if status == "reached":
                if overlay is not None:
                    dist = math.hypot(pos[0] - target_xy[0], pos[1] - target_xy[1])
                    overlay.update(_format_hud_text(strategy, "reached", dist, sim_clock.now(),
                                                     nav, violations, energy))
                break

        teleop.physics_step(phys_dt)
        p.stepSimulation()
        sim_clock.tick()
        step += 1

        if gui:
            # Chase cam: sit above and behind the drone, looking forward
            # along its heading, instead of a fixed top-right angle that
            # doesn't track which way the drone is actually facing.
            # cameraYaw=270-heading places the camera opposite the nose
            # (verified empirically by rendering the nose marker -- the
            # nose is hidden behind the body from this angle, i.e. facing
            # away from the camera, which is the "behind" chase view).
            # Heading is smoothed (shortest-angle lerp) so wind/control
            # wobble doesn't make the view judder.
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
                overlay.update(_format_hud_text(strategy, status, dist, sim_clock.now(),
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
        # generate_city_layout() also returns cosmetic road/pavement/grass
        # ground detail (see arena.py) -- generate_buildings(cfg) would
        # give the identical building list alone, but the richer layout
        # is needed here to render the extra ground detail below.
        city_layout = generate_city_layout(arena_cfg)
        buildings = city_layout.buildings

    building_ids = build_arena_bodies(buildings, arena_cfg)
    if gui and city_layout is not None:
        render_city_ground_detail(city_layout)
    cfg.nofly_rects_xywh = [(b.cx, b.cy, 2 * b.hx, 2 * b.hy) for b in buildings]
    write_nofly_meta(cfg.nofly_meta_path, arena_cfg.x_min, arena_cfg.x_max,
                      arena_cfg.y_min, arena_cfg.y_max, buildings, seed, cfg.simulation_world_style)

    # Matches the original exactly: a hardcoded fixed spawn point, no
    # collision search. That's valid here (and wasn't before this fix)
    # because the collision-filter pairs set up below make buildings
    # non-solid to the drone -- there's nothing to spawn inside of.
    start_xy = (cfg.start_x, cfg.start_y)
    target_seed = seed + 1
    target_xy = pick_target(arena_cfg.x_min, arena_cfg.x_max, arena_cfg.y_min, arena_cfg.y_max,
                             buildings, start_xy, seed=target_seed, min_dist_start=cfg.target_distance)
    write_target_meta(cfg.target_json_path, target_xy[0], target_xy[1], cfg.flight_z,
                       TARGET_RADIUS, cfg.nofly_meta_path, start_xy, cfg.target_distance)

    drone_body = make_drone(start_xy, cfg.flight_z, cfg.drone_radius_m)
    # pass_through=True in the original: buildings don't exert contact
    # force on the drone (a nav miss is a soft zone_violation, not a
    # bounce). Disabling collision *response* per drone-building pair
    # keeps each building's collision shape intact for rayTestBatch (used
    # by lidar.py), so LiDAR still detects them -- only rigid-body contact
    # resolution is skipped.
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
        # reconstruction of experiment_summary.csv from logs/run_logs.json
        # (the CSV itself is written directly below via _flush_csv --
        # this is a redundant, independently-verifiable path, not the
        # runtime's primary one).
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


def _init_csv(path: str) -> None:
    if not os.path.exists(path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=CSV_FIELDNAMES).writeheader()


def _flush_csv(path: str, run_idx: int, buffered) -> None:
    with open(path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDNAMES)
        for rec in buffered:
            writer.writerow({
                "run": run_idx,
                "strategy": rec["strategy"],
                "elapse_time": round(rec["elapsed"], 2),
                "zone_violations": rec["violations"],
                "compute_latency_us": round(rec["compute_latency_us"], 2),
                "compute_energy_j": round(rec["compute_energy_j"], 2),
                "propulsion_energy_j": round(rec["energy_j"], 2),
                "propulsion_mean_power_w": round(rec["mean_power_w"], 2),
                "events_handled": rec["events_handled"],
                "event_violated": rec["events_violated"],
                "event_violated_deadline": rec["events_violated_deadline"],
                "event_violated_preemptive": rec["events_violated_preemptive"],
                "event_violation_rate": round(
                    rec["events_violated"] / rec["events_handled"] if rec["events_handled"] else 0.0, 2),
            })


def main(gui: bool = True, strategies=None, simulation_runs: int = None) -> None:
    cfg = TeleopConfig()
    if simulation_runs is not None:
        cfg.simulation_runs = simulation_runs
    strategies = strategies or cfg.analyzer_strategies

    logcfg = AsyncLoggerCfg(
        logfile=cfg.log_path, max_bytes=0, queue_maxsize=8000,
        drop_on_full=False, console=False, level=logging.INFO, json_format=True,
    )
    log_handle = setup_async_logger(logcfg)

    _init_csv(cfg.results_csv_path)

    if gui:
        _prefer_nvidia_gpu()
        # Sky-blue GUI background instead of PyBullet's default flat gray.
        # Version-sensitive option string -- harmless no-op on builds that
        # don't support it, so no separate feature check needed.
        p.connect(p.GUI, options="--background_color_red=0.53 "
                                  "--background_color_green=0.81 --background_color_blue=0.92")
    else:
        p.connect(p.DIRECT)
    if gui:
        # Shadows read the scene's depth much better than flat-shaded
        # boxes; hiding the default side panels clears the view of the
        # arena instead of shrinking it into a corner of the window.
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        # Off-angle, elevated light so buildings cast long, natural
        # shadows across the street grid instead of pooling straight
        # underneath (PyBullet's default light sits near-overhead).
        p.configureDebugVisualizer(lightPosition=[60, -80, 100])

    # OS-level overlay window for live run metrics -- one per process,
    # persists across attempts/strategies (see hud_overlay.py).
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
                _flush_csv(cfg.results_csv_path, run_idx, buffered)
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
    run_analysis(zone_metric="mean")


if __name__ == "__main__":
    gui_mode = "--headless" not in sys.argv
    main(gui=gui_mode)
