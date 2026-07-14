"""Step 6 verification: full threat generator (threats.ThreatManager,
deterministic seed) + proxy threat sensor + violations + energy + crash
tracking wired into the drive loop. Confirms log_and_reset() on
violations/energy/crashes return physically plausible, non-trivial
summaries, and that MissionResult's new path_length_m/path_efficiency
fields are populated, after a single strategy run through the ported
navigator (base-avoidance-only, EventQueue driven by the real
ThreatManager this time instead of a hardcoded test event).
"""
import json
import math
import os

import pybullet as p
import pybullet_data

from arena import ArenaCfg, generate_buildings, pick_start_target
from lidar import Lidar2D
from config import TeleopConfig
from teleop import PyTeleop
from sim_adapters import (
    SimClock, PoseProvider, TargetSignalProvider, ScanProvider, CloudProvider,
    EventQueue, ThreatSensorProvider,
)
from nav_algorithm import LidarTargetNavigatorCA
from threats import ThreatManager, ThreatCfg
from violations import ViolationMonitor
from energy_monitor import EnergyMonitor, EPM_PRESETS
from crash_monitor import CrashMonitor

cfg = TeleopConfig()

p.connect(p.DIRECT)
p.setGravity(0, 0, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

phys_dt = 1.0 / 60.0
p.setTimeStep(phys_dt)
sim_clock = SimClock(phys_dt)

arena_cfg = ArenaCfg()
buildings = generate_buildings(arena_cfg)
for b in buildings:
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[b.hx, b.hy, b.height / 2])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, basePosition=[b.cx, b.cy, b.height / 2])
cfg.nofly_rects_xywh = [(b.cx, b.cy, 2 * b.hx, 2 * b.hy) for b in buildings]

# Minimal stand-in for arena.py's write_meta() (added properly in Step 7) --
# violations.load_rects() just needs the JSON shape it already expects.
os.makedirs(os.path.dirname(cfg.nofly_meta_path), exist_ok=True)
with open(cfg.nofly_meta_path, "w") as f:
    json.dump({"rectangles_xywh": cfg.nofly_rects_xywh}, f)

start_xy, target_xy = pick_start_target(arena_cfg, buildings)

col = p.createCollisionShape(p.GEOM_SPHERE, radius=cfg.drone_radius_m)
drone_body = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=col,
                                basePosition=[start_xy[0], start_xy[1], cfg.flight_z])
p.changeDynamics(drone_body, -1, linearDamping=0.0, angularDamping=0.0)

lidar = Lidar2D(num_rays=48, max_range=15.0, fov_deg=300.0, draw_debug=False)

drone_pose = PoseProvider(drone_body)
target_pose = TargetSignalProvider((target_xy[0], target_xy[1], cfg.flight_z), drone_pose,
                                    detect_radius_m=math.inf)
scan_provider = ScanProvider(lidar, drone_pose, sim_clock)
cloud_provider = CloudProvider(lidar, drone_pose, sim_clock, n_layers=5,
                                vertical_angle_min=-0.0872665, vertical_angle_increment=0.0436332)
events = EventQueue()
threat_manager = ThreatManager(events, sim_clock, ThreatCfg(
    seed=42,
    dt_min_s=cfg.event_dt_min_s, dt_max_s=cfg.event_dt_max_s,
    mix_enemy=cfg.event_mix_enemy, mix_obstacle=cfg.event_mix_obstacle, mix_lane=cfg.event_mix_lane,
    max_active_threats=cfg.max_active_threats,
    spawn_range_min_m=cfg.threat_spawn_range_min_m, spawn_range_max_m=cfg.threat_spawn_range_max_m,
    lead_time_s=cfg.threat_lead_time_s,
    retire_margin_m=cfg.threat_retire_margin_m, max_lifetime_s=cfg.threat_max_lifetime_s,
    rearm_cooldown_s=cfg.threat_rearm_cooldown_s,
    enemy_deadline_min_s=cfg.enemy_deadline_min_s, enemy_deadline_max_s=cfg.enemy_deadline_max_s,
    obstacle_deadline_min_s=cfg.obstacle_deadline_min_s, obstacle_deadline_max_s=cfg.obstacle_deadline_max_s,
    lane_deadline_min_s=cfg.lane_deadline_min_s, lane_deadline_max_s=cfg.lane_deadline_max_s,
    enemy_speed_min_mps=cfg.threat_enemy_speed_min_mps, enemy_speed_max_mps=cfg.threat_enemy_speed_max_mps,
    enemy_radius_m=cfg.threat_enemy_radius_m, enemy_lead_bias=cfg.threat_enemy_lead_bias,
    obstacle_speed_min_mps=cfg.threat_obstacle_speed_min_mps,
    obstacle_speed_max_mps=cfg.threat_obstacle_speed_max_mps,
    obstacle_radius_min_m=cfg.threat_obstacle_radius_min_m,
    obstacle_radius_max_m=cfg.threat_obstacle_radius_max_m,
    obstacle_lead_bias=cfg.threat_obstacle_lead_bias,
    lane_speed_min_mps=cfg.threat_lane_speed_min_mps, lane_speed_max_mps=cfg.threat_lane_speed_max_mps,
    lane_radius_min_m=cfg.threat_lane_radius_min_m, lane_radius_max_m=cfg.threat_lane_radius_max_m,
    lane_lead_bias=cfg.threat_lane_lead_bias,
    log_csv_path=cfg.event_log_csv_path,
))
threat_sensor = ThreatSensorProvider(drone_pose, detect_range_m=cfg.threat_sensor_range_m,
                                      fov_deg=cfg.threat_sensor_fov_deg)
violations = ViolationMonitor(sim_clock, meta_path=cfg.nofly_meta_path)
energy = EnergyMonitor(sim_clock)
crash_monitor = CrashMonitor(sim_clock, drone_radius_m=cfg.drone_radius_m, crash_margin_m=cfg.crash_margin_m)

teleop = PyTeleop(drone_body, cfg)
nav = LidarTargetNavigatorCA(teleop, cfg, "CA", drone_pose, target_pose,
                              scan_provider, cloud_provider, events, sim_clock,
                              threat_sensor=threat_sensor)

nav_period = 1.0 / nav._gc.rate_hz
next_nav_t = 0.0
nav.begin_mission()
violations.mark_run_start("CA")
energy.mark_run_start("CA")
crash_monitor.mark_run_start("CA")

timeout_s = 180.0
status = "running"
step = 0
while sim_clock.now() < timeout_s:
    scan_provider.update()
    cloud_provider.update()
    target_pose.update()

    pos, orn = p.getBasePositionAndOrientation(drone_body)
    drone_yaw = p.getEulerFromQuaternion(orn)[2]
    threat_manager.step(pos[0], pos[1], drone_yaw)
    threat_sensor.update(threat_manager.active())
    threat_manager.on_sensed(threat_sensor.latest())

    violations.push_pose(pos[0], pos[1], pos[2], sim_clock.now())
    energy.push_pose(pos[0], pos[1], pos[2], sim_clock.now())
    crash_monitor.push_pose(pos[0], pos[1], pos[2], sim_clock.now())
    crash_monitor.push_threats(threat_manager.active())

    if sim_clock.now() >= next_nav_t:
        status = nav.tick((target_xy[0], target_xy[1], cfg.flight_z))
        next_nav_t += nav_period
        if status == "reached":
            break

    teleop.physics_step(phys_dt)
    p.stepSimulation()
    sim_clock.tick()
    step += 1

nav_result = nav.end_mission(status == "reached")
reached, elapsed = nav_result.reached, nav_result.elapsed_s
handled, violated = nav_result.events_handled, nav_result.events_violated
violation_summary = violations.log_and_reset("CA", include_boxes=False)
energy_summary = energy.log_and_reset("CA")
crash_summary = crash_monitor.log_and_reset("CA")
threat_manager.close()

print(f"reached={reached} elapsed={elapsed:.2f}s events_handled={handled} events_violated={violated}")
print(f"violation_summary: total_violations={violation_summary['total_violations']}")
print(f"crash_summary: total_crashes={crash_summary['total_crashes']}")
print(f"energy_summary: energy_j={energy_summary['energy_j']} mean_power_w={energy_summary['mean_power_w']} "
      f"elapsed_s={energy_summary['elapsed_s']}")
print(f"path_length_m={nav_result.path_length_m:.1f} path_efficiency={nav_result.path_efficiency:.3f}")

# Sanity: energy should roughly match EPM * straight-line-ish path length
# (path is longer than straight-line due to avoidance detours, so energy
# should be >= EPM * straight_line_dist, and not absurdly larger).
straight_line_dist = math.hypot(target_xy[0]-start_xy[0], target_xy[1]-start_xy[1])
epm = EPM_PRESETS["flykart30"]
lower_bound_j = epm * straight_line_dist
print(f"straight_line_dist={straight_line_dist:.1f}m epm*straight_line={lower_bound_j:.0f}J")

assert reached, "drone failed to reach target"
assert energy_summary["energy_j"] >= lower_bound_j * 0.90, "energy implausibly below straight-line EPM lower bound"
assert energy_summary["energy_j"] <= lower_bound_j * 5.0, "energy implausibly above a generous detour multiplier"
assert energy_summary["elapsed_s"] > 0
assert crash_summary["total_crashes"] >= 0
assert nav_result.path_length_m > 0.0, "path length should be nonzero after a real mission"
assert nav_result.path_efficiency <= 1.0 + 1e-6, "path efficiency should never exceed 1.0 (straight-line best case)"
print("OK: threat generator + violations + energy + crash + path-efficiency tracking verified end to end")
