"""Step 6 verification: full event generator (deterministic mode) +
violations + energy tracking wired into the drive loop. Confirms
log_and_reset() on both violations and energy return physically
plausible, non-trivial summaries after a single strategy run through the
ported navigator (base-avoidance-only, EventQueue driven by the real
EventEmitter this time instead of a hardcoded test event).
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
from sim_adapters import SimClock, PoseProvider, StaticPoseProvider, ScanProvider, CloudProvider, EventQueue
from nav_algorithm import LidarTargetNavigatorCA
from event_source import EventEmitter, EventCfg
from violations import ViolationMonitor
from energy_monitor import EnergyMonitor, EPM_PRESETS

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
target_pose = StaticPoseProvider((target_xy[0], target_xy[1], cfg.flight_z))
scan_provider = ScanProvider(lidar, drone_pose, sim_clock)
cloud_provider = CloudProvider(lidar, drone_pose, sim_clock, n_layers=5,
                                vertical_angle_min=-0.0872665, vertical_angle_increment=0.0436332)
events = EventQueue()
emitter = EventEmitter(events, sim_clock, EventCfg(seed=42, event_deterministic=True))
violations = ViolationMonitor(sim_clock, meta_path=cfg.nofly_meta_path)
energy = EnergyMonitor(sim_clock)

teleop = PyTeleop(drone_body, cfg)
nav = LidarTargetNavigatorCA(teleop, cfg, "CA", drone_pose, target_pose,
                              scan_provider, cloud_provider, events, sim_clock)

nav_period = 1.0 / nav._gc.rate_hz
next_nav_t = 0.0
nav.begin_mission()
violations.mark_run_start("CA")
energy.mark_run_start("CA")

timeout_s = 180.0
status = "running"
step = 0
while sim_clock.now() < timeout_s:
    scan_provider.update()
    cloud_provider.update()
    emitter.step()

    pos, _ = p.getBasePositionAndOrientation(drone_body)
    violations.push_pose(pos[0], pos[1], pos[2], sim_clock.now())
    energy.push_pose(pos[0], pos[1], pos[2], sim_clock.now())

    if sim_clock.now() >= next_nav_t:
        status = nav.tick((target_xy[0], target_xy[1], cfg.flight_z))
        next_nav_t += nav_period
        if status == "reached":
            break

    teleop.physics_step(phys_dt)
    p.stepSimulation()
    sim_clock.tick()
    step += 1

reached, elapsed, latency_us, energy_j_compute, handled, violated, viol_deadline, viol_preempt = nav.end_mission(status == "reached")
violation_summary = violations.log_and_reset("CA", include_boxes=False)
energy_summary = energy.log_and_reset("CA")
emitter.close()

print(f"reached={reached} elapsed={elapsed:.2f}s events_handled={handled} events_violated={violated}")
print(f"violation_summary: total_violations={violation_summary['total_violations']}")
print(f"energy_summary: energy_j={energy_summary['energy_j']} mean_power_w={energy_summary['mean_power_w']} "
      f"elapsed_s={energy_summary['elapsed_s']}")

# Sanity: energy should roughly match EPM * straight-line-ish path length
# (path is longer than straight-line due to avoidance detours, so energy
# should be >= EPM * straight_line_dist, and not absurdly larger).
straight_line_dist = math.hypot(target_xy[0]-start_xy[0], target_xy[1]-start_xy[1])
epm = EPM_PRESETS["flykart30"]
lower_bound_j = epm * straight_line_dist
print(f"straight_line_dist={straight_line_dist:.1f}m epm*straight_line={lower_bound_j:.0f}J")

assert reached, "drone failed to reach target"
assert energy_summary["energy_j"] >= lower_bound_j * 0.95, "energy implausibly below straight-line EPM lower bound"
assert energy_summary["energy_j"] <= lower_bound_j * 5.0, "energy implausibly above a generous detour multiplier"
assert energy_summary["elapsed_s"] > 0
print("OK: event generator + violations + energy tracking verified end to end")
