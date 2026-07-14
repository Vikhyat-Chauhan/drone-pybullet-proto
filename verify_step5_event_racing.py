"""Step 5 verification: inject one hardcoded event and confirm the native
APE1/APE2/APE3 threads race, produce proposals, and nav.tick() resolves
and commits to a winning command -- isolating the ctypes/_build_ape_params
marshaling from the full deterministic event generator (ported next, in
Step 6). Headless (p.DIRECT), selector_mode="CA" so all three APEs run.
"""
import logging
import math
import time

import pybullet as p
import pybullet_data

from arena import ArenaCfg, generate_buildings, pick_start_target
from lidar import Lidar2D
from config import TeleopConfig
from teleop import PyTeleop
from sim_adapters import SimClock, PoseProvider, StaticPoseProvider, ScanProvider, CloudProvider, EventQueue
from nav_algorithm import LidarTargetNavigatorCA

logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger("nav_algorithm")

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

teleop = PyTeleop(drone_body, cfg)
nav = LidarTargetNavigatorCA(teleop, cfg, "CA", drone_pose, target_pose,
                              scan_provider, cloud_provider, events, sim_clock)

nav_period = 1.0 / nav._gc.rate_hz
next_nav_t = 0.0
nav.begin_mission()

# Inject one hardcoded event 2 sim-seconds in, with a deadline generous
# enough for all three APEs to finish (ape3_budget_ms is the largest).
EVENT_AT_S = 2.0
DEADLINE_S = 2.0  # generous, well above ape3_budget_ms/1000
event_pushed = False

print(f"ape1_budget_ms={nav._edc.ape1_budget_ms:.3f} ape2_budget_ms={nav._edc.ape2_budget_ms:.3f} "
      f"ape3_budget_ms={nav._edc.ape3_budget_ms:.3f}")

status = "running"
step = 0
while sim_clock.now() < 30.0:
    scan_provider.update()
    cloud_provider.update()

    if not event_pushed and sim_clock.now() >= EVENT_AT_S:
        events.push({
            "kind": "SUDDEN_OBSTACLE",
            "t_emit": sim_clock.now(),
            "t_recv": sim_clock.now(),
            "deadline_s": DEADLINE_S,
            "meta": {},
        })
        event_pushed = True
        print(f"\n--- pushed test event at sim_t={sim_clock.now():.3f}s ---\n")

    if sim_clock.now() >= next_nav_t:
        status = nav.tick((target_xy[0], target_xy[1], cfg.flight_z))
        next_nav_t += nav_period
        if status == "reached":
            break

    teleop.physics_step(phys_dt)
    p.stepSimulation()
    sim_clock.tick()
    step += 1

    if event_pushed and sim_clock.now() > EVENT_AT_S + DEADLINE_S + 1.0:
        break

reached, elapsed, latency_us, energy_j, handled, violated, viol_deadline, viol_preempt = nav.end_mission(status == "reached")
print(f"\nevents_handled={handled} events_violated={violated} "
      f"(deadline={viol_deadline}, preemptive={viol_preempt})")
assert handled >= 1, "event was never picked up by nav.tick()"
assert violated == 0, "event should have resolved well within its generous deadline"
print("OK: native APE1/APE2/APE3 event racing pipeline verified end to end")
