"""Step 4 verification: single-strategy run through the ported
LidarTargetNavigatorCA (base avoidance path only -- EventQueue is built
but never pushed to, so no APE racing happens yet). Headless (p.DIRECT).
Confirms nav.tick() correctly drives PyTeleop -> DronePhysics -> PyBullet
body from start to target around procedurally generated buildings.
"""
import math
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent.parent))

import pybullet as p
import pybullet_data

from sim.arena import ArenaCfg, generate_buildings, pick_start_target
from sim.lidar import Lidar2D
from config import TeleopConfig
from sim.teleop import PyTeleop
from sim.sim_adapters import SimClock, PoseProvider, TargetSignalProvider, ScanProvider, CloudProvider, EventQueue
from nav.algorithm import LidarTargetNavigatorCA

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

# nfz rects consumed by nav.algorithm._nofly_rects() via getattr(cfg, "nofly_rects_xywh", ...)
cfg.nofly_rects_xywh = [(b.cx, b.cy, 2 * b.hx, 2 * b.hy) for b in buildings]

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

teleop = PyTeleop(drone_body, cfg)
nav = LidarTargetNavigatorCA(teleop, cfg, "APE1", drone_pose, target_pose,
                              scan_provider, cloud_provider, events, sim_clock)

nav_period = 1.0 / nav._gc.rate_hz
next_nav_t = 0.0
nav.begin_mission()

timeout_s = 120.0
status = "running"
step = 0
t0_wall = time.time()
while sim_clock.now() < timeout_s:
    scan_provider.update()
    cloud_provider.update()
    target_pose.update()

    if sim_clock.now() >= next_nav_t:
        status = nav.tick((target_xy[0], target_xy[1], cfg.flight_z))
        next_nav_t += nav_period
        if status == "reached":
            break

    teleop.physics_step(phys_dt)
    p.stepSimulation()
    sim_clock.tick()
    step += 1

    if step % 300 == 0:
        pos, _ = p.getBasePositionAndOrientation(drone_body)
        dist = math.hypot(pos[0] - target_xy[0], pos[1] - target_xy[1])
        print(f"sim_t={sim_clock.now():6.1f}s pos=({pos[0]:7.2f},{pos[1]:7.2f}) dist={dist:6.2f} status={status}")

nav_result = nav.end_mission(status == "reached")
reached, elapsed = nav_result.reached, nav_result.elapsed_s
handled, violated = nav_result.events_handled, nav_result.events_violated
print(f"\nreached={reached} elapsed={elapsed:.2f}s sim_steps={step} wall_time={time.time()-t0_wall:.2f}s")
print(f"events_handled={handled} events_violated={violated}")
assert reached, "drone failed to reach target within timeout"
print("OK: single APE1 (base-avoidance-only) run reached target")
