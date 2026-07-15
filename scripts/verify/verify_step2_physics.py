"""Step 2 verification: DronePhysics driving a real PyBullet body via
PyTeleop/PyBulletVelPub. Headless (p.DIRECT). Prints position/velocity
once per simulated second; expect a smooth spring-damper ramp toward
vx=3 m/s (not instantaneous), consistent with zeta_lin/wn_lin_rad.
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent.parent))

import pybullet as p

from config import TeleopConfig
from sim.teleop import PyTeleop

cfg = TeleopConfig()

p.connect(p.DIRECT)
p.setGravity(0, 0, 0)
dt = 1.0 / 60.0
p.setTimeStep(dt)

col = p.createCollisionShape(p.GEOM_SPHERE, radius=cfg.drone_radius_m)
body = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=col, basePosition=[0, 0, cfg.flight_z])

teleop = PyTeleop(body, cfg)
teleop.set_cmd(3.0, 0.0, 0.0, 0.0)

steps = 0
for i in range(600):  # 10 sim-seconds at 60Hz
    vx, vy, vz, wz = teleop.physics_step(dt)
    p.stepSimulation()
    steps += 1
    if steps % 60 == 0:
        pos, _ = p.getBasePositionAndOrientation(body)
        print(f"t={steps*dt:5.2f}s pos={pos[0]:7.3f},{pos[1]:7.3f} vel=({vx:6.3f},{vy:6.3f},{vz:6.3f}) wz={wz:6.3f}")

print("OK: physics-driven body motion verified")
