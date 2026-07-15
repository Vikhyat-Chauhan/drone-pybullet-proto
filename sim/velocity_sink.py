"""Drop-in replacement for CANavigator's transport.py::GzVelPub.

DronePhysics already produces fully actuator-shaped (vx, vy, vz, wz)
output -- it IS the dynamics model -- so we set that velocity directly on
the PyBullet body (kinematic) rather than applying forces/torques, which
would double-apply dynamics on top of what DronePhysics already computed.

IMPORTANT: (vx, vy) from DronePhysics/nav/algorithm.py are BODY-FRAME (vy
is always 0 here; only forward+yaw-rate are used). PyBullet's
resetBaseVelocity takes WORLD-frame velocity, unlike Gazebo's
velocity-command plugin which rotated body-frame internally, so we rotate
by yaw here ourselves -- otherwise the drone flies a fixed world direction
regardless of heading.
"""
import math
import pybullet as p


class PyBulletVelPub:
    def __init__(self, body_id: int):
        self.body_id = body_id

    def send(self, lin_xyz, ang_zyx=(0.0, 0.0, 0.0)) -> None:
        vx, vy, vz = lin_xyz
        _, orn = p.getBasePositionAndOrientation(self.body_id)
        yaw = p.getEulerFromQuaternion(orn)[2]
        world_vx = vx * math.cos(yaw) - vy * math.sin(yaw)
        world_vy = vx * math.sin(yaw) + vy * math.cos(yaw)
        p.resetBaseVelocity(self.body_id, linearVelocity=[world_vx, world_vy, vz],
                             angularVelocity=list(ang_zyx))
