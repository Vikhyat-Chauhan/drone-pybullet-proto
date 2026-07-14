"""Drop-in replacement for CANavigator's transport.py::GzVelPub.

DronePhysics already produces fully actuator-shaped (vx, vy, vz, wz)
output (drag, wind, jerk/tilt caps applied internally) -- it IS the
dynamics model. We set that velocity directly on the PyBullet body
(kinematic) rather than applying forces/torques, which would double-
apply dynamics on top of what DronePhysics already computed. This
mirrors how the original drove Gazebo: cmd_vel *velocity* commands,
not force commands.

IMPORTANT: (vx, vy) from DronePhysics/nav_algorithm.py are BODY-FRAME
(vx = forward speed, vy always 0 in this codebase -- only forward+yaw-
rate are actually used). Gazebo's multicopter velocity-command plugin
rotates body-frame cmd_vel into world frame internally using the
model's own orientation; PyBullet's resetBaseVelocity takes WORLD-frame
linear velocity directly, so we must do that rotation here ourselves --
otherwise the drone flies in a fixed world direction regardless of its
heading.
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
