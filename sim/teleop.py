"""Port of CANavigator's navigation/teleop.py::GzTeleop -> PyTeleop.

The original ran a free-running wall-clock daemon thread publishing at
cfg.rate_hz, independent of Gazebo's step loop -- wrong for headless
PyBullet batch mode, which wants to step physics as fast as possible with
no wall-clock sleep. Threading is stripped: the drive loop calls
physics_step(dt) synchronously once per PyBullet step. set_cmd()/stop()
keep their original names/semantics for a minimal-diff port of
nav/algorithm.py.
"""
from __future__ import annotations

import logging
import threading
from typing import Tuple

from sim.velocity_sink import PyBulletVelPub
from sim.physics import DronePhysics


class PyTeleop:
    def __init__(self, body_id: int, cfg):
        if getattr(cfg, "rate_hz", 0) <= 0:
            raise ValueError(f"rate_hz must be > 0, got {cfg.rate_hz}")

        self.cfg = cfg
        self._logger = logging.getLogger(__name__)
        self._pub = PyBulletVelPub(body_id)
        self._physics = DronePhysics(cfg)
        self._lock = threading.Lock()

    def set_cmd(self, vx: float, vy: float, vz: float, wz: float) -> None:
        with self._lock:
            self._physics.update_cmd(vx, vy, vz, wz)

    def physics_step(self, dt: float) -> Tuple[float, float, float, float]:
        """Synchronous replacement for the threaded _publish_loop; call once
        per sim tick from the drive loop."""
        with self._lock:
            vx, vy, vz, wz = self._physics.step(dt)
        self._pub.send((vx, vy, vz), (0.0, 0.0, wz))
        return vx, vy, vz, wz

    def publish_once(self, linear, angular=(0.0, 0.0, 0.0)) -> None:
        self._pub.send(linear, angular)

    def stop(self) -> None:
        self._logger.info({"event": "STOP"})
        self.set_cmd(0.0, 0.0, 0.0, 0.0)
        self._physics.reset()
