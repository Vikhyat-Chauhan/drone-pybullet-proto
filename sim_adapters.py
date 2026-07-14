"""Plain-data replacements for CANavigator's ROS message types, plus
PyBullet-backed providers that nav_algorithm.py polls synchronously
instead of subscribing to. Ported from navigation/nav_algorithm_T.py's
_PoseSub/_ScanSub/_CloudSub/_EventSub -- duck-typed to the same
attributes/return shapes those classes exposed, so the decision logic
in nav_algorithm.py needs near-zero changes at the substitution sites.
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import pybullet as p


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    t0 = +2.0 * (w * z + x * y)
    t1 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t0, t1)


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class ScanMsg:
    """Replaces sensor_msgs/LaserScan -- same field names the original
    _ScanSub._sector_min/_window_vals read off msg.ranges/angle_min/
    angle_increment."""
    ranges: List[float]
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float


def sector_min(msg: Optional[ScanMsg], center_deg: float, half_width_deg: float) -> float:
    """Ported verbatim from _ScanSub._sector_min (nav_algorithm_T.py)."""
    if msg is None or not msg.ranges:
        return float("inf")
    n = len(msg.ranges)
    inc = msg.angle_increment
    if not math.isfinite(inc) or abs(inc) < 1e-9:
        center_idx = n // 2
        half = max(1, int(half_width_deg / 90.0 * n))
        lo = max(0, center_idx - half); hi = min(n - 1, center_idx + half)
    else:
        center = math.radians(center_deg); half = math.radians(half_width_deg)
        lo = int((center - half - msg.angle_min) / inc)
        hi = int((center + half - msg.angle_min) / inc)
        lo = max(0, min(n - 1, lo)); hi = max(0, min(n - 1, hi))
        if lo > hi: lo, hi = hi, lo
    window = [r for r in msg.ranges[lo:hi + 1] if math.isfinite(r) and r > 0.0]
    return min(window) if window else float("inf")


def window_vals(msg: Optional[ScanMsg], center_deg: float, half_width_deg: float) -> List[float]:
    """Ported verbatim from _ScanSub._window_vals (nav_algorithm_T.py)."""
    if msg is None or not msg.ranges:
        return []
    n = len(msg.ranges)
    inc = msg.angle_increment
    if not math.isfinite(inc) or abs(inc) < 1e-9:
        center_idx = n // 2
        half = max(1, int(half_width_deg / 90.0 * n))
        lo = max(0, center_idx - half); hi = min(n - 1, center_idx + half)
    else:
        center = math.radians(center_deg); half = math.radians(half_width_deg)
        lo = int((center - half - msg.angle_min) / inc)
        hi = int((center + half - msg.angle_min) / inc)
        lo = max(0, min(n - 1, lo)); hi = max(0, min(n - 1, hi))
        if lo > hi: lo, hi = hi, lo
    return [r for r in msg.ranges[lo:hi + 1] if math.isfinite(r) and r > 0.0]


class SimClock:
    """Step-count x fixed timestep. Every timing-sensitive module reads
    sim time through one shared instance of this instead of wall time --
    decouples experiment determinism from however fast p.stepSimulation()
    actually runs (GUI vs headless/no-sleep, machine load), matching the
    determinism Gazebo's simulated /clock gave the original ROS stack."""
    def __init__(self, dt: float):
        self._dt = dt
        self._steps = 0

    def tick(self) -> None:
        self._steps += 1

    def now(self) -> float:
        return self._steps * self._dt


class PoseProvider:
    """Replaces _PoseSub for a dynamic PyBullet body -- no polling thread
    needed, PyBullet state is always current in-process."""
    def __init__(self, body_id: int):
        self.body_id = body_id

    def latest(self) -> Tuple[float, float, float, float]:
        pos, orn = p.getBasePositionAndOrientation(self.body_id)
        yaw = yaw_from_quat(*orn)
        return (pos[0], pos[1], pos[2], yaw)


class StaticPoseProvider:
    """Replaces _PoseSub for a static target -- no PyBullet query needed
    since the target never moves."""
    def __init__(self, xyz: Tuple[float, float, float]):
        self._xyz = xyz

    def latest(self) -> Tuple[float, float, float]:
        return self._xyz


class TargetSignalProvider:
    """Search-and-rescue target sensor: replaces StaticPoseProvider's
    always-known ground truth with an omnidirectional (RSSI/thermal-style)
    detection gate. Holds the real target xyz internally (still needed for
    physically meaningful "reached" checks) but latest() returns None until
    the drone has come within detect_radius_m of it -- same (x, y, z) return
    shape as StaticPoseProvider.latest() once detected, so it drops into
    LidarTargetNavigatorCA's target_pose slot with no changes to
    nav_algorithm.py's _latest_target()/tick() fallback path.

    Detection is sticky (once_locked): once found, latest() keeps returning
    the location even if the drone later drifts back outside detect_radius_m,
    so search/homing mode doesn't flicker.
    """
    def __init__(self, xyz: Tuple[float, float, float], pose_provider: PoseProvider,
                 detect_radius_m: float = math.inf):
        self._xyz = xyz
        self._pose = pose_provider
        self._detect_radius_m = detect_radius_m
        self._locked = False

    def update(self) -> None:
        if self._locked:
            return
        x, y, z, _yaw = self._pose.latest()
        dx, dy, dz = self._xyz[0] - x, self._xyz[1] - y, self._xyz[2] - z
        if math.sqrt(dx * dx + dy * dy + dz * dz) <= self._detect_radius_m:
            self._locked = True

    def latest(self) -> Optional[Tuple[float, float, float]]:
        return self._xyz if self._locked else None

    def true_xyz(self) -> Tuple[float, float, float]:
        """Ground-truth target position, regardless of detection status
        -- used for metrics (e.g. straight-line distance for
        path-efficiency) that need the real target even before it's
        been sensed."""
        return self._xyz


class ScanProvider:
    """Replaces _ScanSub. update() must be called once per sim tick from
    the drive loop; latest() returns the cached (ScanMsg, t_last) pair,
    matching _ScanSub.latest()'s exact shape."""
    def __init__(self, lidar, pose_provider: PoseProvider, sim_clock: SimClock, range_min: float = 0.1):
        self._lidar = lidar
        self._pose = pose_provider
        self._clock = sim_clock
        self._range_min = range_min
        self._scan: Optional[ScanMsg] = None
        self._t_last = 0.0

    def update(self) -> None:
        pose = self._pose.latest()
        pairs = self._lidar.scan(pose[:3], yaw=pose[3])
        ranges = [r for _, r in pairs]
        angle_min = pairs[0][0]
        angle_increment = (pairs[-1][0] - pairs[0][0]) / max(1, len(pairs) - 1)
        self._scan = ScanMsg(ranges, angle_min, angle_increment, self._range_min, self._lidar.max_range)
        self._t_last = self._clock.now()

    def latest(self) -> Tuple[Optional[ScanMsg], float]:
        return self._scan, self._t_last


class CloudProvider:
    """Replaces _CloudSub. update() casts a multi-layer scan once per
    tick; latest() matches _CloudSub.latest()'s (ranges, n_ranges,
    n_layers, t_last) shape."""
    def __init__(self, lidar, pose_provider: PoseProvider, sim_clock: SimClock,
                 n_layers: int, vertical_angle_min: float, vertical_angle_increment: float):
        self._lidar = lidar
        self._pose = pose_provider
        self._clock = sim_clock
        self._n_layers = n_layers
        self._v_min = vertical_angle_min
        self._v_inc = vertical_angle_increment
        self._ranges: List[float] = []
        self._n_ranges = 0
        self._t_last = 0.0

    def update(self) -> None:
        pose = self._pose.latest()
        flat, n_ranges, n_layers = self._lidar.scan_multilayer(
            pose[:3], yaw=pose[3], n_layers=self._n_layers,
            vertical_angle_min=self._v_min, vertical_angle_increment=self._v_inc,
        )
        self._ranges = flat
        self._n_ranges = n_ranges
        self._t_last = self._clock.now()

    def latest(self) -> Tuple[List[float], int, int, float]:
        return self._ranges, self._n_ranges, self._n_layers, self._t_last


class EventQueue:
    """Multi-slot mailbox for newly-armed threat-arrival events. Used to
    be a single-slot (last-write-wins) mailbox back when events had no
    spatial existence and only one could be "in flight" at a time; now
    that threats.ThreatManager can keep several moving threats active
    concurrently, push() appends (defensively capped) and pop_new()
    atomically drains and returns everything pushed since the last call.
    A "__RESET__" kind still clears the queue without becoming a
    deliverable event."""
    def __init__(self, max_pending: int = 16):
        self._lock = threading.Lock()
        self._pending: List[Dict] = []
        self._max_pending = max_pending

    def push(self, evt: Dict) -> None:
        with self._lock:
            if evt.get("kind") == "__RESET__":
                self._pending.clear()
                return
            self._pending.append(evt)
            if len(self._pending) > self._max_pending:
                self._pending.pop(0)

    def clear(self) -> None:
        with self._lock:
            self._pending.clear()

    def pop_new(self) -> List[Dict]:
        """Drains and returns every event pushed since the last call, in
        arrival order."""
        with self._lock:
            out, self._pending = self._pending, []
            return out

    def pop(self) -> Optional[Dict]:
        """Back-compat single-event accessor (oldest pending event, if
        any) for callers that only care about one event at a time."""
        with self._lock:
            if not self._pending:
                return None
            return self._pending.pop(0)


class ThreatSensorProvider:
    """Proxy 'camera' threat sensor: a forward-facing cone with limited
    range, modeled on TargetSignalProvider's detection-gate pattern but
    for moving threats (threats.py::ThreatManager) instead of the
    search-and-rescue target. update(threats) must be called once per
    sim tick with the current list of active threats
    (ThreatManager.active()); latest() returns the subset within
    range/FOV as {id, range_m, bearing_rad, closing_speed_mps, radius_m}
    dicts, sorted by ascending range -- threats outside the cone or
    range are simply invisible until they enter it, which is what makes
    genuine misses (and a too-slow planner missing a threat already
    inside the cone) possible.

    closing_speed_mps is computed from the threat's own velocity
    projected onto the line-of-sight only -- the drone's own velocity
    doesn't factor in, a deliberate simplification that keeps this a
    pure per-threat calc instead of a full relative-velocity one."""
    def __init__(self, pose_provider: PoseProvider, detect_range_m: float, fov_deg: float):
        self._pose = pose_provider
        self._range = detect_range_m
        self._half_fov = math.radians(fov_deg) / 2.0
        self._visible: List[Dict] = []

    def update(self, threats: List[Dict]) -> None:
        x, y, _z, yaw = self._pose.latest()
        out = []
        for th in threats:
            dx, dy = th["x"] - x, th["y"] - y
            rng = math.hypot(dx, dy)
            if rng > self._range or rng < 1e-6:
                continue
            bearing = _wrap_pi(math.atan2(dy, dx) - yaw)
            if abs(bearing) > self._half_fov:
                continue
            closing_speed = -(dx * th["vx"] + dy * th["vy"]) / rng
            out.append({
                "id": th["id"], "range_m": rng, "bearing_rad": bearing,
                "closing_speed_mps": closing_speed, "radius_m": th["radius"],
            })
        out.sort(key=lambda d: d["range_m"])
        self._visible = out

    def latest(self) -> List[Dict]:
        return self._visible
