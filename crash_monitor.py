#!/usr/bin/env python3
# Spatial collision detection between the drone and active moving threats
# (threats.py::ThreatManager). Modeled on violations.py::ViolationMonitor's
# armed-per-entity state machine (one violation per intrusion, not per
# tick) but keyed dynamically by threat id instead of a fixed rectangle
# list, since threats spawn and retire over a mission's lifetime. This is
# the ground-truth "did the drone actually get hit" signal -- independent
# of nav_algorithm.py's deadline-race bookkeeping (events_violated), which
# only tracks whether a planner finished in time, not whether the
# resulting maneuver (or lack of one) actually kept the drone clear.

import math
import logging
from threading import Lock
from typing import Any, Dict, List, Optional, Set


class CrashMonitor:
    def __init__(self, sim_clock, drone_radius_m: float, crash_margin_m: float = 0.15):
        self._logger = logging.getLogger(__name__)
        self._lock = Lock()
        self._sim_clock = sim_clock
        self._drone_radius_m = drone_radius_m
        self._crash_margin_m = crash_margin_m

        self._armed: Dict[int, bool] = {}
        self._inside_prev: Dict[int, bool] = {}
        self._total_crashes: int = 0

        self._segment_label: str = "run"
        self._segment_start_wall: float = 0.0
        self._last_pose: tuple = (0.0, 0.0)

    # --------------------- Pose/threat intake ----------------------
    def push_pose(self, x: float, y: float, z: float, t: float) -> None:
        with self._lock:
            self._last_pose = (x, y)

    def push_threats(self, threats: List[Dict[str, Any]]) -> None:
        with self._lock:
            x, y = self._last_pose
            seen_ids: Set[int] = set()
            for th in threats:
                tid = th["id"]
                seen_ids.add(tid)
                if tid not in self._armed:
                    self._armed[tid] = True
                    self._inside_prev[tid] = False

                dist = math.hypot(th["x"] - x, th["y"] - y)
                now_colliding = dist < (self._drone_radius_m + th["radius"] + self._crash_margin_m)
                was_colliding = self._inside_prev[tid]
                self._inside_prev[tid] = now_colliding

                if was_colliding and not now_colliding:
                    self._armed[tid] = True
                    continue
                if not was_colliding and now_colliding and self._armed[tid]:
                    self._total_crashes += 1
                    self._armed[tid] = False

            # Drop bookkeeping for threats ThreatManager has already
            # retired, so this dict doesn't grow unbounded over a long run.
            for tid in list(self._armed.keys()):
                if tid not in seen_ids:
                    self._armed.pop(tid, None)
                    self._inside_prev.pop(tid, None)

    # --------------------- Public controls ----------------------
    def mark_run_start(self, label: str = "run"):
        with self._lock:
            self._segment_label = label
            self._segment_start_wall = self._sim_clock.now()
            self._total_crashes = 0
            self._armed = {}
            self._inside_prev = {}

    def log_and_reset(self, label: Optional[str] = None) -> Dict[str, Any]:
        with self._lock:
            lbl = label if label is not None else self._segment_label
            summary = {
                "label": lbl,
                "wall_started_at": round(self._segment_start_wall, 3),
                "total_crashes": int(self._total_crashes),
                "last_pose": {"x": round(self._last_pose[0], 3), "y": round(self._last_pose[1], 3)},
                "rule": "one_crash_per_intrusion",
            }
            self._logger.info(summary, extra={"type": "CRASHSUMMARY"})

            self._segment_label = "run"
            self._segment_start_wall = self._sim_clock.now()
            self._total_crashes = 0
            self._armed = {}
            self._inside_prev = {}
            return summary
