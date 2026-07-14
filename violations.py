#!/usr/bin/env python3
# Port of CANavigator's tools/violations.py::ViolationMonitor.
# Node/subscription plumbing stripped; push_pose(x, y, z, t) replaces the
# ROS pose-subscription callback -- caller (main.py drive loop) calls it
# once per sim tick. load_rects()/point_in_rect() and the visit-counting
# state machine are ported verbatim.

import json, os
from typing import List, Tuple, Optional, Dict, Any
import logging
from threading import Lock

META_PATH = os.path.join("models", "generated", "generated_nofly_meta.json")


def load_rects(meta_path: str) -> List[Tuple[float, float, float, float]]:
    """Return list of rectangles as (cx, cy, w, h) from meta JSON.

    Returns an empty list if the file doesn't exist yet (e.g. before the
    first arena generation pass has run).
    """
    if not os.path.exists(meta_path):
        return []
    with open(meta_path, "r") as f:
        meta = json.load(f)
    rects = meta.get("rectangles_xywh", [])
    return [tuple(map(float, r)) for r in rects]


def point_in_rect(x: float, y: float, cx: float, cy: float, w: float, h: float) -> bool:
    """Axis-aligned membership (no padding)."""
    half_w = max(0.0, w * 0.5)
    half_h = max(0.0, h * 0.5)
    return (cx - half_w) <= x <= (cx + half_w) and (cy - half_h) <= y <= (cy + half_h)


class ViolationMonitor:
    """
    Tracks:
      - exactly one violation per visit (first ENTER after being outside) per rectangle

    QUIET mode:
      - No per-event logs during flight.
      - Call log_and_reset() to emit a VIOLATIONSUMMARY and clear counters for the next run.
    """

    def __init__(self, sim_clock, meta_path: str = META_PATH):
        self._logger = logging.getLogger(__name__)
        self._lock = Lock()
        self._sim_clock = sim_clock

        self._meta_path = meta_path
        self._rects = load_rects(meta_path)
        n = len(self._rects)

        self._armed: List[bool] = [True] * n
        self._inside_prev: List[bool] = [False] * n
        self._violations_per_rect: List[int] = [0] * n

        self._total_violations: int = 0
        self._segment_label: str = "run"
        self._segment_start_wall: float = 0.0
        self._last_pose: Tuple[float, float] = (0.0, 0.0)

        self._logger.info(
            {"boxes": n, "mode": "quiet(log on demand)", "rule": "one_violation_per_visit"},
            extra={"type": "LOADEDBOXES"},
        )

    # --------------------- Pose intake ----------------------
    def push_pose(self, x: float, y: float, z: float, t: float) -> None:
        with self._lock:
            for i, (cx, cy, w, h) in enumerate(self._rects):
                now_in = point_in_rect(x, y, cx, cy, w, h)

                was_in = self._inside_prev[i]
                self._inside_prev[i] = now_in

                if was_in and not now_in:
                    self._armed[i] = True
                    continue

                if not was_in and now_in and self._armed[i]:
                    self._violations_per_rect[i] += 1
                    self._total_violations += 1
                    self._armed[i] = False

            self._last_pose = (x, y)

    # --------------------- Public controls ----------------------
    def reload_rects(self, meta_path: Optional[str] = None):
        with self._lock:
            self._meta_path = meta_path if meta_path is not None else self._meta_path
            self._rects = load_rects(self._meta_path)
            n = len(self._rects)

            self._armed = [True] * n
            self._inside_prev = [False] * n
            self._violations_per_rect = [0] * n
            self._total_violations = 0

            self._logger.info(
                {"boxes": n, "mode": "quiet(log on demand)", "rule": "one_violation_per_visit"},
                extra={"type": "LOADEDBOXES"},
            )

    def mark_run_start(self, label: str = "run"):
        with self._lock:
            self._segment_label = label
            self._segment_start_wall = self._sim_clock.now()
            self._total_violations = 0
            for i in range(len(self._violations_per_rect)):
                self._violations_per_rect[i] = 0

            n = len(self._rects)
            self._armed = [True] * n
            self._inside_prev = [False] * n

    def _build_summary(self, label: str, include_boxes: bool) -> Dict[str, Any]:
        summary: Dict[str, Any] = {
            "label": label,
            "wall_started_at": round(self._segment_start_wall, 3),
            "total_violations": int(self._total_violations),
            "last_pose": {"x": round(self._last_pose[0], 3), "y": round(self._last_pose[1], 3)},
            "rule": "one_violation_per_visit",
        }
        if include_boxes:
            per_box = []
            for i, (cx, cy, w, h) in enumerate(self._rects):
                per_box.append({
                    "rect_idx": i,
                    "center": {"x": round(cx, 3), "y": round(cy, 3)},
                    "w": round(w, 3),
                    "h": round(h, 3),
                    "violations": int(self._violations_per_rect[i]),
                })
            summary["per_box"] = per_box
        return summary

    def log_and_reset(self, label: Optional[str] = None, include_boxes: bool = False) -> Dict[str, Any]:
        with self._lock:
            lbl = label if label is not None else self._segment_label
            summary = self._build_summary(lbl, include_boxes)

            self._logger.info(summary, extra={"type": "VIOLATIONSUMMARY"})

            self._segment_label = "run"
            self._segment_start_wall = self._sim_clock.now()
            self._total_violations = 0
            for i in range(len(self._violations_per_rect)):
                self._violations_per_rect[i] = 0
            n = len(self._rects)
            self._armed = [True] * n
            self._inside_prev = [False] * n

            return summary
