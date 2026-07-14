#!/usr/bin/env python3
# Port of CANavigator's tools/event_emitter.py::EventEmitter.
#
# The original ran as an rclpy.Node with its own background thread
# polling a ROS /clock-derived sim time every 10ms and publishing
# std_msgs/String events. That's replaced here with a plain class whose
# step() method is called once per sim tick from the main drive loop
# (see sim_adapters.EventQueue for the consumer side) -- no polling
# thread, no ROS topic. Deterministic mode (the only mode main.py
# actually uses) needs no wall-clock/pose input at all: it's a pure
# logical clock driven by SimClock ticks, so this is strictly simpler
# than the original's threaded design, not just a port.
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Dict, Optional
import json, math, random, csv, os

from sim_adapters import SimClock, EventQueue


@dataclass
class EventCfg:
    seed: Optional[int] = None
    event_deterministic: bool = False

    dt_min_s: float = 0.02
    dt_max_s: float = 4.0

    deadline_alpha: float = 0.85
    deadline_min_s: float = 0.147
    deadline_max_s: float = 3.50
    global_deadline_s: Optional[float] = field(default=None, init=False)

    mix_enemy: float = 0.33
    mix_obstacle: float = 0.33
    mix_lane: float = 0.34

    log_csv_path: Optional[str] = "logs/events_log.csv"


class EventEmitter:
    """Deterministic (or pose-driven non-deterministic) event generator.

    Deterministic mode: emission timing/types/deadlines are purely
    RNG-based from `seed`, driven by a logical clock advanced via
    SimClock -- matches the original's "ignores drone pose" deterministic
    path exactly.

    Non-deterministic mode: uses sim-time deltas; push_pose() must be
    called each tick to keep drone pose current (only used by _choose_kind
    metadata in the original -- pose isn't actually read there either,
    kept for API parity).
    """

    def __init__(self, events_out: EventQueue, sim_clock: SimClock, gen_cfg: Optional[EventCfg] = None):
        self._cfg = gen_cfg or EventCfg()
        self._rnd = random.Random(self._cfg.seed)
        self._out = events_out
        self._sim_clock = sim_clock

        self._pose_latest = None

        self._t_logical = 0.0
        self._i = 0

        # Side-channel for GUI event-marker rendering (main.py): observing
        # this must never affect EventQueue's own pop-based consumption,
        # which nav_algorithm.py's tick() owns exclusively.
        self.last_event: Optional[Dict[str, Any]] = None
        self.last_emit_seq: int = 0

        self._csv_fp = None
        if self._cfg.log_csv_path:
            os.makedirs(os.path.dirname(self._cfg.log_csv_path) or ".", exist_ok=True)
            self._csv_fp = open(self._cfg.log_csv_path, "w", newline="")
            self._csv = csv.writer(self._csv_fp)
            self._csv.writerow(["t_emit", "kind", "deadline_s", "meta_json"])

        # Deterministic-mode scheduling state
        self._sim_t_next: Optional[float] = None
        self._next_dt: float = self._draw_dt()

        # Non-deterministic-mode scheduling state
        self._last_sim_t: Optional[float] = None
        self._nondet_dt: float = self._draw_dt()

    # ---------------- Lifecycle ----------------
    def reset(self) -> None:
        """Deterministic reset: rewind RNG/clock/index; push a __RESET__
        sentinel so the navigator's EventQueue is cleared of any stale
        event from the previous run before new events begin arriving."""
        self._rnd = random.Random(self._cfg.seed)
        self._t_logical = 0.0
        self._i = 0
        self._sim_t_next = None
        self._next_dt = self._draw_dt()
        self._last_sim_t = None
        self._nondet_dt = self._draw_dt()
        self._out.push({"kind": "__RESET__", "deadline_s": 0.0, "t_emit": -1.0, "meta": {}})

    def close(self) -> None:
        if self._csv_fp:
            try:
                self._csv_fp.flush()
            finally:
                self._csv_fp.close()
            self._csv_fp = None

    # ---------------- Internals ----------------
    def push_pose(self, x: float, y: float, z: float, t: float) -> None:
        self._pose_latest = (x, y, z, t)

    def _draw_dt(self) -> float:
        return math.exp(self._rnd.uniform(math.log(self._cfg.dt_min_s), math.log(self._cfg.dt_max_s)))

    def _deadline_from_dt(self, delta_t: float) -> float:
        if self._cfg.global_deadline_s is not None:
            return float(self._cfg.global_deadline_s)
        d = self._cfg.deadline_alpha * float(delta_t)
        return max(self._cfg.deadline_min_s, min(self._cfg.deadline_max_s, d))

    def _choose_kind(self) -> str:
        r = self._rnd.random()
        if r < self._cfg.mix_enemy: return "ENEMY"
        if r < self._cfg.mix_enemy + self._cfg.mix_obstacle: return "SUDDEN_OBSTACLE"
        return "LANE_BLOCK"

    def _make_meta_nondet(self, kind: str) -> Dict[str, Any]:
        if kind == "ENEMY":
            return {"speed": round(self._rnd.uniform(1.0, 4.0), 3)}
        if kind == "SUDDEN_OBSTACLE":
            return {"radius": round(self._rnd.uniform(1.0, 2.5), 3)}
        w = self._rnd.uniform(6.0, 12.0)
        h = self._rnd.uniform(4.0, 10.0)
        return {"rect_wh": [round(w, 3), round(h, 3)]}

    def _emit(self, kind: str, t_emit: float, deadline: float, meta: Dict[str, Any]) -> None:
        obj = {"kind": kind, "t_emit": t_emit, "t_recv": self._sim_clock.now(), "deadline_s": deadline, "meta": meta}
        self._out.push(obj)
        self.last_event = obj
        self.last_emit_seq += 1
        if self._csv_fp:
            try:
                self._csv.writerow([t_emit, kind, deadline, json.dumps(meta)])
            except Exception:
                pass

    # ---------------- Main step (called once per sim tick) ----------------
    def step(self) -> None:
        now_sim = self._sim_clock.now()

        if self._cfg.event_deterministic:
            if self._sim_t_next is None:
                self._sim_t_next = now_sim + self._next_dt
                return
            if now_sim < self._sim_t_next:
                return

            dt = self._next_dt
            self._t_logical += dt
            t_emit = self._t_logical
            kind = self._choose_kind()
            meta = {"i": self._i, "kind": kind}
            self._i += 1
            deadline = self._deadline_from_dt(dt)

            self._next_dt = self._draw_dt()
            self._sim_t_next = now_sim + self._next_dt

            self._emit(kind, t_emit, deadline, meta)
        else:
            if self._last_sim_t is None:
                self._last_sim_t = now_sim
                return
            if now_sim - self._last_sim_t < self._nondet_dt:
                return

            delta_t = now_sim - self._last_sim_t
            self._last_sim_t = now_sim
            t_emit = now_sim
            kind = self._choose_kind()
            meta = self._make_meta_nondet(kind)
            deadline = self._deadline_from_dt(delta_t)

            self._nondet_dt = self._draw_dt()
            self._emit(kind, t_emit, deadline, meta)
