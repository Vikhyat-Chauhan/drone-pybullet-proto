#!/usr/bin/env python3
"""Real moving-obstacle ("threat") generation and kinematics.

Replaces the old purely-timing model (event_source.py::EventEmitter
pushing bare {kind, deadline_s} dicts with no spatial existence at all)
with actual world-frame objects that have a position and velocity,
spawned ahead of the drone and biased (per-kind) to intercept its path.
This is what makes "handling an event" mean something physical: each APE
now has to sense a real threat (sim_adapters.ThreatSensorProvider) and
steer to keep clear of it within its native compute-latency budget, not
just race a synthetic clock deadline.

The inter-arrival cadence and kind mix (log-uniform dt, ENEMY/
SUDDEN_OBSTACLE/LANE_BLOCK weights) reimplement event_source.py's
EventEmitter formulas directly rather than wrapping an EventEmitter
instance -- EventEmitter is built around pushing self-contained {kind,
deadline_s, meta} dicts with its own CSV logging and reset semantics,
which doesn't compose cleanly with owning live Threat objects that must
be advanced every tick and looked up by id. The cadence math itself
(_draw_dt/_choose_kind below) is copied verbatim from EventEmitter's.
"""
from __future__ import annotations

import csv
import json
import math
import os
import random
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from sim_adapters import EventQueue, SimClock


@dataclass
class Threat:
    id: int
    kind: str
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    spawn_t: float
    deadline_s: float
    t_recv: float

    def as_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id, "kind": self.kind, "x": self.x, "y": self.y,
            "vx": self.vx, "vy": self.vy, "radius": self.radius,
            "spawn_t": self.spawn_t, "deadline_s": self.deadline_s,
            "t_recv": self.t_recv,
        }


@dataclass
class ThreatCfg:
    seed: Optional[int] = None

    # Inter-arrival cadence (log-uniform, same distribution shape as the
    # old EventEmitter) and kind mix.
    dt_min_s: float = 0.02
    dt_max_s: float = 0.60
    mix_enemy: float = 0.33
    mix_obstacle: float = 0.33
    mix_lane: float = 0.34

    max_active_threats: int = 3
    spawn_range_min_m: float = 10.0
    spawn_range_max_m: float = 18.0
    lead_time_s: float = 1.5
    retire_margin_m: float = 5.0
    max_lifetime_s: float = 20.0
    # A still-approaching threat is re-armed (a fresh deadline race
    # triggered) at most this often -- without it, a threat only ever
    # gets a single one-shot avoidance decision no matter how long it
    # continues closing, which understates a real embedded system's
    # continuous sense-plan-act cycle. Also sets how finely the deadline
    # race tracks a closing threat: at the original 0.3s, a fast ENEMY
    # (closing ~8-16 m/s) advances ~2.4-4.8m between re-arms -- coarser
    # than the ~1.5m collision buffer (drone_radius + threat_radius +
    # crash_margin), so the tight final-approach deadline window could be
    # stepped over entirely between samples. Lowering this must stay well
    # above nav_algorithm.py's own tick period (1/GoToConfig.rate_hz,
    # ~33ms at the default 30Hz): drop below that and, with up to
    # max_active_threats concurrently re-arming on independent schedules,
    # multiple re-arms land in the SAME nav tick often enough to trigger
    # spurious PREEMPTIVE violations regardless of APE speed (a brand
    # -new arrival always preempts a proposal that's had 0ms to become
    # ready) -- confirmed empirically to swamp the real deadline-timing
    # signal at 0.05s. 0.15s (~4.5x the tick period) keeps re-arms rare
    # enough per tick to avoid that while still landing inside the
    # collision buffer for the fastest threats.
    rearm_cooldown_s: float = 0.15

    # Deadline clamp bounds are per-kind, not one shared [min, max] pair.
    # The raw value (range_m / closing_speed_mps -- see on_sensed()) stays
    # a genuine physically-derived reaction budget, but a shared clamp
    # washes out the kind signal in practice: with one sensor cone/range
    # for every kind, most encounters (of any speed) are geometrically
    # tangential/grazing rather than head-on, so their radial closing
    # speed sits near zero right at closest approach regardless of the
    # threat's own speed -- the raw formula alone doesn't reliably
    # separate "fast" from "slow" kinds event-to-event. Per-kind bounds
    # make that separation deliberate: ENEMY's window sits entirely below
    # APE3's compute budget (always too fast for the slow/high-quality
    # planner), APE2's straddles it (a coin-flip), well above APE1's;
    # SUDDEN_OBSTACLE straddles APE3's budget; LANE_BLOCK's window sits
    # above all three (always makeable). See docs/POWER_MODEL.md and
    # config.py's threat_* fields for the APE budget numbers this is
    # calibrated against.
    enemy_deadline_min_s: float = 0.05
    enemy_deadline_max_s: float = 0.15
    obstacle_deadline_min_s: float = 0.15
    obstacle_deadline_max_s: float = 0.40
    lane_deadline_min_s: float = 0.40
    lane_deadline_max_s: float = 0.65

    enemy_speed_min_mps: float = 8.0
    enemy_speed_max_mps: float = 16.0
    enemy_radius_m: float = 0.6
    enemy_lead_bias: float = 0.85

    obstacle_speed_min_mps: float = 3.0
    obstacle_speed_max_mps: float = 7.0
    obstacle_radius_min_m: float = 1.2
    obstacle_radius_max_m: float = 2.5
    obstacle_lead_bias: float = 0.4

    lane_speed_min_mps: float = 0.0
    lane_speed_max_mps: float = 1.0
    lane_radius_min_m: float = 2.0
    lane_radius_max_m: float = 3.5
    lane_lead_bias: float = 0.0

    log_csv_path: Optional[str] = "logs/events_log.csv"


class ThreatManager:
    """Owns the set of currently-active moving threats. step() must be
    called once per sim tick from the drive loop with the drone's
    current world pose; it advances existing threats, retires ones that
    have passed behind the drone (successfully evaded/passed) or
    exceeded their lifetime, and spawns new ones -- up to
    `max_active_threats` -- pushing each newly-armed threat into the
    shared EventQueue for nav_algorithm.py to react to. Never
    force-evicts an unresolved threat to make room for a new one: if the
    cap is already reached when a spawn is due, that spawn attempt is
    simply skipped and cadence continues (retried on the next draw)."""

    def __init__(self, events_out: EventQueue, sim_clock: SimClock,
                 threat_cfg: Optional[ThreatCfg] = None):
        self._cfg = threat_cfg or ThreatCfg()
        self._out = events_out
        self._sim_clock = sim_clock
        self._rnd = random.Random(self._cfg.seed)

        self._active: Dict[int, Threat] = {}
        self._next_id: int = 0
        self._armed_at: Dict[int, float] = {}

        self._last_step_t: Optional[float] = None
        self._prev_drone_xy: Optional[tuple] = None
        self._drone_vx: float = 0.0
        self._drone_vy: float = 0.0

        self._sim_t_next: Optional[float] = None
        self._next_dt: float = self._draw_dt()

        self._csv_fp = None
        if self._cfg.log_csv_path:
            os.makedirs(os.path.dirname(self._cfg.log_csv_path) or ".", exist_ok=True)
            self._csv_fp = open(self._cfg.log_csv_path, "w", newline="")
            self._csv = csv.writer(self._csv_fp)
            self._csv.writerow(["t_emit", "kind", "deadline_s", "meta_json"])

    # ---------------- Lifecycle ----------------
    def reset(self) -> None:
        self._active.clear()
        self._armed_at.clear()
        self._rnd = random.Random(self._cfg.seed)
        self._sim_t_next = None
        self._next_dt = self._draw_dt()
        self._last_step_t = None
        self._prev_drone_xy = None
        self._next_id = 0
        self._out.push({"kind": "__RESET__", "deadline_s": 0.0, "t_emit": -1.0, "meta": {}})

    def close(self) -> None:
        if self._csv_fp:
            try:
                self._csv_fp.flush()
            finally:
                self._csv_fp.close()
            self._csv_fp = None

    # ---------------- Accessors ----------------
    def active(self) -> List[Dict[str, Any]]:
        return [t.as_dict() for t in self._active.values()]

    # ---------------- Cadence (ported from event_source.EventEmitter) ----------------
    def _draw_dt(self) -> float:
        return math.exp(self._rnd.uniform(math.log(self._cfg.dt_min_s), math.log(self._cfg.dt_max_s)))

    def _choose_kind(self) -> str:
        r = self._rnd.random()
        if r < self._cfg.mix_enemy:
            return "ENEMY"
        if r < self._cfg.mix_enemy + self._cfg.mix_obstacle:
            return "SUDDEN_OBSTACLE"
        return "LANE_BLOCK"

    def _kind_params(self, kind: str):
        c = self._cfg
        if kind == "ENEMY":
            return c.enemy_speed_min_mps, c.enemy_speed_max_mps, c.enemy_radius_m, c.enemy_radius_m, c.enemy_lead_bias
        if kind == "SUDDEN_OBSTACLE":
            return (c.obstacle_speed_min_mps, c.obstacle_speed_max_mps,
                    c.obstacle_radius_min_m, c.obstacle_radius_max_m, c.obstacle_lead_bias)
        return c.lane_speed_min_mps, c.lane_speed_max_mps, c.lane_radius_min_m, c.lane_radius_max_m, c.lane_lead_bias

    def _deadline_bounds(self, kind: str):
        c = self._cfg
        if kind == "ENEMY":
            return c.enemy_deadline_min_s, c.enemy_deadline_max_s
        if kind == "SUDDEN_OBSTACLE":
            return c.obstacle_deadline_min_s, c.obstacle_deadline_max_s
        return c.lane_deadline_min_s, c.lane_deadline_max_s

    # ---------------- Spawn ----------------
    def _spawn_threat(self, kind: str, drone_x: float, drone_y: float, drone_yaw: float, now: float) -> None:
        """Creates the threat's kinematic state only -- does NOT arm an
        event yet. A threat spawns beyond the proxy sensor's range
        (spawn_range_min/max_m, typically > threat_sensor_range_m) so it
        has no spatial existence to the navigator until it's actually
        sensed; see on_sensed() for where the deadline-race event is
        armed, using the threat's real sensed range/closing_speed at
        that moment rather than its spawn-time kinematics."""
        c = self._cfg
        rng = self._rnd

        spawn_range = rng.uniform(c.spawn_range_min_m, c.spawn_range_max_m)
        bearing = drone_yaw + rng.uniform(-math.pi / 2.0, math.pi / 2.0)
        sx = drone_x + spawn_range * math.cos(bearing)
        sy = drone_y + spawn_range * math.sin(bearing)

        v_min, v_max, r_min, r_max, lead_bias = self._kind_params(kind)
        speed = rng.uniform(v_min, v_max)
        radius = rng.uniform(r_min, r_max)

        rand_theta = rng.uniform(0.0, 2.0 * math.pi)
        rand_dir = (math.cos(rand_theta), math.sin(rand_theta))

        future_x = drone_x + self._drone_vx * c.lead_time_s
        future_y = drone_y + self._drone_vy * c.lead_time_s
        ix, iy = future_x - sx, future_y - sy
        idist = math.hypot(ix, iy)
        intercept_dir = (ix / idist, iy / idist) if idist > 1e-6 else rand_dir

        bias = max(0.0, min(1.0, lead_bias))
        dirx = (1.0 - bias) * rand_dir[0] + bias * intercept_dir[0]
        diry = (1.0 - bias) * rand_dir[1] + bias * intercept_dir[1]
        dnorm = math.hypot(dirx, diry)
        if dnorm < 1e-6:
            dirx, diry = intercept_dir
        else:
            dirx, diry = dirx / dnorm, diry / dnorm

        vx, vy = speed * dirx, speed * diry

        tid = self._next_id
        self._next_id += 1
        # deadline_s is a placeholder until on_sensed() recomputes it from
        # the real sensed range/closing_speed at detection time.
        _, kind_deadline_max_s = self._deadline_bounds(kind)
        th = Threat(id=tid, kind=kind, x=sx, y=sy, vx=vx, vy=vy, radius=radius,
                    spawn_t=now, deadline_s=kind_deadline_max_s, t_recv=now)
        self._active[tid] = th

    # ---------------- Detection-triggered arming ----------------
    def on_sensed(self, visible_threats: List[Dict[str, Any]]) -> None:
        """Call once per tick with the current ThreatSensorProvider
        output (range_m/bearing_rad/closing_speed_mps already computed
        against real drone pose). Arms a deadline-race event the first
        tick a threat becomes sensor-visible -- this is what makes
        `deadline_s` a genuine reaction-time budget (computed from the
        threat's real range/closing_speed at the moment it's actually
        detectable) instead of an arbitrary spawn-time distance that
        might be well outside the sensor's range/FOV. A still-closing
        threat is re-armed (a fresh race triggered) every
        rearm_cooldown_s -- without this, a threat that keeps
        approaching after its first (possibly insufficient) evasive
        command would never get re-evaluated, understating a real
        embedded system's continuous sense-plan-act cycle."""
        now = self._sim_clock.now()
        for vt in visible_threats:
            tid = vt["id"]
            closing_speed = float(vt["closing_speed_mps"])
            last_armed = self._armed_at.get(tid)
            if last_armed is not None:
                still_urgent = closing_speed > 0.1 and (now - last_armed) >= self._cfg.rearm_cooldown_s
                if not still_urgent:
                    continue
            th = self._active.get(tid)
            if th is None:
                continue
            self._armed_at[tid] = now

            range_m = float(vt["range_m"])
            deadline_s = range_m / max(closing_speed, 0.05)
            kind_min_s, kind_max_s = self._deadline_bounds(th.kind)
            deadline_s = max(kind_min_s, min(kind_max_s, deadline_s))
            th.deadline_s = deadline_s
            th.t_recv = now

            meta = {"id": tid, "kind": th.kind, "speed": round(math.hypot(th.vx, th.vy), 3),
                    "radius": round(th.radius, 3)}
            evt = {"kind": th.kind, "t_emit": now, "t_recv": now, "deadline_s": deadline_s,
                   "id": tid, "range_m": range_m, "meta": meta}
            self._out.push(evt)
            if self._csv_fp:
                try:
                    self._csv.writerow([now, th.kind, deadline_s, json.dumps(meta)])
                except Exception:
                    pass

    # ---------------- Main step (called once per sim tick) ----------------
    def step(self, drone_x: float, drone_y: float, drone_yaw: float) -> None:
        now = self._sim_clock.now()
        dt = 0.0 if self._last_step_t is None else max(0.0, now - self._last_step_t)
        self._last_step_t = now

        if self._prev_drone_xy is not None and dt > 1e-6:
            self._drone_vx = (drone_x - self._prev_drone_xy[0]) / dt
            self._drone_vy = (drone_y - self._prev_drone_xy[1]) / dt
        self._prev_drone_xy = (drone_x, drone_y)

        heading = (math.cos(drone_yaw), math.sin(drone_yaw))
        to_retire = []
        for tid, th in self._active.items():
            th.x += th.vx * dt
            th.y += th.vy * dt
            rel_x, rel_y = th.x - drone_x, th.y - drone_y
            behind = (rel_x * heading[0] + rel_y * heading[1]) < -self._cfg.retire_margin_m
            expired = (now - th.spawn_t) > self._cfg.max_lifetime_s
            if behind or expired:
                to_retire.append(tid)
        for tid in to_retire:
            del self._active[tid]
            self._armed_at.pop(tid, None)

        if self._sim_t_next is None:
            self._sim_t_next = now + self._next_dt
            return
        if now < self._sim_t_next:
            return

        kind = self._choose_kind()
        self._next_dt = self._draw_dt()
        self._sim_t_next = now + self._next_dt

        if len(self._active) >= self._cfg.max_active_threats:
            return  # cap reached -- skip this spawn, cadence continues

        self._spawn_threat(kind, drone_x, drone_y, drone_yaw, now)
