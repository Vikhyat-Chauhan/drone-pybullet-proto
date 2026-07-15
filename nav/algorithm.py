#!/usr/bin/env python3
# Core navigation algorithm for the Drone simulation
# includes Decision logic (event racing, APE cascade resolution, NFZ repulsion,
# TTC/stopping-distance, breadcrumbs, jerk caps); only
# the I/O boundary -> tick() (one decision step, driven externally) + 
# begin_mission()/end_mission(), so experiment/orchestrator.py
# can interleave p.stepSimulation() at the physics rate independent of the
# nav rate.
#
# Public API:
#   LidarTargetNavigatorCA.begin_mission()
#   LidarTargetNavigatorCA.tick(target_xyz=None) -> str
#       one of: "reached", "no_pose", "no_target", "hard_stale", "running"
#   LidarTargetNavigatorCA.end_mission() -> (reached, elapsed_s, total_latency_us,
#       compute_energy_j, events_handled, events_violated,
#       events_violated_deadline, events_violated_preemptive)
#   LidarTargetNavigatorCA.go_to(target_xyz=None, timeout_s=None) -> same 8-tuple
#       (standalone convenience wrapper; advances its own SimClock ticks,
#       does NOT step PyBullet -- only useful without a real physics body)

import math, threading, time, ctypes
from dataclasses import dataclass
from typing import Optional, Tuple, Set, List, Deque, Dict
from collections import deque
import logging

from sim.teleop import PyTeleop
from config import TeleopConfig
from nav.mcu_cycle_model import (
    McuCycleMeter, latency_to_energy_j, APE_LATENCY_US, DEADLINE_SCALE,
)
import nav.ape_native as ape_native
from sim.sim_adapters import (
    ScanMsg, sector_min, window_vals, SimClock,
    PoseProvider, StaticPoseProvider, ScanProvider, CloudProvider, EventQueue,
)


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ===== Navigation configs (defaults) =====
@dataclass
class GoToConfig:
    goal_radius_m: float = 4.0
    kp_lin: float = 1.2
    kp_z: float = 1.0
    kp_yaw: float = 2.0
    max_v: float = 15.0
    max_vz: float = 3.5
    max_wz: float = 1.4
    slow_yaw_threshold: float = 1.0
    rate_hz: float = 30.0
    edge_guard_m: float = 4.5
    edge_guard_scale: float = 0.6

@dataclass
class AvoidCfg:
    safe_m: float = 5.0
    hysteresis_m: float = 1.0
    front_deg: float = 5.0
    side_deg: float = 30.0
    side_center_deg: float = 30.0
    turn_rate: float = 0.9
    watchdog_sec: float = 0.6
    hard_stale_sec: float = 1.2
    min_turn_sec: float = 0.7

@dataclass
class BreadcrumbCfg:
    cell_xy_m: float = 2.0
    cell_z_m: float = 2.0
    capacity: int = 3000

@dataclass
class SafetyCfg:
    ambiguity_eps_m: float = 0.5
    ttc_soft_s: float = 2.2
    ttc_hard_s: float = 1.4
    v_min_frac: float = 0.20
    near_obs_m: float = 3.0
    cap_wz_near_obs: float = 1.2
    corner_deg: float = 30.0
    corner_inflate_m: float = 2.0
    progress_window_s: float = 3.0
    min_progress_m: float = 1.0
    escape_yaw_rad: float = 0.8
    escape_time_s: float = 0.8
    crumb_oscillations_to_flip: int = 12
    dv_max_mps_per_s: float = 6.0
    jw_max_radps2: float = 3.0
    clear_ahead_thresh_m: float = 16.0
    dv_clear_scale: float = 0.35
    yaw_align_rad: float = 0.25

@dataclass
class RiskCfg:
    vehicle_radius_m: float = 0.7
    max_decel_mps2: float = 4.5
    stop_margin_m: float = 2.0
    gate_half_deg: float = 12.0
    center_weight: float = 0.8
    align_weight: float = 0.8
    sweep_max_deg: float = 60.0
    sweep_step_deg: float = 2.5
    arc_check_m: float = 4.0
    nofly_min_dist_m: float = 3.0
    nofly_soft_w: float = 9.0
    curvature_k: float = 0.9

# APE planning budgets (ms), computed live from mcu_cycle_model.py's
# gem5-measured APE_LATENCY_US (the single modeled Cortex-M7-approximation
# MCU) so they can never drift from the model they're derived from.
_APE1_BUDGET_MS = APE_LATENCY_US["APE1"] * DEADLINE_SCALE / 1000.0
_APE2_BUDGET_MS = APE_LATENCY_US["APE2"] * DEADLINE_SCALE / 1000.0
_APE3_BUDGET_MS = APE_LATENCY_US["APE3"] * DEADLINE_SCALE / 1000.0


@dataclass
class EventDecisionCfg:
    # Each APE thread sleeps for budget_ms at startup to emulate this latency.
    ape1_budget_ms: float = _APE1_BUDGET_MS
    ape2_budget_ms: float = _APE2_BUDGET_MS
    ape3_budget_ms: float = _APE3_BUDGET_MS

    v_cap_frac: float = 0.75
    selector_mode: str = "CA"
    commit_hold_s: float = 0.3

    sudden_obj_radius_m: float = 1.2
    sudden_obj_clearance_m: float = 0.3
    sidestep_deg: float = 110.0
    sidestep_speed_frac: float = 0.35


@dataclass
class ApeAlgoCfg:
    """Tuning for the native DWA (APE2) and VFH (APE3) planners, plus
    multi-layer LiDAR geometry (mirrors the original model.sdf's <vertical>
    block, as plain numbers since there's no SDF here)."""
    n_layers: int = 5
    vertical_angle_min: float = -0.0872665   # -5 deg
    vertical_angle_increment: float = 0.0436332  # (2*5deg)/(n_layers-1)

    dwa_n_v: int = 3
    dwa_n_w: int = 3
    dwa_dt: float = 0.3
    dwa_horizon_s: float = 0.6
    dwa_w_clear: float = 0.4
    dwa_w_heading: float = 0.4
    dwa_w_speed: float = 0.2

    vfh_n_sectors: int = 36
    vfh_threshold: float = 0.3
    vfh_smax_sectors: float = 6.0


class LidarTargetNavigatorCA:
    """
    Default navigator. When an event arrives, APE1/APE2/APE3 workers run in
    parallel; the selector reads out the highest-quality plan that has
    actually finished by the deadline (opportunistic, not predicted — see
    _evt_cascade_order()).
    """

    def __init__(self,
                 teleop: PyTeleop,
                 cfg: TeleopConfig,
                 selector_mode: str,
                 drone_pose: PoseProvider,
                 target_pose: StaticPoseProvider,
                 scan: ScanProvider,
                 cloud: CloudProvider,
                 events: EventQueue,
                 sim_clock: SimClock,
                 goto_cfg: Optional[GoToConfig] = None,
                 avoid_cfg: Optional[AvoidCfg] = None,
                 crumb_cfg: Optional[BreadcrumbCfg] = None,
                 safety_cfg: Optional[SafetyCfg] = None,
                 risk_cfg: Optional[RiskCfg] = None,
                 algo_cfg: Optional[ApeAlgoCfg] = None):
        self._teleop = teleop
        self._cfg = cfg
        self._gc = goto_cfg or GoToConfig()
        self._ac = avoid_cfg or AvoidCfg()
        self._bc = crumb_cfg or BreadcrumbCfg()
        self._sc = safety_cfg or SafetyCfg()
        self._rc = risk_cfg or RiskCfg()
        self._algo = algo_cfg or ApeAlgoCfg()
        self._logger = logging.getLogger(__name__)
        self._logger.propagate = True
        self._cycle_meter = McuCycleMeter()

        self._events_handled: int = 0
        self._events_violated: int = 0
        self._events_violated_deadline: int = 0
        self._events_violated_preemptive: int = 0

        self._tick_count: int = 0
        self._avoiding_ticks: int = 0
        self._commit_hold_ticks: int = 0
        self._path_length: float = 0.0
        self._last_pos: Optional[Tuple[float, float, float]] = None
        self._evt_v_cmd_pre: float = 0.0

        self._drone_pose = drone_pose
        self._target_pose = target_pose
        self._scan_provider = scan
        self._cloud_provider = cloud
        self._evt_queue = events
        self._sim_clock = sim_clock

        self._edc = EventDecisionCfg()
        try:
            sm = selector_mode
            if sm:
                self._edc.selector_mode = str(sm).upper().strip()
        except Exception:
            pass

        self._log("CFG",
                  type="CFG",
                  ape_budgets_ms=[self._edc.ape1_budget_ms, self._edc.ape2_budget_ms, self._edc.ape3_budget_ms],
                  v_cap_frac=self._edc.v_cap_frac,
                  selector_mode=self._edc.selector_mode,
                  commit_hold_s=self._edc.commit_hold_s,
                  sudden_obj_radius_m=self._edc.sudden_obj_radius_m,
                  sudden_obj_clearance_m=self._edc.sudden_obj_clearance_m,
                  sidestep_deg=self._edc.sidestep_deg,
                  sidestep_speed_frac=self._edc.sidestep_speed_frac)

        self._avoiding = False
        self._avoid_sign = 0
        self._avoid_until = 0.0

        self._crumb_set: Set[Tuple[int,int,int]] = set()
        self._crumb_fifo: Deque[Tuple[int,int,int]] = deque()
        self._side_bias: int = +1
        self._crumb_hits_recent: int = 0

        self._progress_t0: Optional[float] = None
        self._progress_d0: Optional[float] = None
        self._escape_until: float = 0.0

        self._v_cmd_prev = 0.0
        self._wz_cmd_prev = 0.0

        self._pending_evt: Optional[Dict] = None
        self._evt_deadline_at: float = 0.0
        self._evt_lock = threading.Lock()
        self._evt_proposals: Dict[str, Dict] = {}

        self._evt_active: bool = False
        self._evt_resolved: bool = False

        self._resolved_cmd: tuple = (0.0, 0.0, 0.0)
        self._evt_resolved_at: float = 0.0
        self._commit_hold_active: bool = False
        self._nav_start_logged: bool = False

        self._t_start: float = 0.0

    # ---------- logging ----------
    def _log(self, msg: str, **fields):
        log_type = fields.pop("type", "GEN")
        try:
            self._logger.info(msg, extra={"type": log_type, "payload": fields})
        except Exception as e:
            print("LOGGING_ERROR:", e)

    # ---------- sim-time ----------
    def _sim_time(self) -> float:
        return self._sim_clock.now()

    def shutdown(self):
        """No-op: nothing to tear down (no ROS nodes/executor in this port)."""
        pass

    # ---------- accessors ----------
    def _latest_drone(self) -> Optional[Tuple[float, float, float, float]]:
        return self._drone_pose.latest()

    def _latest_target(self) -> Optional[Tuple[float, float, float]]:
        return self._target_pose.latest()

    def _scan_metrics(self) -> Tuple[float, float, float, bool, Optional[ScanMsg], float]:
        scan, t_last = self._scan_provider.latest()
        now = self._sim_time()
        stale = (now - t_last) > self._ac.watchdog_sec
        if scan is None:
            return float('inf'), float('inf'), float('inf'), True, None, now
        front = sector_min(scan, 0.0, self._ac.front_deg)
        left  = sector_min(scan, +self._ac.side_center_deg, self._ac.side_deg)
        right = sector_min(scan, -self._ac.side_center_deg, self._ac.side_deg)
        return front, left, right, stale, scan, now

    def _cell(self, x: float, y: float, z: float) -> Tuple[int,int,int]:
        return (int(round(x/self._bc.cell_xy_m)),
                int(round(y/self._bc.cell_xy_m)),
                int(round(z/self._bc.cell_z_m)))

    def _crumb_add(self, cell: Tuple[int,int,int]) -> None:
        if cell in self._crumb_set:
            return
        self._crumb_set.add(cell)
        self._crumb_fifo.append(cell)
        if len(self._crumb_fifo) > max(1, self._bc.capacity):
            old = self._crumb_fifo.popleft()
            self._crumb_set.discard(old)

    # ---------- LiDAR helpers ----------
    def _frange(self, a: float, b: float, step: float):
        x = a
        while x <= b + 1e-9:
            yield x
            x += step

    def _sweep_candidates(self, scan: ScanMsg) -> List[float]:
        step = max(0.5, float(self._rc.sweep_step_deg))
        M = max(5.0, float(self._rc.sweep_max_deg))
        return [d for d in self._frange(-M, +M, step)]

    def _range_at(self, scan: ScanMsg, center_deg: float, half_w_deg: float = 2.0) -> float:
        return sector_min(scan, center_deg, half_w_deg)

    def _gap_metrics(self, scan: ScanMsg) -> Tuple[float, float]:
        L = self._range_at(scan, +self._rc.gate_half_deg, half_w_deg=2.0)
        R = self._range_at(scan, -self._rc.gate_half_deg, half_w_deg=2.0)
        width = (L + R)
        skew  = (L - R)
        if not math.isfinite(width): width = float('inf')
        if not math.isfinite(skew):  skew = 0.0
        return width, skew

    # ---------- No-fly helpers ----------
    def _nofly_rects(self):
        return getattr(self._cfg, "nofly_rects_xywh", None) or []

    def _min_dist_nofly(self, x: float, y: float) -> float:
        rects = self._nofly_rects()
        if not rects: return float('inf')
        best = float('inf')
        for (cx, cy, w, h) in rects:
            dx = max(0.0, abs(x - cx) - 0.5*w)
            dy = max(0.0, abs(y - cy) - 0.5*h)
            best = min(best, math.hypot(dx, dy))
        return best

    def _nfz_repulsion_vec(self, x: float, y: float) -> Tuple[float, float, float]:
        rects = self._nofly_rects()
        if not rects: return 0.0, 0.0, 0.0
        eps = 0.5
        fx = fy = 0.0
        cost = 0.0
        for (cx, cy, w, h) in rects:
            dx = x - cx
            dy = y - cy
            dx_out = max(0.0, abs(dx) - 0.5*w)
            dy_out = max(0.0, abs(dy) - 0.5*h)
            d = math.hypot(dx_out, dy_out)
            cost += 1.0 / (d + eps)
            if d > 1e-3:
                fx += (dx_out / d) / (d + eps)
                fy += (dy_out / d) / (d + eps)
        return fx, fy, cost

    # ---------- arc safety check ----------
    def _arc_is_clear(self, scan: ScanMsg, deg: float, arc_m: float) -> bool:
        r = self._range_at(scan, deg, half_w_deg=2.0)
        return (not math.isfinite(r)) or (r >= arc_m)

    def _choose_heading(self, scan: ScanMsg, yaw_err: float, x: float, y: float) -> float:
        yaw_err_deg = math.degrees(yaw_err)
        best = 0.0
        best_score = -1e18
        _, _, nfz_soft0 = self._nfz_repulsion_vec(x, y)
        for deg in self._sweep_candidates(scan):
            if not self._arc_is_clear(scan, deg, self._rc.arc_check_m):
                continue
            r = self._range_at(scan, deg, half_w_deg=2.0)
            if not math.isfinite(r): r = 0.0
            l = self._range_at(scan, deg + self._rc.gate_half_deg, 2.0)
            rr = self._range_at(scan, deg - self._rc.gate_half_deg, 2.0)
            skew = abs(l - rr) if (math.isfinite(l) and math.isfinite(rr)) else 0.0
            score = r - self._rc.center_weight*skew - self._rc.align_weight*abs(deg - yaw_err_deg) - self._rc.nofly_soft_w*nfz_soft0
            if score > best_score:
                best_score, best = score, deg
        return math.radians(best)

    def _stopping_limited_speed(self, v_des: float, dmin: float) -> float:
        if not math.isfinite(dmin) or dmin <= self._rc.stop_margin_m:
            return 0.0
        vmax = math.sqrt(max(0.0, 2.0*self._rc.max_decel_mps2*(dmin - self._rc.stop_margin_m)))
        return min(v_des, vmax)

    # ---------- event planners ----------
    def _evt_put(self, name, v, wz, vz, score, ready_t):
        with self._evt_lock:
            self._evt_proposals[name] = {"v": v, "wz": wz, "vz": vz, "score": score, "ready_t": ready_t}

    def _build_ape_params(self, snap, multilayer: bool) -> ape_native.ApeParams:
        """Marshals scan + nav state + config into the native planner's
        param struct."""
        scan = snap["scan"]
        p = ape_native.ApeParams()

        if multilayer:
            cloud_ranges, cloud_n_ranges, cloud_n_layers, _ = snap["cloud"]
            if cloud_ranges and cloud_n_ranges > 0 and cloud_n_layers > 0:
                flat, n_ranges, n_layers = cloud_ranges, cloud_n_ranges, cloud_n_layers
            elif scan is not None and scan.ranges:
                flat, n_ranges, n_layers = list(scan.ranges), len(scan.ranges), 1
            else:
                flat, n_ranges, n_layers = [0.0], 1, 1
        elif scan is not None and scan.ranges:
            flat, n_ranges, n_layers = list(scan.ranges), len(scan.ranges), 1
        else:
            flat, n_ranges, n_layers = [0.0], 1, 1

        # ranges must outlive p (POINTER doesn't keep the buffer alive) —
        # stash it as a plain attribute so it isn't garbage-collected
        # while the native call still holds the pointer.
        arr = (ctypes.c_float * len(flat))(*flat)
        p._ranges_keepalive = arr
        p.ranges = ctypes.cast(arr, ctypes.POINTER(ctypes.c_float))
        p.n_ranges = n_ranges
        p.n_layers = n_layers

        if scan is not None:
            p.angle_min = float(scan.angle_min)
            p.angle_increment = float(scan.angle_increment)
            p.range_min = float(scan.range_min)
            p.range_max = float(scan.range_max)
        else:
            p.angle_min = -math.pi
            p.angle_increment = 2.0 * math.pi / max(1, n_ranges)
            p.range_min = 0.05
            p.range_max = 60.0

        p.vertical_angle_min = self._algo.vertical_angle_min
        p.vertical_angle_increment = self._algo.vertical_angle_increment

        p.v_cmd = float(snap["v_cmd"])
        p.yaw_err = float(snap["yaw_err"])

        p.max_v = self._gc.max_v
        p.max_wz = self._gc.max_wz
        p.max_vz = self._gc.max_vz
        p.kp_yaw = self._gc.kp_yaw
        p.vehicle_radius_m = self._rc.vehicle_radius_m
        p.max_decel_mps2 = self._rc.max_decel_mps2
        p.stop_margin_m = self._rc.stop_margin_m
        p.safe_m = self._ac.safe_m
        p.front_deg = self._ac.front_deg
        p.side_deg = self._ac.side_deg
        p.v_cap_frac = self._edc.v_cap_frac
        p.sidestep_deg = self._edc.sidestep_deg
        p.sidestep_speed_frac = self._edc.sidestep_speed_frac
        p.sudden_obj_radius_m = self._edc.sudden_obj_radius_m
        p.sudden_obj_clearance_m = self._edc.sudden_obj_clearance_m
        p.curvature_k = self._rc.curvature_k

        p.dwa_n_v = self._algo.dwa_n_v
        p.dwa_n_w = self._algo.dwa_n_w
        p.dwa_dt = self._algo.dwa_dt
        p.dwa_horizon_s = self._algo.dwa_horizon_s
        p.dwa_w_clear = self._algo.dwa_w_clear
        p.dwa_w_heading = self._algo.dwa_w_heading
        p.dwa_w_speed = self._algo.dwa_w_speed

        p.vfh_n_sectors = self._algo.vfh_n_sectors
        p.vfh_threshold = self._algo.vfh_threshold
        p.vfh_smax_sectors = self._algo.vfh_smax_sectors

        return p

    def _evt_plan_ape1(self, snap, budget_ms, arrival_sim_t):
        """Runs the native planner synchronously (real compute costs
        microseconds), then schedules the proposal ready at
        arrival_sim_t + budget_ms/1000 in SIM time use since sim-time
        scheduling is exact regardless of wall-clock speed."""
        params = self._build_ape_params(snap, multilayer=False)
        r = ape_native.plan_ape1(params)
        ready_t = arrival_sim_t + max(0.0, budget_ms) / 1000.0
        return self._evt_put("APE1", r.v, r.wz, r.vz, r.score, ready_t)

    def _evt_plan_ape2(self, snap, budget_ms, arrival_sim_t):
        # APE2 = DWA (native_api.c's ape_native_plan_ape2); called here with
        # multilayer=False, which degenerates its multi-layer min-range
        # consensus lookup to single-layer (see ape2_dwa.c).
        params = self._build_ape_params(snap, multilayer=False)
        r = ape_native.plan_ape2(params)
        ready_t = arrival_sim_t + max(0.0, budget_ms) / 1000.0
        return self._evt_put("APE2", r.v, r.wz, r.vz, r.score, ready_t)

    def _evt_plan_ape3(self, snap, budget_ms, arrival_sim_t):
        # APE3 = VFH (native_api.c's ape_native_plan_ape3); its polar-
        # histogram valley search only reads layer 0's rays, but the
        # multilayer scan here also feeds its final per-layer stopping
        # check (see ape3_vfh.c).
        params = self._build_ape_params(snap, multilayer=True)
        r = ape_native.plan_ape3(params)
        ready_t = arrival_sim_t + max(0.0, budget_ms) / 1000.0
        return self._evt_put("APE3", r.v, r.wz, r.vz, r.score, ready_t)

    # ---------- event helpers ----------
    def _evt_deadline_feasible(self, deadline_s: float) -> bool:
        return deadline_s >= (self._edc.ape1_budget_ms / 1000.0)

    def _evt_cascade_order(self) -> List[str]:
        if self._edc.selector_mode == "CA":
            return ["APE3", "APE2", "APE1"]
        return [self._edc.selector_mode]

    def _evt_violate(self, reason: str = "miss"):
        if self._pending_evt is None:
            return
        self._events_violated += 1
        if reason == "DEADLINE":
            self._events_violated_deadline += 1
        elif reason == "PREEMPTIVE":
            self._events_violated_preemptive += 1

    def _evt_clear(self):
        with self._evt_lock:
            self._evt_proposals.clear()
        self._pending_evt = None
        self._evt_deadline_at = 0.0
        self._evt_active = False
        self._evt_resolved = False

    # ---------- APE calibration ----------
    def _calibrate_budgets(self, n_reps: int = 30) -> None:
        import statistics
        _scan = ScanMsg(
            ranges=[10.0] * 61,
            angle_min=-math.radians(30.0),
            angle_increment=math.radians(1.0),
            range_min=0.1,
            range_max=30.0,
        )
        _snap = {
            "v_cmd": 10.0,
            "scan": _scan,
            "yaw_err": 0.1,
            "cloud": ([], 0, 0, 0.0),
        }
        results = {}
        for name, fn in [("APE1", self._evt_plan_ape1),
                          ("APE2", self._evt_plan_ape2),
                          ("APE3", self._evt_plan_ape3)]:
            times_ms = []
            for _ in range(n_reps):
                with self._evt_lock:
                    self._evt_proposals.pop(name, None)
                t0 = time.perf_counter()
                fn(_snap, 0, 0.0)
                times_ms.append((time.perf_counter() - t0) * 1000.0)
            results[name] = {
                "mean": statistics.mean(times_ms),
                "p95":  sorted(times_ms)[int(0.95 * len(times_ms))],
                "wcet": max(times_ms),
            }
        with self._evt_lock:
            self._evt_proposals.clear()
        self._log("CALIBRATION", type="CALIBRATION",
                  ape1_mean_ms=round(results["APE1"]["mean"], 3),
                  ape1_p95_ms =round(results["APE1"]["p95"],  3),
                  ape1_wcet_ms=round(results["APE1"]["wcet"], 3),
                  ape2_mean_ms=round(results["APE2"]["mean"], 3),
                  ape2_p95_ms =round(results["APE2"]["p95"],  3),
                  ape2_wcet_ms=round(results["APE2"]["wcet"], 3),
                  ape3_mean_ms=round(results["APE3"]["mean"], 3),
                  ape3_p95_ms =round(results["APE3"]["p95"],  3),
                  ape3_wcet_ms=round(results["APE3"]["wcet"], 3),
                  configured_budget_ms=[self._edc.ape1_budget_ms,
                                        self._edc.ape2_budget_ms,
                                        self._edc.ape3_budget_ms])

    # ---------- mission bracket ----------
    def begin_mission(self) -> None:
        self._t_start = self._sim_time()
        self._cycle_meter.begin()
        self._events_handled = 0
        self._events_violated = 0
        self._events_violated_deadline = 0
        self._events_violated_preemptive = 0
        self._nav_start_logged = False
        self._avoiding = False
        self._v_cmd_prev = 0.0
        self._wz_cmd_prev = 0.0
        self._progress_t0 = None
        self._progress_d0 = None
        self._escape_until = 0.0
        self._evt_clear()

        # -- diagnostic-only counters (not part of the public return tuple;
        # pulled from run_logs.json's MISSION_STATS record for analysis) --
        self._tick_count = 0
        self._avoiding_ticks = 0
        self._commit_hold_ticks = 0
        self._path_length = 0.0
        self._last_pos: Optional[Tuple[float, float, float]] = None

    def end_mission(self, reached: bool) -> Tuple[bool, float, float, float, int, int, int, int]:
        self._teleop.stop()
        elapsed = self._sim_time() - self._t_start
        total_latency_us, _ = self._cycle_meter.end()
        compute_energy_j = latency_to_energy_j(total_latency_us, elapsed)
        self._log("MISSION_STATS", type="MISSION_STATS",
                  tick_count=self._tick_count,
                  avoiding_ticks=self._avoiding_ticks,
                  commit_hold_ticks=self._commit_hold_ticks,
                  path_length_m=round(self._path_length, 3))
        return (reached, elapsed, total_latency_us, compute_energy_j,
                self._events_handled, self._events_violated,
                self._events_violated_deadline, self._events_violated_preemptive)

    # ---------- core: one decision-loop iteration ----------
    def tick(self, target_xyz: Optional[Tuple[float, float, float]] = None) -> str:
        """One iteration of the original go_to() while-loop body. Caller
        owns time advancement; this issues one velocity command via
        self._teleop.set_cmd(). Returns "reached" | "no_pose" | "no_target"
        | "hard_stale" | "running"."""
        rate = max(1.0, float(self._gc.rate_hz))
        dt = 1.0 / rate

        dpose = self._latest_drone()
        if dpose is None:
            return "no_pose"

        if target_xyz is None:
            tpose = self._latest_target()
            if tpose is None:
                self._teleop.set_cmd(0.0, 0.0, 0.0, 0.0)
                return "no_target"
            tx, ty, tz = tpose
        else:
            tx, ty, tz = target_xyz

        x, y, z, yaw = dpose
        ex, ey, ez = (tx - x), (ty - y), (tz - z)
        dist_xy = math.hypot(ex, ey)
        dist = math.sqrt(ex*ex + ey*ey + ez*ez)

        self._tick_count += 1
        if self._last_pos is not None:
            lx, ly, lz = self._last_pos
            self._path_length += math.sqrt((x-lx)**2 + (y-ly)**2 + (z-lz)**2)
        self._last_pos = (x, y, z)

        if not self._nav_start_logged:
            self._log("POSES", type="POSES",
                      nav_start_drone_pose=(x, y, z, yaw),
                      nav_start_target=(tx, ty, tz),
                      nav_start_dist_m=round(dist, 3),
                      nav_start_dist_xy_m=round(dist_xy, 3))
            self._nav_start_logged = True

        if dist <= self._gc.goal_radius_m:
            return "reached"

        # ---------- Base go-to ----------
        hdg_des = math.atan2(ey, ex)
        yaw_err = _wrap_pi(hdg_des - yaw)

        v_cmd  = min(self._gc.max_v, self._gc.kp_lin * dist_xy)
        if abs(yaw_err) > self._gc.slow_yaw_threshold:
            v_cmd = min(v_cmd, self._gc.edge_guard_scale * self._gc.max_v)

        vz_cmd = max(-self._gc.max_vz, min(self._gc.max_vz, self._gc.kp_z * ez))
        wz_cmd = max(-self._gc.max_wz, min(self._gc.max_wz, self._gc.kp_yaw * yaw_err))

        front, left, right, stale, scan, now = self._scan_metrics()

        # ---- Event intake ----
        # A tick may see multiple new arrivals; each is processed through
        # the single-active-race logic in order (an arrival mid-race
        # salvages/preempts it, matching the original single-slot queue).
        for evt in self._evt_queue.pop_new():
            deadline_s = max(0.0, float(evt.get("deadline_s", 0.0)))
            self._log("EVENT", type="ARRIVAL",
                      t_rec=evt["t_recv"],
                      deadline_s=deadline_s,
                      deadline_computed=evt["t_recv"] + deadline_s)
            self._events_handled += 1

            if self._evt_active:
                now_t = self._sim_time()
                with self._evt_lock:
                    ready_curr = {n: p for n, p in self._evt_proposals.items() if now_t >= p["ready_t"]}
                salvage_order = self._evt_cascade_order()
                chosen_curr = next(
                    ((n, ready_curr[n]) for n in salvage_order if n in ready_curr),
                    None
                )
                if chosen_curr is not None:
                    _, prop = chosen_curr
                    v_cmd, wz_cmd, vz_cmd = prop["v"], prop["wz"], prop["vz"]
                    if not self._evt_resolved:
                        self._evt_resolved = True
                    self._evt_clear()
                else:
                    self._evt_violate("PREEMPTIVE")
                    self._log("EVENT", type="PREEMPTIVE", c_time=self._sim_time())
                    self._evt_clear()

            if self._edc.selector_mode == "CA":
                feasible = self._evt_deadline_feasible(deadline_s)
            else:
                feasible = True

            if not feasible:
                self._events_violated += 1
                self._events_violated_deadline += 1
                self._log("EVENT", type="DEADLINE_PREEMPT",
                          c_time=self._sim_time(), deadline_s=deadline_s,
                          reason="deadline shorter than fastest APE budget")
            else:
                self._commit_hold_active = False
                self._pending_evt = evt
                self._evt_deadline_at = evt["t_recv"] + deadline_s
                self._evt_active = True
                self._evt_resolved = False

                with self._evt_lock:
                    self._evt_proposals = {}

                snap = {
                    "v_cmd": v_cmd,
                    "scan": scan,
                    "yaw_err": _wrap_pi(math.atan2(ey, ex) - yaw),
                    "cloud": self._cloud_provider.latest(),
                }
                self._evt_v_cmd_pre = v_cmd

                # Each native call runs synchronously; availability is
                # scheduled in sim-time (ready_t) -- see _evt_plan_ape1.
                mode = self._edc.selector_mode
                arrival_sim_t = self._sim_time()
                if mode in ("CA", "APE1"):
                    self._evt_plan_ape1(snap, self._edc.ape1_budget_ms, arrival_sim_t)
                if mode in ("CA", "APE2"):
                    self._evt_plan_ape2(snap, self._edc.ape2_budget_ms, arrival_sim_t)
                if mode in ("CA", "APE3"):
                    self._evt_plan_ape3(snap, self._edc.ape3_budget_ms, arrival_sim_t)

        event_active = self._evt_active and (self._pending_evt is not None)

        # Hard-stale: brake until scans recover
        _, t_last = self._scan_provider.latest()
        if (self._sim_time() - t_last) > self._ac.hard_stale_sec:
            self._teleop.set_cmd(0.0, 0.0, 0.0, 0.0)
            return "hard_stale"

        # ---------- Breadcrumb bookkeeping ----------
        cell = self._cell(x, y, z)
        if cell in self._crumb_set:
            self._crumb_hits_recent += 1
        else:
            self._crumb_hits_recent = max(0, self._crumb_hits_recent - 1)
        self._crumb_add(cell)

        # ---------- Doorway/corridor metrics ----------
        corr_width, corr_skew = (float('inf'), 0.0)
        if scan is not None:
            corr_width, corr_skew = self._gap_metrics(scan)

        # ---------- No-fly proximity & repulsion ----------
        nf_dist = self._min_dist_nofly(x, y)
        fx, fy, nfz_soft = self._nfz_repulsion_vec(x, y)
        rep_angle = math.atan2(fy, fx) if (fx*fx + fy*fy) > 1e-6 else None
        if rep_angle is not None and math.isfinite(rep_angle):
            yaw_rep_err = _wrap_pi(rep_angle - yaw)
            wz_cmd += 0.4 * max(-self._gc.max_wz, min(self._gc.max_wz, yaw_rep_err))

        # ---------- Corner/edge guard ----------
        effective_safe_m = self._ac.safe_m
        if abs(math.degrees(yaw_err)) > self._sc.corner_deg:
            effective_safe_m += self._sc.corner_inflate_m
        if min(left, right) < self._gc.edge_guard_m:
            v_cmd = min(v_cmd, self._gc.edge_guard_scale * self._gc.max_v)
        if math.isfinite(nf_dist) and nf_dist < self._rc.nofly_min_dist_m:
            effective_safe_m = max(effective_safe_m, self._rc.nofly_min_dist_m)
            v_cmd = min(v_cmd, 0.4 * self._gc.max_v)

        # ---------- Event window — opportunistic best-available selector ----------
        if event_active:
            now_t = self._sim_time()
            tl = max(0.0, self._evt_deadline_at - now_t)
            with self._evt_lock:
                ready = {n: p for n, p in self._evt_proposals.items() if now_t >= p["ready_t"]}

            cascade = self._evt_cascade_order()
            best_ready = next(((n, ready[n]) for n in cascade if n in ready), None)
            best_possible_ready = (best_ready is not None and best_ready[0] == cascade[0])

            if best_possible_ready or tl <= 0.0:
                if best_ready is None:
                    self._evt_violate("DEADLINE")
                    self._log("EVENT", type="DEADLINE", c_time=self._sim_time())
                    self._evt_clear()
                else:
                    winner_name, prop = best_ready
                    v_cmd, wz_cmd, vz_cmd = prop["v"], prop["wz"], prop["vz"]
                    if not self._evt_resolved:
                        self._log("EVENT", type="RESOLVED",
                                  planner=winner_name,
                                  ready_t=prop["ready_t"],
                                  v_pre=round(getattr(self, "_evt_v_cmd_pre", float("nan")), 3),
                                  v_out=round(prop["v"], 3),
                                  max_v=self._gc.max_v)
                        _running = (["APE1", "APE2", "APE3"]
                                    if self._edc.selector_mode == "CA"
                                    else [winner_name])
                        self._cycle_meter.record_event(winner_name, _running)
                        self._evt_resolved = True
                        self._resolved_cmd = (prop["v"], prop["wz"], prop["vz"])
                        self._evt_resolved_at = self._sim_time()
                        self._commit_hold_active = True
                        self._evt_clear()

        # ---------- Commitment hold ----------
        if self._commit_hold_active:
            hold_elapsed = self._sim_time() - self._evt_resolved_at
            if hold_elapsed < self._edc.commit_hold_s:
                v_cmd, wz_cmd, vz_cmd = self._resolved_cmd
                self._commit_hold_ticks += 1
            else:
                self._commit_hold_active = False

        # ---------- Avoidance / heading select ----------
        if stale:
            v_cmd = 0.0
            wz_cmd = 0.0
            self._avoiding = False
        else:
            if self._avoiding:
                self._avoiding_ticks += 1
                if now < self._avoid_until or front < (effective_safe_m + self._ac.hysteresis_m):
                    v_cmd = 0.0
                    wz_cmd = self._avoid_sign * min(self._gc.max_wz, self._ac.turn_rate)
                else:
                    self._avoiding = False
            else:
                if front < effective_safe_m:
                    self._avoiding = True
                    diff = abs(left - right)
                    if diff < self._sc.ambiguity_eps_m:
                        self._avoid_sign = (+1 if self._side_bias > 0 else -1)
                    else:
                        self._avoid_sign = (+1 if left > right else -1)
                    self._avoid_until = now + self._ac.min_turn_sec
                    v_cmd = 0.0
                    wz_cmd = self._avoid_sign * min(self._gc.max_wz, self._ac.turn_rate)
                else:
                    if scan is not None:
                        hdg_off = self._choose_heading(scan, yaw_err, x, y)
                        yaw_goal = _wrap_pi(hdg_off)
                        wz_cmd = max(-self._gc.max_wz,
                                     min(self._gc.max_wz, self._gc.kp_yaw * yaw_goal))
                    side_min = min(left, right)
                    if side_min < 1.2 * self._rc.vehicle_radius_m + 0.6:
                        if left < right:
                            wz_cmd -= 0.25
                        else:
                            wz_cmd += 0.25
                    if self._crumb_hits_recent >= self._sc.crumb_oscillations_to_flip:
                        self._side_bias *= -1
                        self._crumb_hits_recent = 0

        # ---------- Doorway + TTC + stopping distance ----------
        dmin = float('inf')
        if scan is not None:
            window = window_vals(scan, 0.0, max(5.0, self._ac.front_deg))
            dmin = min(window) if window else float('inf')
            min_clear = 2.0*self._rc.vehicle_radius_m + 0.6
            if math.isfinite(corr_width) and corr_width < (min_clear + 1.0):
                v_cmd = min(v_cmd, 0.25 * self._gc.max_v)
                sgn = 1.0 if corr_skew > 0.0 else -1.0
                wz_cmd = max(-self._gc.max_wz,
                             min(self._gc.max_wz, wz_cmd + 0.5*sgn))
            if v_cmd > 0.05 and math.isfinite(dmin) and dmin > 0.0:
                ttc = dmin / max(v_cmd, 1e-3)
                if ttc < self._sc.ttc_soft_s:
                    num = (ttc - self._sc.ttc_hard_s)
                    den = max(self._sc.ttc_soft_s - self._sc.ttc_hard_s, 1e-3)
                    frac = max(self._sc.v_min_frac, min(1.0, num / den))
                    v_cmd = self._gc.max_v * frac
                v_cmd = self._stopping_limited_speed(v_cmd, dmin)

        v_cmd = min(v_cmd, self._gc.max_v / (1.0 + self._rc.curvature_k * abs(wz_cmd)))

        nearest = min(front, left, right)
        if nearest < self._sc.near_obs_m:
            wz_cmd = max(self._sc.cap_wz_near_obs * -1.0,
                         min(self._sc.cap_wz_near_obs, wz_cmd))

        # ---------- Progress watchdog ----------
        if self._progress_t0 is None:
            self._progress_t0 = now
            self._progress_d0 = dist

        if now < self._escape_until:
            v_cmd = 0.0
            wz_cmd = (self._sc.escape_yaw_rad / self._sc.escape_time_s) * (
                1 if self._avoid_sign >= 0 else -1)
        elif (now - self._progress_t0) > self._sc.progress_window_s:
            gained = (self._progress_d0 - dist)
            if gained < self._sc.min_progress_m:
                self._escape_until = now + self._sc.escape_time_s
                v_cmd = 0.0
                wz_cmd = (self._sc.escape_yaw_rad / self._sc.escape_time_s) * (
                    1 if self._avoid_sign >= 0 else -1)
            self._progress_t0 = now
            self._progress_d0 = dist

        # ---------- Command ramp/jerk caps ----------
        base_dv_max = self._sc.dv_max_mps_per_s * dt
        dwz_max = self._sc.jw_max_radps2 * dt

        empty_heading = (
            (not self._avoiding)
            and math.isfinite(dmin)
            and dmin >= self._sc.clear_ahead_thresh_m
            and abs(wz_cmd) <= self._sc.yaw_align_rad
        )

        dv_up_max = base_dv_max * (self._sc.dv_clear_scale if empty_heading else 1.0)
        dv_down_max = base_dv_max * 1.5

        if v_cmd > self._v_cmd_prev:
            v_cmd = min(self._v_cmd_prev + dv_up_max, v_cmd)
        else:
            v_cmd = max(self._v_cmd_prev - dv_down_max, v_cmd)

        wz_cmd = max(self._wz_cmd_prev - dwz_max,
                     min(self._wz_cmd_prev + dwz_max, wz_cmd))

        self._v_cmd_prev = v_cmd
        self._wz_cmd_prev = wz_cmd

        self._teleop.set_cmd(v_cmd, 0.0, vz_cmd, wz_cmd)
        return "running"

    # ---------- standalone convenience wrapper ----------
    def go_to(self,
              target_xyz: Optional[Tuple[float, float, float]] = None,
              timeout_s: Optional[float] = None) -> Tuple[bool, float, float, float, int, int, int, int]:
        """Standalone use without a real PyBullet drive loop: advances its
        own SimClock and sleeps dt in real time, but never calls
        p.stepSimulation(). experiment/orchestrator.py calls tick()
        directly instead, to interleave physics stepping."""
        rate = max(1.0, float(self._gc.rate_hz))
        dt = 1.0 / rate
        self.begin_mission()
        reached = False
        while True:
            status = self.tick(target_xyz)
            self._sim_clock.tick()
            if status == "reached":
                reached = True
                break
            if timeout_s is not None and (self._sim_time() - self._t_start) > timeout_s:
                break
            time.sleep(dt)
        return self.end_mission(reached)
