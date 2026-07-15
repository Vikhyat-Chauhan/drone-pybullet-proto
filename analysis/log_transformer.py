#!/usr/bin/env python3
"""
Log transformer for logs/run_logs.json (STRICT TeleopConfig version).

Offline reconstruction of experiment_summary.csv from the raw JSON event
log, independent of (and cross-checkable against) experiment/orchestrator.py's own
direct CSV write via results_io.flush_csv -- the runtime's primary path.

- Detects run boundaries as: collect EVENTs up to teleop's STOP record,
  then emits a row on the immediately following "main"-logger terminator
  {reached, elapsed, violations, energy_j, mean_power_w, compute_latency_us}
  (see experiment/orchestrator.py's per-strategy loop in run_attempt()).
- Requires at least one terminal EVENT (nav/algorithm.py logs type=
  RESOLVED on success, or DEADLINE/PREEMPTIVE/DEADLINE_PREEMPT on a
  violation; the non-terminal ARRIVAL record is not counted).
- CSV columns (order preserved):
    timestamp, strategy, run, reached, elapse_time, energy_kj, mean_power_kw,
    compute_latency_us, nav_start_dist_xy_m, zone_violations,
    event_violated, events_handled, event_violations
"""

from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional
import json
import csv

# ---------- Shape helpers ----------
# nav/algorithm.py's LidarTargetNavigatorCA._log("EVENT", type=<TYPE>, ...)
# carries the outcome in the top-level "type" field, not payload. ARRIVAL
# fires once per event on intake (= orchestrator's events_handled counter).
# DEADLINE/PREEMPTIVE/DEADLINE_PREEMPT are terminal violation outcomes for a
# subset of those events; RESOLVED isn't counted separately -- "handled
# minus violated" already implies it.
_EVENT_ARRIVAL_TYPE = "ARRIVAL"
_EVENT_VIOLATION_TYPES = {"DEADLINE", "PREEMPTIVE", "DEADLINE_PREEMPT"}

def _is_stop(rec: Dict[str, Any]) -> bool:
    # sim/teleop.py: self._logger.info({"event": "STOP"}); logger name is "sim.teleop".
    return rec.get("name") == "sim.teleop" and isinstance(rec.get("msg"), dict) and rec["msg"].get("event") == "STOP"

def _is_main_terminator(rec: Dict[str, Any]) -> bool:
    # experiment/orchestrator.py: _main_logger = logging.getLogger("main"); see run_attempt()
    if rec.get("name") != "main":
        return False
    msg = rec.get("msg")
    # Terminator keyed by reached & elapsed; other fields validated at emit-time.
    return isinstance(msg, dict) and ("reached" in msg) and ("elapsed" in msg)

def _is_event_arrival(rec: Dict[str, Any]) -> bool:
    return rec.get("msg") == "EVENT" and rec.get("type") == _EVENT_ARRIVAL_TYPE and isinstance(rec.get("payload"), dict)

def _is_event_violation(rec: Dict[str, Any]) -> bool:
    return rec.get("msg") == "EVENT" and rec.get("type") in _EVENT_VIOLATION_TYPES

def _is_nav_start(rec: Dict[str, Any]) -> bool:
    # POSES line that carries nav-start distances
    return rec.get("type") == "POSES" and rec.get("msg") == "POSES" and isinstance(rec.get("payload"), dict)

# ---------- Core transformer ----------
@dataclass
class TransformCfg:
    input_log_path: str
    output_csv_path: str
    # Optional distance filter: only emit rows when nav_start_dist_xy_m >= this threshold
    min_nav_start_dist_xy_m: Optional[float] = None

def transform(cfg: TransformCfg) -> List[Dict[str, Any]]:
    in_path = Path(cfg.input_log_path)
    out_path = Path(cfg.output_csv_path)

    if not in_path.exists():
        raise FileNotFoundError(f"[log_transformer] Input log not found: {in_path}")

    out_path.parent.mkdir(parents=True, exist_ok=True)

    current: Dict[str, Any] = {
        "events_handled": 0,
        "event_violations": 0,
        "saw_any_event": False,
        "run_closed": False,            # becomes True at STOP; emit on next MAIN reached
        "nav_seen": False,              # must see POSES before emitting a row
        "nav_start_dist_m": None,       # from POSES
        "nav_start_dist_xy_m": None,    # from POSES
    }
    rows: List[Dict[str, Any]] = []

    def reset_counters():
        current["events_handled"] = 0
        current["event_violations"] = 0
        current["saw_any_event"] = False
        current["run_closed"] = False
        current["nav_seen"] = False
        current["nav_start_dist_m"] = None
        current["nav_start_dist_xy_m"] = None

    with in_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except Exception:
                continue

            # Required: one POSES record before nav per run.
            if _is_nav_start(rec):
                payload = rec["payload"]
                try:
                    current["nav_start_dist_xy_m"] = float(payload["nav_start_dist_xy_m"])
                except Exception:
                    raise ValueError(f"[log_transformer] POSES missing 'nav_start_dist_xy_m' at ts={rec.get('ts')}")
                try:
                    current["nav_start_dist_m"] = float(payload["nav_start_dist_m"])
                except Exception:
                    raise ValueError(f"[log_transformer] POSES missing 'nav_start_dist_m' at ts={rec.get('ts')}")
                current["nav_seen"] = True
                continue

            # Legacy per-event ZONEVIOLATION records are intentionally ignored.

            if _is_event_arrival(rec):
                current["saw_any_event"] = True
                current["events_handled"] += 1
                continue

            if _is_event_violation(rec):
                current["event_violations"] += 1
                continue

            if _is_stop(rec):
                current["run_closed"] = True
                continue

            if _is_main_terminator(rec) and current["run_closed"]:
                # We require: at least one EVENT and the POSES record
                if current["saw_any_event"]:
                    if not current["nav_seen"]:
                        raise RuntimeError("[log_transformer] Run reached terminator without prior POSES line (nav_start).")

                    # --- Distance filter applied here ---
                    if cfg.min_nav_start_dist_xy_m is not None:
                        nav_dist = current["nav_start_dist_xy_m"]
                        if nav_dist is None:
                            raise RuntimeError("[log_transformer] Missing nav_start_dist_xy_m when applying distance filter.")
                        if nav_dist < float(cfg.min_nav_start_dist_xy_m):
                            reset_counters()
                            continue
                    # -----------------------------------

                    ts = rec.get("ts")
                    msg = rec.get("msg", {})
                    strategy = rec.get("strategy")

                    # STRICT: pull final metrics from the "main" terminator record
                    try:
                        zone_violations = int(msg["violations"])
                    except Exception:
                        raise ValueError(f"[log_transformer] main terminator missing integer 'violations' at ts={ts}")

                    # Energy & power (convert to kJ / kW)
                    try:
                        energy_kj = float(msg["energy_j"]) / 1000.0
                    except Exception:
                        raise ValueError(f"[log_transformer] main terminator missing numeric 'energy_j' at ts={ts}")
                    try:
                        mean_power_kw = float(msg["mean_power_w"]) / 1000.0
                    except Exception:
                        raise ValueError(f"[log_transformer] main terminator missing numeric 'mean_power_w' at ts={ts}")

                    # compute_latency_us: mcu_cycle_model latency (µs) for this run's APE invocations
                    compute_latency_us = 0.0
                    try:
                        if "compute_latency_us" in msg and msg["compute_latency_us"] is not None:
                            compute_latency_us = float(msg["compute_latency_us"])
                    except Exception:
                        raise ValueError(f"[log_transformer] terminator 'compute_latency_us' not numeric at ts={ts}")

                    row = {
                        "timestamp": ts,
                        "strategy": strategy,
                        # 'run' is assigned in a post-pass once we know grouping across strategies
                        "reached": bool(msg.get("reached")),
                        "elapse_time": float(msg.get("elapsed", 0.0)),
                        "energy_kj": energy_kj,
                        "mean_power_kw": mean_power_kw,
                        "compute_latency_us": compute_latency_us,
                        "nav_start_dist_xy_m": current["nav_start_dist_xy_m"],
                        "zone_violations": zone_violations,
                        "event_violated": current["event_violations"] > 0,
                        "events_handled": int(current["events_handled"]),
                        "event_violations": int(current["event_violations"]),
                    }
                    rows.append(row)
                reset_counters()
                continue

    # ---------- Assign per-cycle run tags ----------
    def assign_run_tags(rows: List[Dict[str, Any]]) -> None:
        CYCLE_STRATS = {"APE1", "APE2", "APE3", "CA"}
        seen: set = set()
        run_idx = 1
        for r in rows:
            # Default to current run index
            r["run"] = run_idx
            s = str(r.get("strategy")) if r.get("strategy") is not None else ""
            if s in CYCLE_STRATS:
                seen.add(s)
                if seen == CYCLE_STRATS:
                    # Completed one full cycle; next row starts a new run
                    seen.clear()
                    run_idx += 1

    assign_run_tags(rows)

    # Write CSV (order preserved)
    cols = [
        "timestamp",
        "strategy",
        "run",  # grouping across strategies
        "reached",
        "elapse_time",
        "energy_kj",
        "mean_power_kw",
        "compute_latency_us",
        "nav_start_dist_xy_m",
        "zone_violations",
        "event_violated",
        "events_handled",
        "event_violations",
    ]
    with out_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)

    return rows

# ---------- Public entrypoint: STRICT TeleopConfig ----------
def run_from_cfg() -> List[Dict[str, Any]]:
    # Deliberately NOT cfg.results_csv_path: that's orchestrator.py's own
    # canonical CSV output (results_io.flush_csv) with a different schema
    # (see module docstring) -- writing here would clobber it. This output
    # is only for cross-checking the raw log against that canonical CSV.
    return transform(TransformCfg(
        input_log_path="logs/run_logs.json",
        output_csv_path="logs/results/experiment_summary_reconstructed.csv",
        min_nav_start_dist_xy_m=0,
    ))

if __name__ == "__main__":
    run_from_cfg()