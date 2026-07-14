# analysis/statistics_analyzer.py
from __future__ import annotations
import os, re
import numpy as np
import pandas as pd
from typing import Dict, Any

def run_analysis(zone_metric: str = "mean") -> Dict[str, Any]:
    """
    Summarize experiment CSV per strategy.

    Reads cfg.results_csv_path; writes summary CSV to cfg.analyzer_out_dir.

    Required columns (case-insensitive):
      strategy | elapse_time | zone_violations | propulsion_energy_j | propulsion_mean_power_w

    Optional columns (included in summary if present):
      run | compute_latency_us | events_handled | event_violation_rate
      event_violated_deadline | event_violated_preemptive
    """
    zm = str(zone_metric).strip().lower()
    if zm not in {"median", "mean"}:
        raise ValueError("zone_metric must be 'median' or 'mean'")

    # --- Strict dependency on TeleopConfig ---
    import sys as _sys, os as _os
    _sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    from config import TeleopConfig
    cfg = TeleopConfig()

    if not (isinstance(getattr(cfg, "results_csv_path", None), str) and cfg.results_csv_path):
        raise AttributeError("TeleopConfig must define a non-empty string 'results_csv_path' (input CSV path).")
    if not (isinstance(getattr(cfg, "analyzer_out_dir", None), str) and cfg.analyzer_out_dir):
        raise AttributeError("TeleopConfig must define a non-empty string 'analyzer_out_dir' (output folder).")

    csv_path: str = cfg.results_csv_path
    out_dir: str = cfg.analyzer_out_dir
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSV not found at {csv_path}")
    os.makedirs(out_dir, exist_ok=True)

    # ---------- Read ----------
    df_raw = pd.read_csv(csv_path)

    # Physically-grounded compute/propulsion/total power + per-mission energy
    # + endurance (Glauert forward-flight momentum theory, calibrated against
    # real DJI FlyCart 30 spec-sheet hover data — see power_estimate.py).
    # Optional: only computed if the required source columns are present, so
    # this doesn't break analysis of older/partial CSVs.
    _phys_cols = ("compute_energy_j", "propulsion_mean_power_w")
    _have_phys_cols = all(
        any(c.strip().lower() == want.lower() for c in df_raw.columns)
        for want in _phys_cols
    )
    df_phys: pd.DataFrame | None = None
    if _have_phys_cols:
        from analysis.power_estimate import per_run_physical_metrics
        df_phys = per_run_physical_metrics(df_raw, float(cfg.target_distance))

    # ---------- Column resolution ----------
    def resolve(col_name_expected: str) -> str:
        expected_lower = col_name_expected.strip().lower()
        for c in df_raw.columns:
            if c.strip().lower() == expected_lower:
                return c
        patt = re.compile(rf"^{re.escape(col_name_expected)}$", re.I)
        for c in df_raw.columns:
            if patt.search(c):
                return c
        raise KeyError(f"Missing required column: '{col_name_expected}'. Found columns: {list(df_raw.columns)}")

    def resolve_opt(col_name_expected: str) -> str | None:
        expected_lower = col_name_expected.strip().lower()
        for c in df_raw.columns:
            if c.strip().lower() == expected_lower:
                return c
        patt = re.compile(rf"^{re.escape(col_name_expected)}$", re.I)
        for c in df_raw.columns:
            if patt.search(c):
                return c
        return None  # optional

    # Required columns (case-insensitive)
    COL = {
        "strategy":        resolve("strategy"),
        "elapsed":         resolve("elapse_time"),
        "zone_violations": resolve("zone_violations"),
        "energy_j":        resolve("propulsion_energy_j"),
        "mean_power_w":    resolve("propulsion_mean_power_w"),
    }
    _energy_scale = 1.0 / 1000.0  # J → kJ
    _power_scale  = 1.0 / 1000.0  # W → kW

    # Optional / legacy columns
    col_run = resolve_opt("run")
    col_compute_latency_us = resolve_opt("compute_latency_us")
    col_compute_energy_j = resolve_opt("compute_energy_j")
    col_events_handled = resolve_opt("events_handled")
    col_event_violation_rate = resolve_opt("event_violation_rate")
    col_event_violated_deadline   = resolve_opt("event_violated_deadline")
    col_event_violated_preemptive = resolve_opt("event_violated_preemptive")

    # ---------- Coercions ----------
    def to_num(s: pd.Series) -> pd.Series:
        return pd.to_numeric(s, errors="coerce")

    compute_latency_col = col_compute_latency_us

    # Build normalized dataframe; missing optional columns become NaN
    data = {
        "_strategy":      df_raw[COL["strategy"]].astype(str).str.strip(),
        "_elapsed_s":     to_num(df_raw[COL["elapsed"]]),
        "_zone_viol":     to_num(df_raw[COL["zone_violations"]]),
        "_energy_kj":     to_num(df_raw[COL["energy_j"]]) * _energy_scale,
        "_mean_power_kw": to_num(df_raw[COL["mean_power_w"]]) * _power_scale,
    }

    if compute_latency_col is not None:
        data["_compute_latency"] = to_num(df_raw[compute_latency_col])
    else:
        data["_compute_latency"] = pd.Series(np.nan, index=df_raw.index, dtype=float)

    if col_compute_energy_j is not None:
        data["_compute_energy_j"] = to_num(df_raw[col_compute_energy_j])
    else:
        data["_compute_energy_j"] = pd.Series(np.nan, index=df_raw.index, dtype=float)

    if col_events_handled is not None:
        data["_events_handled"] = to_num(df_raw[col_events_handled])
    else:
        data["_events_handled"] = pd.Series(np.nan, index=df_raw.index, dtype=float)

    if col_event_violation_rate is not None:
        data["_event_violation_rate"] = to_num(df_raw[col_event_violation_rate])
    else:
        data["_event_violation_rate"] = pd.Series(np.nan, index=df_raw.index, dtype=float)

    if col_run is not None:
        data["_run"] = to_num(df_raw[col_run])
    else:
        data["_run"] = pd.Series(np.nan, index=df_raw.index, dtype=float)

    if col_event_violated_deadline is not None:
        data["_evt_viol_deadline"] = to_num(df_raw[col_event_violated_deadline])
    else:
        data["_evt_viol_deadline"] = pd.Series(np.nan, index=df_raw.index, dtype=float)

    if col_event_violated_preemptive is not None:
        data["_evt_viol_preemptive"] = to_num(df_raw[col_event_violated_preemptive])
    else:
        data["_evt_viol_preemptive"] = pd.Series(np.nan, index=df_raw.index, dtype=float)

    if df_phys is not None:
        data["_compute_power_w"] = df_phys["compute_power_w"].reindex(df_raw.index)
        data["_propulsion_power_w_physical"] = df_phys["propulsion_power_w"].reindex(df_raw.index)
        data["_total_power_w"] = df_phys["total_power_w"].reindex(df_raw.index)
        data["_energy_per_mission_kj"] = df_phys["energy_per_mission_kj"].reindex(df_raw.index)
        data["_endurance_min"] = df_phys["endurance_min"].reindex(df_raw.index)
    else:
        for key in ("_compute_power_w", "_propulsion_power_w_physical", "_total_power_w",
                    "_energy_per_mission_kj", "_endurance_min"):
            data[key] = pd.Series(np.nan, index=df_raw.index, dtype=float)

    df_f = pd.DataFrame(data)
    if df_f.empty:
        empty_csv = os.path.join(out_dir, f"strategy_summary_zone-{zm}.csv")
        pd.DataFrame().to_csv(empty_csv, index=False)
        return {
            "zone_metric": zm,
            "summary_csv": empty_csv,
            "summary": {},
        }

    # ---------- Per-strategy summary on all (already-successful) data ----------
    g = df_f.groupby("_strategy", dropna=False)

    def med(s: pd.Series) -> float:
        s = s.dropna()
        return float(s.median()) if len(s) else float("nan")

    summary = pd.DataFrame({
        "runs_considered":               g.size(),
        "reach_time_median_s":           g["_elapsed_s"].apply(med),
        "reach_time_mean_s":             g["_elapsed_s"].mean(),
        "zone_violations_mean":          g["_zone_viol"].mean(),
        "zone_violations_median":        g["_zone_viol"].median(),
        "propulsion_energy_kj_median":   g["_energy_kj"].apply(med),
        "propulsion_energy_kj_mean":     g["_energy_kj"].mean(),
        "propulsion_mean_power_kw_mean": g["_mean_power_kw"].mean(),
    })

    # Compute-latency summary (if present)
    if not df_f["_compute_latency"].isna().all():
        summary["compute_latency_median_us"] = g["_compute_latency"].apply(med)
        summary["compute_latency_mean_us"]   = g["_compute_latency"].mean()

    # Accumulated compute energy per run (raw Joules, not power) — dominated
    # by mission duration at this platform's idle/TDP ratio (see
    # power_estimate.py), so this tracks time-to-goal more than per-event
    # compute intensity; compute_latency_*_us above is the metric that
    # isolates true per-event compute cost. Kept as its own column since the
    # two tell different stories (e.g. CA has 11x APE1's per-event latency
    # but ~42% LESS total compute energy per run, because CA finishes sooner).
    if not df_f["_compute_energy_j"].isna().all():
        summary["compute_energy_j_median"] = g["_compute_energy_j"].apply(med)
        summary["compute_energy_j_mean"]   = g["_compute_energy_j"].mean()
        summary["compute_energy_j_sum"]    = g["_compute_energy_j"].sum()

    # Event-related summaries (if present)
    if not df_f["_event_violation_rate"].isna().all():
        summary["event_violation_rate_median"] = g["_event_violation_rate"].apply(med)
        summary["event_violation_rate_mean"]   = g["_event_violation_rate"].mean()

    if not df_f["_evt_viol_deadline"].isna().all():
        summary["event_viol_deadline_mean"]   = g["_evt_viol_deadline"].mean()
    if not df_f["_evt_viol_preemptive"].isna().all():
        summary["event_viol_preemptive_mean"] = g["_evt_viol_preemptive"].mean()

    # Physically-grounded power/energy summary (if source columns were present).
    # See analysis/power_estimate.py for the model (Glauert
    # forward-flight momentum theory + parasite drag, calibrated against real
    # DJI FlyCart 30 spec-sheet hover data) — distinct from, and more accurate
    # than, propulsion_mean_power_kw_mean above (the repo's EPM distance-only
    # constant, which has no hover term).
    if not df_f["_total_power_w"].isna().all():
        summary["compute_power_w_mean"]        = g["_compute_power_w"].mean()
        summary["propulsion_power_w_physical_mean"] = g["_propulsion_power_w_physical"].mean()
        summary["total_power_w_mean"]          = g["_total_power_w"].mean()
        summary["energy_per_mission_kj_median"] = g["_energy_per_mission_kj"].apply(med)
        summary["energy_per_mission_kj_mean"]   = g["_energy_per_mission_kj"].mean()
        summary["endurance_min_mean"]           = g["_endurance_min"].mean()

    # ---------- Write summary CSV and return ----------
    summary_csv = os.path.join(out_dir, f"strategy_summary_zone-{zm}.csv")
    summary.round(2).to_csv(summary_csv)

    return {
        "zone_metric": zm,
        "summary_csv": summary_csv,
        "summary": summary.round(2).to_dict(orient="index"),
    }
