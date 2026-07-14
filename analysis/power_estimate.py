# analysis/power_estimate.py
"""
Real-world power/energy estimate for the modelled DJI FlyCart-30-class drone,
combining:

  1. Compute power  — already physically grounded in the repo. Reuses the Fan,
     Weber & Barroso (ISCA 2007) utilization power model and the single
     low-power Cortex-M7-class MCU's active/idle power constants from
     mcu_cycle_model.py, aggregated across all runs in experiment_summary.csv.

  2. Propulsion power — the repo's existing propulsion_energy_j/mean_power_w
     columns are a single literature constant (EPM = 208.9 J/m, Kirschstein et
     al.) with no hover/drag/battery term (see energy_monitor.py).
     This module instead derives propulsion power from momentum (actuator-disk)
     theory + parasite drag, calibrated with an empirical efficiency factor
     derived from the real DJI FlyCart 30 spec sheet (dji.com/flycart-30/specs),
     and reports both a point estimate and a sensitivity range.

     Induced power is modelled as speed-dependent (Glauert's forward-flight
     extension of momentum theory, 1926 — standard treatment in Leishman,
     "Principles of Helicopter Aerodynamics", already cited above for the
     hover case), not the constant hover-power floor used in the first pass
     of this model: translational lift means a rotor disk needs LESS induced
     velocity (and therefore less induced power) in forward flight than in
     hover, for the same thrust. This matters here because the simulated
     cruise speeds (~2-5 m/s) are the same order of magnitude as this
     airframe's hover induced velocity (~4.7 m/s @ 65kg/11.82m^2), so the
     effect is not negligible.

  3. Combined real-world total system power and estimated flight endurance on
     a real 2x DB2000 battery pack (3968.8 Wh).

All formulas/constants are cited inline. See docs/POWER_MODEL.md for the
provenance of the compute-power side of this model (gem5 measurement
methodology, MCU datasheet constants) and the propulsion model.
"""

from __future__ import annotations

import math
import os
import re
from itertools import product
from typing import Any, Dict

import numpy as np
import pandas as pd

import sys as _sys, os as _os
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
from mcu_cycle_model import (
    _ACTIVE_POWER_W as MCU_ACTIVE_POWER_W,
    _IDLE_FRAC as MCU_IDLE_FRAC,
    _N_CORES as MCU_N_CORES,
)

# ---------------------------------------------------------------------------
# DJI FlyCart 30 real-world reference specs (dji.com/flycart-30/specs).
# Used as (a) the mass/rotor geometry for the momentum-theory model, and
# (b) the empirical calibration anchor for ideal-aero -> real-electrical
# efficiency, and (c) the battery capacity for endurance estimates.
# ---------------------------------------------------------------------------
FC30_MASS_EMPTY_KG: float = 65.0          # matches physics.py DronePhysics default mass_kg
FC30_ROTOR_COUNT: int = 8
FC30_PROP_DIAMETER_M: float = 1.3716      # 54 in
FC30_HOVER_TIME_EMPTY_DUAL_MIN: float = 29.0   # empty (65kg), 2x DB2000
FC30_BATTERY_WH_EACH: float = 1984.4      # 52.22 V x 38000 mAh
FC30_BATTERY_COUNT: int = 2
FC30_BATTERY_WH_TOTAL: float = FC30_BATTERY_WH_EACH * FC30_BATTERY_COUNT  # 3968.8 Wh

G_MPS2: float = 9.80665
RHO_SEA_LEVEL: float = 1.225  # kg/m^3, ISA sea level

# Effective flat-plate frontal drag area (Cd*A), literature-typical for a
# large octocopter frame. Not measured for FlyCart-30-class airframes in this
# repo; cross-checked in magnitude against the quadratic per-axis drag
# coefficient k2=0.04 already used in physics.py
# (cites Hattenberger et al. 2023 for the |v|v drag form). Swept as a
# sensitivity parameter below rather than treated as a fixed value.
CDA_FRONTAL_GRID_M2 = (0.3, 0.4, 0.5)

# Figure-of-merit / total hover efficiency grid. Multirotor FM is typically
# 0.6-0.8 (rotor aerodynamic efficiency only); motor+ESC+propeller electrical
# efficiency for small/mid multirotors is commonly cited around ~40% overall
# (see hover-power validation literature, e.g. experimentally measured ~40%
# efficiency quadrotor studies). We instead derive eta empirically below from
# the real DJI spec sheet and sweep a band around it.
ETA_GRID = (0.35, 0.45, 0.55)


def _rotor_disk_area_total_m2(diameter_m: float, rotor_count: int) -> float:
    r = diameter_m / 2.0
    return rotor_count * math.pi * r * r


FC30_DISK_AREA_TOTAL_M2 = _rotor_disk_area_total_m2(FC30_PROP_DIAMETER_M, FC30_ROTOR_COUNT)


def induced_hover_power_w(mass_kg: float, disk_area_m2: float, rho: float = RHO_SEA_LEVEL) -> float:
    """
    Rankine-Froude actuator-disk (momentum) theory ideal induced hover power.

        P_induced = T^1.5 / sqrt(2 * rho * A)

    T = weight (N). Standard treatment: Leishman, "Principles of Helicopter
    Aerodynamics"; applied to multirotor UAV energy estimation e.g. MDPI
    Energies 15(19):7136 (2022).
    """
    thrust_n = mass_kg * G_MPS2
    return (thrust_n ** 1.5) / math.sqrt(2.0 * rho * disk_area_m2)


def induced_velocity_forward_flight_mps(mass_kg: float, disk_area_m2: float,
                                         speed_mps: float, rho: float = RHO_SEA_LEVEL,
                                         iters: int = 60) -> float:
    """
    Glauert's momentum theory for forward flight (1926) — the standard
    extension of actuator-disk theory used to model translational lift
    (Leishman, "Principles of Helicopter Aerodynamics", ch. 2). Solves the
    implicit relation

        v_i * sqrt(V^2 + v_i^2) = v_i_hover^2,   v_i_hover^2 = T / (2*rho*A)

    for the induced velocity v_i at forward speed V, via fixed-point
    iteration (converges quickly and monotonically for this well-behaved
    functional form). At V=0 this reduces exactly to v_i_hover, matching
    induced_hover_power_w's implied induced velocity.
    """
    thrust_n = mass_kg * G_MPS2
    vi_hover_sq = thrust_n / (2.0 * rho * disk_area_m2)
    vi_hover = math.sqrt(vi_hover_sq)
    v = max(0.0, speed_mps)
    if v == 0.0:
        return vi_hover
    vi = vi_hover
    for _ in range(iters):
        vi = vi_hover_sq / math.sqrt(v * v + vi * vi)
    return vi


def induced_power_forward_flight_w(mass_kg: float, disk_area_m2: float,
                                    speed_mps: float, rho: float = RHO_SEA_LEVEL) -> float:
    """
    Speed-dependent ideal induced power: P_induced(V) = T * v_i(V).
    Reduces to induced_hover_power_w(...) at V=0.
    """
    thrust_n = mass_kg * G_MPS2
    vi = induced_velocity_forward_flight_mps(mass_kg, disk_area_m2, speed_mps, rho)
    return thrust_n * vi


def parasite_power_w(cda_m2: float, speed_mps: float, rho: float = RHO_SEA_LEVEL) -> float:
    """Flat-plate parasite/profile drag power: P = 0.5 * rho * (Cd*A) * v^3."""
    return 0.5 * rho * cda_m2 * (max(0.0, speed_mps) ** 3)


def empirical_hover_efficiency() -> float:
    """
    eta = P_ideal(hover) / P_real(hover), derived from the real DJI FlyCart 30
    spec sheet: 65 kg empty+battery mass, dual DB2000 (3968.8 Wh), 29 min
    hover endurance at that empty weight.
    """
    p_ideal = induced_hover_power_w(FC30_MASS_EMPTY_KG, FC30_DISK_AREA_TOTAL_M2)
    p_real = FC30_BATTERY_WH_TOTAL * 3600.0 / (FC30_HOVER_TIME_EMPTY_DUAL_MIN * 60.0)
    return p_ideal / p_real, p_ideal, p_real


def real_propulsion_power_w(speed_mps: float, cda_m2: float, eta: float,
                             mass_kg: float = FC30_MASS_EMPTY_KG,
                             disk_area_m2: float = FC30_DISK_AREA_TOTAL_M2) -> float:
    p_induced = induced_power_forward_flight_w(mass_kg, disk_area_m2, speed_mps)
    p_parasite = parasite_power_w(cda_m2, speed_mps)
    return (p_induced + p_parasite) / eta


# ---------------------------------------------------------------------------
# CSV loading (mirrors analysis/statistics_analyzer.py's case-insensitive
# column resolution so this composes with the same config).
# ---------------------------------------------------------------------------

def _resolve(df: pd.DataFrame, col_name_expected: str) -> str:
    expected_lower = col_name_expected.strip().lower()
    for c in df.columns:
        if c.strip().lower() == expected_lower:
            return c
    patt = re.compile(rf"^{re.escape(col_name_expected)}$", re.I)
    for c in df.columns:
        if patt.search(c):
            return c
    raise KeyError(f"Missing required column: '{col_name_expected}'. Found columns: {list(df.columns)}")


def per_run_physical_metrics(df_raw: pd.DataFrame, target_distance_m: float) -> pd.DataFrame:
    """
    Compute physically-grounded compute/propulsion/total power, per-mission
    energy, and endurance for each row (run) of an experiment_summary-shaped
    dataframe. Index-aligned to df_raw (NaN rows dropped, so callers should
    join back by index, not assume positional alignment).

    Shared by run_power_estimate() (standalone deep-dive CLI) and
    analysis.statistics_analyzer.run_analysis() (the pipeline's canonical
    per-strategy summary), so the physics model has one definition.
    """
    col_strategy = _resolve(df_raw, "strategy")
    col_elapsed = _resolve(df_raw, "elapse_time")
    col_compute_energy_j = _resolve(df_raw, "compute_energy_j")
    col_epm_power_w = _resolve(df_raw, "propulsion_mean_power_w")

    df = pd.DataFrame({
        "strategy": df_raw[col_strategy].astype(str).str.strip(),
        "elapsed_s": pd.to_numeric(df_raw[col_elapsed], errors="coerce"),
        "compute_energy_j": pd.to_numeric(df_raw[col_compute_energy_j], errors="coerce"),
        "epm_power_w": pd.to_numeric(df_raw[col_epm_power_w], errors="coerce"),
    }, index=df_raw.index)
    df = df.dropna(subset=["strategy", "elapsed_s"])
    df = df[df["elapsed_s"] > 0]

    # --- (a) compute power per run ---
    df["compute_power_w"] = df["compute_energy_j"] / df["elapsed_s"]
    df["u_eff"] = (df["compute_power_w"] - MCU_ACTIVE_POWER_W * MCU_IDLE_FRAC) / (
        (MCU_ACTIVE_POWER_W - MCU_ACTIVE_POWER_W * MCU_IDLE_FRAC)
    )
    df["u_eff"] = df["u_eff"].clip(lower=0.0, upper=1.0)

    # --- (b) physically-grounded propulsion power per run (Glauert forward-
    # flight induced power + parasite drag, calibrated against real DJI
    # FlyCart 30 hover-power data — see module docstring / real_propulsion_power_w) ---
    df["cruise_speed_mps"] = target_distance_m / df["elapsed_s"]

    eta_mid, _p_ideal_hover, _p_real_hover_fc30 = empirical_hover_efficiency()
    cda_mid = float(np.median(CDA_FRONTAL_GRID_M2))

    df["propulsion_power_w"] = df["cruise_speed_mps"].apply(
        lambda v: real_propulsion_power_w(v, cda_mid, eta_mid)
    )
    df["epm_vs_physics_ratio"] = df["epm_power_w"] / df["propulsion_power_w"]

    # --- (c) combined total system power + per-mission energy + endurance ---
    df["total_power_w"] = df["compute_power_w"] + df["propulsion_power_w"]
    df["energy_per_mission_kj"] = df["total_power_w"] * df["elapsed_s"] / 1000.0
    df["endurance_min"] = FC30_BATTERY_WH_TOTAL * 3600.0 / df["total_power_w"] / 60.0

    return df


def run_power_estimate() -> Dict[str, Any]:
    import sys as _sys, os as _os
    _sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
    from config import TeleopConfig
    cfg = TeleopConfig()

    csv_path: str = cfg.results_csv_path
    out_dir: str = cfg.analyzer_out_dir
    target_distance_m: float = float(cfg.target_distance)
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSV not found at {csv_path}")
    os.makedirs(out_dir, exist_ok=True)

    df_raw = pd.read_csv(csv_path)
    df = per_run_physical_metrics(df_raw, target_distance_m)
    eta_mid, p_ideal_hover, p_real_hover_fc30 = empirical_hover_efficiency()

    # --- per-strategy aggregation ---
    g = df.groupby("strategy", dropna=False)

    def med(s: pd.Series) -> float:
        s = s.dropna()
        return float(s.median()) if len(s) else float("nan")

    summary = pd.DataFrame({
        "runs_considered": g.size(),
        "compute_power_w_mean": g["compute_power_w"].mean(),
        "compute_power_w_median": g["compute_power_w"].apply(med),
        "compute_power_w_std": g["compute_power_w"].std(),
        "u_eff_mean": g["u_eff"].mean(),
        "cruise_speed_mps_mean": g["cruise_speed_mps"].mean(),
        "propulsion_power_w_mean": g["propulsion_power_w"].mean(),
        "propulsion_power_w_median": g["propulsion_power_w"].apply(med),
        "propulsion_power_w_std": g["propulsion_power_w"].std(),
        "epm_power_w_mean": g["epm_power_w"].mean(),
        "epm_vs_physics_ratio_mean": g["epm_vs_physics_ratio"].mean(),
        "total_power_w_mean": g["total_power_w"].mean(),
        "total_power_w_median": g["total_power_w"].apply(med),
        "energy_per_mission_kj_mean": g["energy_per_mission_kj"].mean(),
        "energy_per_mission_kj_median": g["energy_per_mission_kj"].apply(med),
        "endurance_min_mean": g["endurance_min"].mean(),
        "endurance_min_median": g["endurance_min"].apply(med),
    })

    # --- (d) sensitivity grid: eta x CdA -> total_power_w range per strategy ---
    sens_rows = []
    strat_speed_mean = g["cruise_speed_mps"].mean()
    strat_compute_mean = g["compute_power_w"].mean()
    for strategy in summary.index:
        v = strat_speed_mean[strategy]
        p_compute = strat_compute_mean[strategy]
        totals = []
        for eta, cda in product(ETA_GRID, CDA_FRONTAL_GRID_M2):
            p_prop = real_propulsion_power_w(v, cda, eta)
            totals.append(p_compute + p_prop)
        sens_rows.append({
            "strategy": strategy,
            "total_power_w_min": min(totals),
            "total_power_w_max": max(totals),
        })
    sens_df = pd.DataFrame(sens_rows).set_index("strategy")
    summary = summary.join(sens_df)
    summary["endurance_min_at_max_power"] = (
        FC30_BATTERY_WH_TOTAL * 3600.0 / summary["total_power_w_max"] / 60.0
    )
    summary["endurance_min_at_min_power"] = (
        FC30_BATTERY_WH_TOTAL * 3600.0 / summary["total_power_w_min"] / 60.0
    )

    out_csv = os.path.join(out_dir, "power_estimate_summary.csv")
    summary.round(3).to_csv(out_csv)

    print(f"Empirical hover efficiency eta = {eta_mid:.3f} "
          f"(P_ideal={p_ideal_hover:.0f} W, P_real(DJI spec)={p_real_hover_fc30:.0f} W)")
    print(f"Rotor disk area total = {FC30_DISK_AREA_TOTAL_M2:.2f} m^2 "
          f"({FC30_ROTOR_COUNT} x {FC30_PROP_DIAMETER_M:.4f} m props)")
    print()
    with pd.option_context("display.width", 200, "display.max_columns", None):
        print(summary.round(2))
    print(f"\nWrote {out_csv}")

    return {
        "summary_csv": out_csv,
        "summary": summary.round(3).to_dict(orient="index"),
        "eta_mid": eta_mid,
        "p_ideal_hover_w": p_ideal_hover,
        "p_real_hover_w_fc30_spec": p_real_hover_fc30,
        "disk_area_total_m2": FC30_DISK_AREA_TOTAL_M2,
    }


if __name__ == "__main__":
    run_power_estimate()
