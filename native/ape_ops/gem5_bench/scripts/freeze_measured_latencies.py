#!/usr/bin/env python3
"""
freeze_measured_latencies.py — regenerates gem5_measured_latencies.py
(repo root) from a fresh gem5 cycle-accurate run of the real native APE
planners (ape1_bug_plan/ape2_dwa_plan/ape3_vfh_plan — see
native/ape_ops/src/) against a named CPU profile from ../configs/
(default: cortex_m4_168mhz.py, an STM32F405/Cortex-M4 @ 168MHz — the
actual low-compute flight-controller MCU this repo targets; see
docs/POWER_MODEL.md). This one measurement feeds BOTH live
deadline-feasibility timing (budget_ms) and compute-energy accounting.

Usage:
    GEM5_ROOT=/path/to/gem5 python3 freeze_measured_latencies.py [--profile NAME]

Requires:
    - native/ape_ops/gem5_bench/bench/build/arm/bench_ape{1,2,3}
      (build with: GEM5_ROOT=... make -C ../bench arm)
    - a gem5 checkout with build/ARM/gem5.opt built
      (cd $GEM5_ROOT && scons build/ARM/gem5.opt)

Switching CPU/algorithm again in the future: add a new profile module to
../configs/ (CPU_PRESET/CPU_FREQ/MEM_TYPE/CHIP/VDD/ACTIVE_MA/SLEEP_MA),
then rerun this script with --profile <name>. No other code changes
needed — run_gem5_study.py discovers profiles dynamically, and
mcu_cycle_model.py reads CHIP/CPU_CLOCK_HZ/ACTIVE_MA/SLEEP_MA/VDD
straight out of the regenerated gem5_measured_latencies.py.

See docs/POWER_MODEL.md for the full methodology.
"""
from __future__ import annotations

import argparse
import importlib
import sys
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_gem5_study import run_study, parse_num_cycles, available_profiles

_HERE = Path(__file__).resolve().parent
_GEM5_BENCH_DIR = _HERE.parent
_REPO_ROOT = _GEM5_BENCH_DIR.parent.parent.parent
_CONFIGS_DIR = _GEM5_BENCH_DIR / "configs"

ITERATIONS = 200
PLANNERS = ("ape1", "ape2", "ape3")

# Search-and-rescue exploration workload (target undetected -- see
# bench/src/main.c's APE_BENCH_MODE) measured from the search_apeN
# binaries (bench/Makefile), a genuinely different workload per planner
# from the avoidance one above, not just a different input.
SEARCH_PLANNERS = ("ape1", "ape2", "ape3")


def _clock_hz_from_freq_str(freq: str) -> float:
    """Parses a gem5 --cpu-freq string like '168MHz' into Hz."""
    freq = freq.strip()
    if freq.upper().endswith("MHZ"):
        return float(freq[:-3]) * 1_000_000.0
    if freq.upper().endswith("GHZ"):
        return float(freq[:-3]) * 1_000_000_000.0
    raise ValueError(f"unrecognized CPU_FREQ format: {freq!r}")


def _render(profile_name: str, study, total_cycles: dict[str, int],
            total_cycles_search: dict[str, int]) -> str:
    cpu_clock_hz = _clock_hz_from_freq_str(study.CPU_FREQ)
    return f'''"""
gem5_measured_latencies.py — GENERATED FILE, do not edit by hand.

Regenerate with:
    cd native/ape_ops/gem5_bench
    GEM5_ROOT=/path/to/gem5 python3 scripts/freeze_measured_latencies.py --profile {profile_name}

Raw per-invocation cycle counts from gem5 cycle-accurate simulation
using an in-order MinorCPU @ {study.CPU_FREQ} (see
native/ape_ops/gem5_bench/configs/{profile_name}.py — an in-order
MinorCPU APPROXIMATION of {study.CHIP}, NOT a true M-profile simulation;
gem5 has no M-profile CPU model) of the REAL native APE planners
(ape1_bug_plan/ape2_dwa_plan/ape3_vfh_plan — see native/ape_ops/src/),
not a synthetic op-count proxy. Read directly from each profile's
stats.txt ROI block (see native/ape_ops/gem5_bench/bench/src/main.c for
the m5_reset_stats/m5_dump_stats bracketing, against one fixed
open-corridor scan fixture).

TOTAL_CYCLES_SEARCH is the same measurement for each planner's
search-and-rescue exploration code path (target undetected, from the
search_apeN binaries -- see bench/src/main.c's APE_BENCH_MODE and
nav_algorithm.py's _search_native_plan) -- a genuinely different
workload per planner (APE2/APE3 also touch their persistent search-state
grid), not a different input to the same code.

This is the ACTIVE simulated CPU for this repo's power model: it feeds
both live deadline-feasibility timing (budget_ms, via
mcu_cycle_model.APE_LATENCY_US) and compute-energy accounting (via
mcu_cycle_model.McuCycleMeter, using ACTIVE_MA/SLEEP_MA/VDD below) — one
processor, one measurement, no separate higher-power companion-computer
model. Other CPU profiles live in native/ape_ops/gem5_bench/configs/ and
past frozen results in native/ape_ops/gem5_bench/out/frozen/ — see
docs/POWER_MODEL.md for the full methodology and multi-profile workflow.
"""

PROFILE_NAME = "{profile_name}"
CHIP = "{study.CHIP}"

ITERATIONS = {ITERATIONS}
CPU_CLOCK_HZ = {cpu_clock_hz}  # {profile_name}.py's CPU_FREQ ({study.CPU_FREQ})

# Power-model parameters for CHIP, at CPU_CLOCK_HZ (see the profile
# module above for the datasheet source).
VDD = {study.VDD}
ACTIVE_MA = {study.ACTIVE_MA}
SLEEP_MA = {study.SLEEP_MA}

# Total simulated cycles across ITERATIONS invocations, ROI-only
# (process/loader startup excluded).
TOTAL_CYCLES: dict[str, int] = {{
    "ape1": {total_cycles["ape1"]},
    "ape2": {total_cycles["ape2"]},
    "ape3": {total_cycles["ape3"]},
}}

# Same measurement, search-and-rescue exploration workload (see the
# module docstring above).
TOTAL_CYCLES_SEARCH: dict[str, int] = {{
    "ape1": {total_cycles_search["ape1"]},
    "ape2": {total_cycles_search["ape2"]},
    "ape3": {total_cycles_search["ape3"]},
}}
'''


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default="cortex_m4_168mhz", choices=available_profiles())
    args = ap.parse_args()

    sys.path.insert(0, str(_CONFIGS_DIR))
    study = importlib.import_module(args.profile)

    total_cycles: dict[str, int] = {}
    for planner in PLANNERS:
        binary = _GEM5_BENCH_DIR / "bench" / "build" / "arm" / f"bench_{planner}"
        if not binary.exists():
            raise FileNotFoundError(
                f"{binary} not found — build it first with:\n"
                f"    GEM5_ROOT=... make -C {_GEM5_BENCH_DIR / 'bench'} arm"
            )
        outdir = _GEM5_BENCH_DIR / "out" / "mcu" / args.profile / planner
        stats_path = run_study(args.profile, binary, outdir)
        total_cycles[planner] = parse_num_cycles(stats_path)
        print(f"{planner}: TOTAL_CYCLES = {total_cycles[planner]}")

    total_cycles_search: dict[str, int] = {}
    for planner in SEARCH_PLANNERS:
        binary = _GEM5_BENCH_DIR / "bench" / "build" / "arm" / f"search_{planner}"
        if not binary.exists():
            raise FileNotFoundError(
                f"{binary} not found — build it first with:\n"
                f"    GEM5_ROOT=... make -C {_GEM5_BENCH_DIR / 'bench'} arm"
            )
        outdir = _GEM5_BENCH_DIR / "out" / "mcu" / args.profile / f"search_{planner}"
        stats_path = run_study(args.profile, binary, outdir)
        total_cycles_search[planner] = parse_num_cycles(stats_path)
        print(f"search_{planner}: TOTAL_CYCLES_SEARCH = {total_cycles_search[planner]}")

    rendered = _render(args.profile, study, total_cycles, total_cycles_search)

    out_path = _REPO_ROOT / "gem5_measured_latencies.py"
    out_path.write_text(rendered)
    print(f"Wrote {out_path}")

    frozen_dir = _GEM5_BENCH_DIR / "out" / "frozen"
    frozen_dir.mkdir(parents=True, exist_ok=True)
    frozen_path = frozen_dir / f"{args.profile}.py"
    frozen_path.write_text(rendered)
    print(f"Archived {frozen_path} ({datetime.now(timezone.utc).isoformat()})")


if __name__ == "__main__":
    main()
