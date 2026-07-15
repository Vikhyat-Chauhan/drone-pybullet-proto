#!/usr/bin/env python3
"""
freeze_measured_latencies.py — regenerates nav/gem5_measured_latencies.py
from a fresh gem5 cycle-accurate run of the real native APE planners
(ape1_bug_plan/dwa_plan/vfh_plan — native/ape_ops/src/; which of
dwa_plan/vfh_plan runs under the "ape2"/"ape3" label is whatever
native_api.c's ape_native_plan_ape2/ape_native_plan_ape3 dispatch to)
against a named CPU profile from ../configs/ (default: cortex_m4_168mhz.py,
STM32F405/Cortex-M4 @ 168MHz, this repo's target MCU; see
docs/POWER_MODEL.md). Feeds both live deadline-feasibility timing
(budget_ms) and compute-energy accounting.

Usage:
    GEM5_ROOT=/path/to/gem5 python3 freeze_measured_latencies.py [--profile NAME]

Requires:
    - native/ape_ops/gem5_bench/bench/build/arm/bench_ape{1,2,3}
      (build with: GEM5_ROOT=... make -C ../bench arm)
    - a gem5 checkout with build/ARM/gem5.opt built
      (cd $GEM5_ROOT && scons build/ARM/gem5.opt)

To switch CPU: add a profile module to ../configs/
(CPU_PRESET/CPU_FREQ/MEM_TYPE/CHIP/VDD/ACTIVE_MA/SLEEP_MA), then rerun
with --profile <name> -- run_gem5_study.py discovers profiles dynamically
and nav/mcu_cycle_model.py reads its constants from the regenerated file,
no other code changes needed.

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


def _clock_hz_from_freq_str(freq: str) -> float:
    """Parses a gem5 --cpu-freq string like '168MHz' into Hz."""
    freq = freq.strip()
    if freq.upper().endswith("MHZ"):
        return float(freq[:-3]) * 1_000_000.0
    if freq.upper().endswith("GHZ"):
        return float(freq[:-3]) * 1_000_000_000.0
    raise ValueError(f"unrecognized CPU_FREQ format: {freq!r}")


def _render(profile_name: str, study, total_cycles: dict[str, int]) -> str:
    cpu_clock_hz = _clock_hz_from_freq_str(study.CPU_FREQ)
    return f'''"""
gem5_measured_latencies.py — GENERATED FILE, do not edit by hand.

Regenerate with:
    cd native/ape_ops/gem5_bench
    GEM5_ROOT=/path/to/gem5 python3 scripts/freeze_measured_latencies.py --profile {profile_name}

Raw per-invocation cycle counts from gem5 cycle-accurate simulation using
an in-order MinorCPU @ {study.CPU_FREQ} (native/ape_ops/gem5_bench/configs/{profile_name}.py
— an APPROXIMATION of {study.CHIP}; gem5 has no true M-profile CPU model)
of the REAL native APE planners (ape1_bug_plan/dwa_plan/vfh_plan
— native/ape_ops/src/), not a synthetic op-count proxy. Read from each
profile's stats.txt ROI block (see main.c's m5_reset_stats/m5_dump_stats
bracketing around one fixed open-corridor scan fixture).

The ACTIVE simulated CPU for this repo's power model: feeds both live
deadline-feasibility timing (budget_ms, via mcu_cycle_model.APE_LATENCY_US)
and compute-energy accounting (via McuCycleMeter, using ACTIVE_MA/SLEEP_MA/VDD
below) — one processor, one measurement. Other profiles live in
native/ape_ops/gem5_bench/configs/; see docs/POWER_MODEL.md for the full
methodology.
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

    rendered = _render(args.profile, study, total_cycles)

    out_path = _REPO_ROOT / "nav" / "gem5_measured_latencies.py"
    out_path.write_text(rendered)
    print(f"Wrote {out_path}")

    frozen_dir = _GEM5_BENCH_DIR / "out" / "frozen"
    frozen_dir.mkdir(parents=True, exist_ok=True)
    frozen_path = frozen_dir / f"{args.profile}.py"
    frozen_path.write_text(rendered)
    print(f"Archived {frozen_path} ({datetime.now(timezone.utc).isoformat()})")


if __name__ == "__main__":
    main()
