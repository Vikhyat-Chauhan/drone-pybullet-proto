#!/usr/bin/env python3
"""
remeasure.py — fast-iteration helper, NOT part of the normal pipeline.
Re-measures only the named planner(s) under gem5 and patches their
TOTAL_CYCLES entries into nav/gem5_measured_latencies.py in place,
leaving the other planners' entries untouched.

Exists because full freeze_measured_latencies.py always re-measures all
three planners, and ape3 (RHTP) alone takes ~13 minutes of gem5
cycle-accurate simulation -- a heavy tax to pay on every calibration
iteration touching only one or two planners. Once final values are
chosen, re-run the normal `python scripts/switch_processor.py` for a
clean, canonical full regeneration (already verified reproducible this
session).

Usage:
    GEM5_ROOT=/path/to/gem5 python3 remeasure.py ape1 ape2
    GEM5_ROOT=/path/to/gem5 python3 remeasure.py ape3
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_gem5_study import run_study, parse_num_cycles

_HERE = Path(__file__).resolve().parent
_GEM5_BENCH_DIR = _HERE.parent
_REPO_ROOT = _GEM5_BENCH_DIR.parent.parent.parent
_LATENCIES_PY = _REPO_ROOT / "nav" / "gem5_measured_latencies.py"
_BENCH_DIR = _GEM5_BENCH_DIR / "bench"
PROFILE = "cortex_m3_8mhz"
ITERATIONS = 200


def main() -> None:
    planners = sys.argv[1:] or ["ape1", "ape2"]
    for planner in planners:
        binary = _BENCH_DIR / "build" / "arm" / f"bench_{planner}"
        if not binary.exists():
            raise FileNotFoundError(f"{binary} not found -- build it first")

    total_cycles = {}
    for planner in planners:
        binary = _BENCH_DIR / "build" / "arm" / f"bench_{planner}"
        outdir = _GEM5_BENCH_DIR / "out" / "mcu" / PROFILE / planner
        stats_path = run_study(PROFILE, binary, outdir)
        total_cycles[planner] = parse_num_cycles(stats_path)
        print(f"{planner}: TOTAL_CYCLES = {total_cycles[planner]}")

    text = _LATENCIES_PY.read_text()
    for planner, cycles in total_cycles.items():
        text = re.sub(
            rf'("{planner}":\s*)\d+',
            rf'\g<1>{cycles}',
            text,
        )
    _LATENCIES_PY.write_text(text)
    print(f"Patched {_LATENCIES_PY} ({', '.join(planners)} only, others left as-is)")


if __name__ == "__main__":
    main()
