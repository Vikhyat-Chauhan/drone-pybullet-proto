#!/usr/bin/env python3
"""Pipeline: switch the simulated flight-controller CPU and regenerate the
latency/energy numbers that flow from it.

Wraps the manual sequence described in docs/POWER_MODEL.md §3 into one
command:

  1. Discover available CPU profiles from native/ape_ops/gem5_bench/configs/
     (cortex_m4_168mhz.py, cortex_m7_400mhz.py, or any new profile module
     dropped in there -- see run_gem5_study.available_profiles()).
  2. Build the gem5 benchmark binaries (bench_ape1/2/3) against the real
     native planners if they aren't already built for this checkout.
  3. Run each planner's benchmark under gem5 SE mode for the chosen
     profile (native/ape_ops/gem5_bench/scripts/freeze_measured_latencies.py),
     which rewrites nav/gem5_measured_latencies.py -- the live simulator's
     one source of truth for both deadline timing and compute energy.
  4. Reload nav.mcu_cycle_model against the freshly written table and print
     a summary: per-planner latency (µs), live deadline budget (ms), and
     active-power energy per single invocation (µJ).

Requires GEM5_ROOT (a gem5 checkout with build/ARM/gem5.opt built) and an
ARM cross-compiler (arm-linux-gnueabihf-gcc) -- see docs/POWER_MODEL.md §3
to build gem5 itself. Nothing downstream of nav/gem5_measured_latencies.py
(the live simulator, `make smoke`) needs either; this pipeline only
regenerates that one file.

Usage:
    python scripts/switch_processor.py --list
    GEM5_ROOT=/path/to/gem5 python scripts/switch_processor.py --profile cortex_m7_400mhz
    GEM5_ROOT=/path/to/gem5 python scripts/switch_processor.py            # uses the current default profile
"""
from __future__ import annotations

import argparse
import importlib
import os
import subprocess
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
_GEM5_BENCH_DIR = _REPO_ROOT / "native" / "ape_ops" / "gem5_bench"
_BENCH_DIR = _GEM5_BENCH_DIR / "bench"
_SCRIPTS_DIR = _GEM5_BENCH_DIR / "scripts"

sys.path.insert(0, str(_SCRIPTS_DIR))
from run_gem5_study import available_profiles  # noqa: E402


def _ensure_bench_binaries_built() -> None:
    missing = [
        p for p in ("bench_ape1", "bench_ape2", "bench_ape3")
        if not (_BENCH_DIR / "build" / "arm" / p).exists()
    ]
    if not missing:
        return
    print(f"Building {', '.join(missing)} (GEM5_ROOT={os.environ.get('GEM5_ROOT')}) ...")
    subprocess.run(["make", "arm"], cwd=_BENCH_DIR, check=True)


def _regenerate(profile: str) -> None:
    subprocess.run(
        [sys.executable, str(_SCRIPTS_DIR / "freeze_measured_latencies.py"), "--profile", profile],
        cwd=_GEM5_BENCH_DIR, check=True,
    )


def _print_summary() -> None:
    # Fresh imports, not reload: not necessarily already imported in this
    # process, and must read the just-regenerated latencies file.
    sys.path.insert(0, str(_REPO_ROOT))
    for mod in ("nav.gem5_measured_latencies", "nav.mcu_cycle_model"):
        sys.modules.pop(mod, None)
    latencies = importlib.import_module("nav.gem5_measured_latencies")
    mcu = importlib.import_module("nav.mcu_cycle_model")

    active_power_w = mcu._ACTIVE_POWER_W
    idle_power_w = active_power_w * mcu._IDLE_FRAC

    print()
    print(f"Active profile: {latencies.PROFILE_NAME}  ({latencies.CHIP} @ "
          f"{latencies.CPU_CLOCK_HZ / 1e6:.0f} MHz)")
    print(f"Power: active={active_power_w:.3f} W  idle={idle_power_w:.3f} W  "
          f"(VDD={latencies.VDD} V, ACTIVE_MA={latencies.ACTIVE_MA}, SLEEP_MA={latencies.SLEEP_MA})")
    print()
    header = f"{'planner':<8} {'latency_us':>12} {'budget_ms':>11} {'energy_per_call_uj':>20}"
    print(header)
    print("-" * len(header))
    for name in ("APE1", "APE2", "APE3"):
        latency_us = mcu.APE_LATENCY_US[name]
        budget_ms = latency_us * mcu.DEADLINE_SCALE / 1000.0
        energy_uj = active_power_w * latency_us  # W * us == uJ, single call (u_eff=1.0)
        print(f"{name:<8} {latency_us:12.2f} {budget_ms:11.2f} {energy_uj:20.3f}")
    print()
    print("energy_per_call_uj assumes the core runs flat-out (active power) for the")
    print("call's full measured latency -- the marginal cost of one invocation, not a")
    print("mission-averaged figure. Mission-level compute energy (idle + active mix")
    print("over wall-clock time) comes from nav.mcu_cycle_model.latency_to_energy_j(),")
    print("used automatically by every `run.py` run via McuCycleMeter.")


def main() -> None:
    profiles = available_profiles()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--profile", choices=profiles, default=None,
                     help="CPU profile to switch to (default: keep the current active one, just re-measure it)")
    ap.add_argument("--list", action="store_true", help="list available profiles and exit")
    args = ap.parse_args()

    if args.list:
        for name in profiles:
            print(name)
        return

    if not os.environ.get("GEM5_ROOT"):
        print("GEM5_ROOT must be set to a gem5 checkout with build/ARM/gem5.opt built.\n"
              "See docs/POWER_MODEL.md §3 for how to build one.", file=sys.stderr)
        sys.exit(1)

    profile = args.profile
    if profile is None:
        sys.path.insert(0, str(_REPO_ROOT))
        from nav.gem5_measured_latencies import PROFILE_NAME
        profile = PROFILE_NAME

    _ensure_bench_binaries_built()
    _regenerate(profile)
    _print_summary()


if __name__ == "__main__":
    main()
