#!/usr/bin/env python3
"""Print each APE's gem5-measured latency, deadline budget, and per-call
compute energy, read from the checked-in nav/gem5_measured_latencies.py
snapshot -- no gem5/ARM toolchain needed.

To regenerate that snapshot against a different CPU profile or after a
native planner change, use scripts/switch_processor.py instead; this script
only reads whatever is currently frozen there.
"""

from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT))

from nav import gem5_measured_latencies as latencies
from nav import mcu_cycle_model as mcu


def main() -> None:
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
        budget_ms = latency_us / 1000.0
        energy_uj = active_power_w * latency_us  # W * us == uJ, single call (u_eff=1.0)
        print(f"{name:<8} {latency_us:12.2f} {budget_ms:11.2f} {energy_uj:20.3f}")
    print()
    print("energy_per_call_uj assumes the core runs flat-out (active power) for the")
    print("call's full measured latency -- the marginal cost of one invocation, not a")
    print("mission-averaged figure. Mission-level compute energy (idle + active mix")
    print("over wall-clock time) comes from nav.mcu_cycle_model.latency_to_energy_j(),")
    print("used automatically by every `run.py` run via McuCycleMeter.")


if __name__ == "__main__":
    main()
