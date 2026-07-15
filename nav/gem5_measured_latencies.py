"""
gem5_measured_latencies.py — GENERATED FILE, do not edit by hand.

Regenerate with:
    cd native/ape_ops/gem5_bench
    GEM5_ROOT=/path/to/gem5 python3 scripts/freeze_measured_latencies.py --profile cortex_m4_168mhz

Raw per-invocation cycle counts from gem5 cycle-accurate simulation
using an in-order MinorCPU @ 168MHz (see
native/ape_ops/gem5_bench/configs/cortex_m4_168mhz.py — an in-order
MinorCPU APPROXIMATION of STM32F405, NOT a true M-profile simulation;
gem5 has no M-profile CPU model) of the REAL native APE planners
(ape1_bug_plan/ape2_dwa_plan/ape3_vfh_plan — see native/ape_ops/src/),
not a synthetic op-count proxy. Read directly from each profile's
stats.txt ROI block (see native/ape_ops/gem5_bench/bench/src/main.c for
the m5_reset_stats/m5_dump_stats bracketing, against one fixed
open-corridor scan fixture).

This is the ACTIVE simulated CPU for this repo's power model: it feeds
both live deadline-feasibility timing (budget_ms, via
mcu_cycle_model.APE_LATENCY_US) and compute-energy accounting (via
mcu_cycle_model.McuCycleMeter, using ACTIVE_MA/SLEEP_MA/VDD below) — one
processor, one measurement, no separate higher-power companion-computer
model. Other CPU profiles live in native/ape_ops/gem5_bench/configs/ and
past frozen results in native/ape_ops/gem5_bench/out/frozen/ — see
docs/POWER_MODEL.md for the full methodology and multi-profile workflow.
"""

PROFILE_NAME = "cortex_m4_168mhz"
CHIP = "STM32F405"

ITERATIONS = 200
CPU_CLOCK_HZ = 168000000.0  # cortex_m4_168mhz.py's CPU_FREQ (168MHz)

# Power-model parameters for CHIP, at CPU_CLOCK_HZ (see the profile
# module above for the datasheet source).
VDD = 3.3
ACTIVE_MA = 87.0
SLEEP_MA = 59.0

# Total simulated cycles across ITERATIONS invocations, ROI-only
# (process/loader startup excluded).
TOTAL_CYCLES: dict[str, int] = {
    "ape1": 220728,
    "ape2": 1613533,
    "ape3": 4775193,
}
