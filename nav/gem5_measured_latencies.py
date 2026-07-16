"""
gem5_measured_latencies.py — GENERATED FILE, do not edit by hand.

Regenerate with:
    cd native/ape_ops/gem5_bench
    GEM5_ROOT=/path/to/gem5 python3 scripts/freeze_measured_latencies.py --profile cortex_m3_8mhz

Raw per-invocation cycle counts from gem5 cycle-accurate simulation using
an in-order MinorCPU @ 8MHz (native/ape_ops/gem5_bench/configs/cortex_m3_8mhz.py
— an APPROXIMATION of STM32F103C8; gem5 has no true M-profile CPU model)
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

PROFILE_NAME = "cortex_m3_8mhz"
CHIP = "STM32F103C8"

ITERATIONS = 200
CPU_CLOCK_HZ = 8000000.0  # cortex_m3_8mhz.py's CPU_FREQ (8MHz)

# Power-model parameters for CHIP, at CPU_CLOCK_HZ (see the profile
# module above for the datasheet source).
VDD = 3.3
ACTIVE_MA = 5.0
SLEEP_MA = 0.61

# Total simulated cycles across ITERATIONS invocations, ROI-only
# (process/loader startup excluded).
TOTAL_CYCLES: dict[str, int] = {
    "ape1": 227565,
    "ape2": 1971746,
    "ape3": 332606894,
}
