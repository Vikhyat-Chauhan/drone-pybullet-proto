"""
gem5_measured_latencies.py — GENERATED FILE, do not edit by hand.

Regenerate with:
    cd native/ape_ops/gem5_bench
    GEM5_ROOT=/path/to/gem5 python3 scripts/freeze_measured_latencies.py --profile cortex_m4_168mhz

NOTE: ape2/ape3 values below were swapped by hand (not re-run through gem5)
after fixing a bug where bench/src/main.c's APE_BENCH_TARGET==2/3 branches
called the wrong planner (ape3_vfh_plan/ape2_dwa_plan reversed vs.
native_api.c's real ape_native_plan_ape2/ape_native_plan_ape3 dispatch).
Since the fixture/config are otherwise unchanged, the two totals are simply
swapped -- but this should still be re-run through actual gem5 to confirm.

Raw per-invocation cycle counts from gem5 cycle-accurate simulation using
an in-order MinorCPU @ 168MHz (native/ape_ops/gem5_bench/configs/cortex_m4_168mhz.py
— an APPROXIMATION of STM32F405; gem5 has no true M-profile CPU model)
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
    "ape1": 220766,
    "ape2": 4015125,
    "ape3": 1372686,
}
