"""
cortex_m7_400mhz.py — gem5 SE-mode study definition for the
Cortex-M7-APPROXIMATION measurement of the real APE planners.

gem5 has no true M-profile (Cortex-M) CPU model at all — SE mode itself
requires a Linux-ABI userspace target, which real M-profile MCUs don't
run. This study instead uses gem5's in-order MinorCPU (via
configs/example/arm/starter_se.py "minor" preset) clocked at 400 MHz as
the closest available approximation: in-order, single-issue, no
speculation — architecturally the right *shape* for a Cortex-M7-class
core, even though the ISA underneath (ARMv7/v8 SE-mode, not true
M-profile Thumb2-only) is not identical. This caveat is inherited from
CANavigator's original methodology and is treated the same way here:
flagged, not hidden.

This module holds ONLY the parameters for one gem5 CPU study (timing +
power) — see ../scripts/run_gem5_study.py for how it's invoked, and
../scripts/freeze_measured_latencies.py --profile cortex_m7_400mhz to
regenerate gem5_measured_latencies.py against it. This is a historical/
comparison profile, superseded as the live default by
cortex_m4_168mhz.py (a more representative low-compute flight-controller
chip) — see docs/POWER_MODEL.md.
"""

CPU_PRESET = "minor"        # starter_se.py's cpu_types["minor"] = MinorCPU
CPU_FREQ = "400MHz"          # matches gem5_measured_latencies.py's CPU_CLOCK_HZ
MEM_TYPE = "DDR3_1600_8x8"

# Chip this profile approximates, and its power-model parameters —
# STM32H743 (Cortex-M7), DS12110, Run/Sleep mode current at VOS0, 400 MHz, 3.3V.
CHIP = "STM32H743"
VDD = 3.3
ACTIVE_MA = 285.0  # DS12110, Run mode, 400 MHz, Typ
SLEEP_MA = 50.0    # DS12110, Sleep mode, Typ
