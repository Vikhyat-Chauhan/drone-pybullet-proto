"""
cortex_m4_168mhz.py — gem5 SE-mode study definition for the
Cortex-M4-APPROXIMATION measurement of the real APE planners, targeting
an STM32F405 @ 168 MHz. Historical/comparison profile, superseded as the
live default by cortex_m3_8mhz.py -- APE3's own compute cost grew large
enough (a deeper search-based planner) that this chip's budgets
(APE1/APE2 microseconds, APE3 ~19.6ms) were negligible against the sim's
deadline model rather than physically comparable to it; see
docs/POWER_MODEL.md's "Is this realistic?" section.

Chosen over the higher-end STM32H743/Cortex-M7 @ 400 MHz
(cortex_m7_400mhz.py) because it's the actual, widely-documented MCU
found on low-compute drones doing onboard obstacle avoidance: the
Bitcraze Crazyflie 2.x nano-drone, and the most common Betaflight/PX4
F4-class flight controllers. Same gem5-approximation caveats apply as
cortex_m7_400mhz.py: gem5 has no true M-profile CPU model, so this uses
the same in-order MinorCPU "minor" preset (architecturally the right
*shape* — in-order, single-issue, no speculation — for a Cortex-M4
core) at a different --cpu-freq.

This module holds ONLY the parameters for one gem5 CPU study (timing +
power) — see ../scripts/run_gem5_study.py for how it's invoked, and
../scripts/freeze_measured_latencies.py --profile cortex_m4_168mhz
to regenerate gem5_measured_latencies.py against it (no longer the
default profile that flag falls back to).
"""

CPU_PRESET = "minor"        # starter_se.py's cpu_types["minor"] = MinorCPU
CPU_FREQ = "168MHz"
MEM_TYPE = "DDR3_1600_8x8"

# Chip this profile approximates, and its power-model parameters —
# STM32F405 (Cortex-M4), DS8626 Rev 9, Run/Sleep mode current, 168 MHz, 3.3V.
CHIP = "STM32F405"
VDD = 3.3
ACTIVE_MA = 87.0  # DS8626 Rev 9, Table 20 (Run, ART enabled, all peripherals enabled, 168MHz, Typ, TA=25C)
SLEEP_MA = 59.0   # DS8626 Rev 9, Table 22 (Sleep, all peripherals enabled, 168MHz, Typ, TA=25C)
