"""
cortex_m4_168mhz.py — gem5 SE-mode study definition for the
Cortex-M4-APPROXIMATION measurement of the real APE planners, targeting
an STM32F405 @ 168 MHz — the live default profile (see
docs/POWER_MODEL.md).

Chosen over the higher-end STM32H743/Cortex-M7 @ 400 MHz
(cortex_m7_400mhz.py) because it's the actual, widely-documented MCU
found on low-compute drones doing onboard obstacle avoidance: the
Bitcraze Crazyflie 2.x nano-drone, and the most common Betaflight/PX4
F4-class flight controllers. Same gem5-approximation caveats apply as
cortex_m7_400mhz.py: gem5 has no true M-profile CPU model, so this uses
the same in-order MinorCPU "minor" preset (architecturally the right
*shape* — in-order, single-issue, no speculation — for a Cortex-M4
core) at a different --cpu-freq. An empirical check in this repo
(rerunning bench_ape1 at 168MHz vs. 400MHz) found gem5's measured
numCycles shifts by only ~0.13% between the two frequencies (memory
access latency is modeled in fixed ns and converted to core-clock
cycles, so it's not perfectly frequency-invariant) — small, but real,
which is why this profile is measured via its own gem5 run rather than
a post-hoc scalar applied to the 400MHz cycle counts.

This module holds ONLY the parameters for one gem5 CPU study (timing +
power) — see ../scripts/run_gem5_study.py for how it's invoked, and
../scripts/freeze_measured_latencies.py --profile cortex_m4_168mhz
(the default) to regenerate gem5_measured_latencies.py against it.
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
