"""
cortex_m3_8mhz.py — gem5 SE-mode study definition for the
Cortex-M3-APPROXIMATION measurement of the real APE planners, targeting
an STM32F103C8 ("Blue Pill") running off its internal 8MHz HSI RC
oscillator (not the full 72MHz external/PLL clock) — the live default
profile (see docs/POWER_MODEL.md) — a genuine low-power operating point
of a real, ubiquitous industry chip: the STM32F103 is the MCU behind the
Naze32/CC3D generation of flight controllers (the F1-class predecessor to
the F4-class STM32F405 in cortex_m4_168mhz.py) and one of the most widely
manufactured general-purpose MCUs in industry overall. Running it off HSI
instead of the PLL-boosted 72MHz is itself a common real firmware choice
on battery-constrained boards to trade compute throughput for power --
not a synthetic clock-down.

Purpose: cortex_m4_168mhz.py/cortex_m7_400mhz.py's APE budgets are
dominated by APE3's own compute cost but land in the microsecond-to-tens-
of-milliseconds range -- realistic for the chips they model, but too fast
to show meaningful interaction with the sim's deadline model (deadline_
min_ms/deadline_max_ms in conf/events/, and the ~500ms UAV detect-classify-
respond ceiling cited in docs/POWER_MODEL.md). This profile is 21x slower
in raw clock (168MHz / 8MHz) than cortex_m4_168mhz.py, which -- combined
with APE3 being the dominant cost -- pushes its budget from ~19.6ms to
~412ms: comparable to, rather than negligible against, the timescale of a
physically incoming object the sim's deadline model is meant to
represent (APE1/APE2 correspondingly move from microseconds to ~0.14ms/
~2.5ms -- still fast, but no longer negligible, so the APE1/2/3 racing-
selector cascade is now exercising real relative timing rather than an
always-trivial APE1/APE2 win). Same gem5-approximation caveats apply as
the other profiles here: gem5 has no true M-profile CPU model, so this
uses the same in-order MinorCPU "minor" preset (architecturally the right
*shape* -- in-order, single-issue, no speculation -- for a Cortex-M3
core, even without its narrower 3-stage pipeline / no-FPU specifics) at a
different --cpu-freq.

This module holds ONLY the parameters for one gem5 CPU study (timing +
power) — see ../scripts/run_gem5_study.py for how it's invoked, and
../scripts/freeze_measured_latencies.py --profile cortex_m3_8mhz
(the default) to regenerate gem5_measured_latencies.py against it.
"""

CPU_PRESET = "minor"        # starter_se.py's cpu_types["minor"] = MinorCPU
CPU_FREQ = "8MHz"
MEM_TYPE = "DDR3_1600_8x8"

# Chip this profile approximates, and its power-model parameters --
# STM32F103C8 (Cortex-M3), DS5319, Run mode current at the internal 8MHz
# HSI RC oscillator, 3.3V. Both numbers below are APPROXIMATE: the primary
# DS5319 datasheet PDF could not be fetched directly in this environment
# (ST's site timed out repeatedly), so they're triangulated from secondary
# references rather than read off the table directly, unlike
# cortex_m4_168mhz.py/cortex_m7_400mhz.py's power constants -- re-derive
# both once the datasheet table itself can be read.
#   ACTIVE_MA: DS5319's Run-mode table (code with data processing running
#   from Flash, all peripherals disabled) is cited secondhand as ~4-5mA
#   Typ at 8MHz/HSI; takes the upper end as a conservative estimate.
#   SLEEP_MA: DS5319's Sleep-mode Typ current (CPU clock-gated, bus/
#   peripheral clocks still running off SYSCLK) is only confirmed at
#   72MHz (~5.5mA Typ, matching a ~5.9mA measured figure from a third-
#   party evaluation) -- frequency-scaled linearly down to 8MHz
#   (5.5 * 8/72) since Sleep-mode current on this family is dominated by
#   bus/peripheral clocking rather than core execution, similarly to Run
#   mode's near-linear mA/MHz scaling.
CHIP = "STM32F103C8"
VDD = 3.3
ACTIVE_MA = 5.0   # DS5319, Run mode, HSI 8MHz, Typ, TA=25C (approximate -- see note above)
SLEEP_MA = 0.61   # DS5319, Sleep mode, Typ, TA=25C, freq-scaled from 72MHz (approximate -- see note above)
