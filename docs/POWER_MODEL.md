# Power model

This document covers the full power/energy-measurement pipeline used by this
repo: compute power, propulsion power, battery endurance, and the gem5
measurement methodology behind the compute-power side. It replaces
CANavigator's `docs/ARCHITECTURE.md` ("Energy Model" section),
`docs/gem5_power_study.md`, and `docs/ca_architecture_deviations.md`, none of
which exist in this repo — everything referenced from here is self-contained
and regenerable from within `drone-pybullet-proto`.

## 1. Compute power (single MCU model)

`mcu_cycle_model.py` models CPU energy for a low-power flight-controller
MCU — the kind of part a genuinely low-compute drone would actually
carry, running its own navigation stack with no separate companion
computer. **One gem5 measurement feeds both compute-energy accounting
and live deadline-feasibility timing** — there is no second, different
CPU model anywhere in this pipeline (an earlier version of this model
paired a high-power multi-core companion-computer energy model with a
separate MCU-class timing model; that split has been retired in favor of
a single, consistently-modeled low-compute part).

Power model (Fan, Weber & Barroso, ISCA 2007), applied to a single core:

```
P = P_idle + (P_active - P_idle) * U_eff
E = P * wall_s
U_eff = (total_latency_us * 1e-6) / (wall_s * N_cores)
```

**Active profile: STM32F405, Cortex-M4 @ 168 MHz** (`DS8626` Rev 9). This
replaced an earlier STM32H743/Cortex-M7 @ 400 MHz baseline: the H743 is
one of ST's higher-end M-series parts, not representative of what
actually flies. The F405 is the real, widely-documented chip used in
low-compute drones doing onboard obstacle avoidance — the Bitcraze
Crazyflie 2.x nano-drone, and the most common Betaflight/PX4 F4-class
flight controllers.

| Constant | Value | Source |
|---|---|---|
| Active power | 0.287 W | 87 mA @ 3.3V, Run mode, 168 MHz, ART enabled, all peripherals enabled (DS8626 Rev 9, Table 20, Typ, TA=25°C) |
| Idle fraction | 67.8% (~0.195 W) | 59 mA Sleep mode / 87 mA Run mode (DS8626 Rev 9, Table 22, Typ, TA=25°C) |
| N_cores | 1 (single Cortex-M4 core) | — |
| Clock | 168 MHz | matches the active gem5 profile's `CPU_CLOCK_HZ` |

The idle fraction is much higher here (67.8%) than the old H743 baseline
(17.55%) — not a bug. On the F405, Sleep mode only clock-gates the CPU;
with peripherals left enabled (matching how both datasheet tables were
read, for consistency), most of the Run-mode current draw is still there
in Sleep. This is a real property of the chip, not a modeling artifact.

Cortex-M parts don't publish a "TDP" the way companion-computer SoCs do;
`_ACTIVE_POWER_W` is the CPU-running figure, and `_IDLE_FRAC` uses Sleep
mode (CPU clock-gated, peripherals on) rather than Stop mode, since
Stop-mode's near-zero draw isn't realistic for a controller sitting idle
between back-to-back events on a live control loop.

`total_latency_us` comes from `McuCycleMeter`, which sums per-event APE
compute cost under a "parallel-halt" execution model: when running CA
(all three APEs simultaneously), every APE contributes
`min(its_latency, latency_of_selected)`; solo modes (APE1/APE2/APE3) just
use that APE's own latency.

Per-planner latency comes from `APE_LATENCY_US` in `mcu_cycle_model.py`,
derived from `gem5_measured_latencies.py` (§3 below) — real gem5-measured
cycle counts of the actual native planners, not a synthetic op-count
proxy. This is the SAME table `nav_algorithm.py`'s `budget_ms` (live
deadline feasibility) is derived from — one processor, one measurement,
used for both purposes. `ACTIVE_MA`/`SLEEP_MA`/`VDD`/`CHIP` are frozen
into the same generated file, so the power constants above always match
whichever chip the cycle counts were actually measured against — see §3.

### Is this realistic for avoiding a sudden incoming object?

The raw gem5 numbers alone are not the realistic figure here. An idle
Cortex-M4 running only the APE call would clear even APE3's raw budget
(~152 µs) trivially against this sim's 147-650 ms threat deadlines —
neither ISR entry (tens of ns) nor a full FreeRTOS scheduling tick
(~1 ms) comes close to mattering at that timescale. Taken bare-metal, the
deadline race would be moot: APE3 would win essentially every time.

`DEADLINE_SCALE=1000` (`mcu_cycle_model.py`) is therefore not a
bare-metal hardware-overhead figure — it's a deliberate **CPU-contention
stress multiplier**, modeling the APE call sharing the one flight-
controller core with everything else this class of MCU runs without a
separate companion computer (sensor fusion/filtering, telemetry, the
motor-mixer control loop, other RTOS tasks) instead of running alone.
That contention level is asserted, not measured — this sim doesn't model
those other tasks directly — and is the actual free modeling parameter
here, not the underlying gem5 cycle counts.

At `DEADLINE_SCALE=1000`, `budget_ms` comes out to roughly **APE1 ≈
7.1 ms, APE2 ≈ 73.2 ms, APE3 ≈ 151.6 ms**. Published UAV
obstacle-avoidance literature gives a useful sanity check for whether
that's a *plausible* contention level, not a proof of it: Falanga, Kim &
Scaramuzza (*Dynamic obstacle avoidance for quadrotors with event
cameras*, Science Robotics, 2020) report that "today's autonomous drones
have reaction times of tens of milliseconds," and cite field measurements
putting UAS at typical cruise speeds (10-15 m/s) under roughly a 500 ms
total detect→classify→respond budget before a collision becomes
unavoidable. APE1's ~7.1 ms budget sits comfortably inside that window;
APE3's ~151.6 ms budget consumes a meaningful fraction of it, leaving
less margin for sensing/classification latency ahead of it. That's the
concrete justification for this sim's APE1/2/3 racing-selector design
(cascade through faster, lower-quality planners as the deadline tightens,
rather than always waiting for the highest-quality one) — under a
plausible contended-CPU scenario, the slow planner alone can burn through
a meaningful share of the literature-cited reaction-time budget. Under a
bare-metal (uncontended) assumption instead, this justification
evaporates — see `mcu_cycle_model.py`'s `DEADLINE_SCALE` comment.

## 2. Propulsion power (`analysis/power_estimate.py`)

The simulator's own `propulsion_energy_j`/`propulsion_mean_power_w` columns
use a single literature constant (EPM = 208.9 J/m, Kirschstein et al.) with
no hover/drag/battery term — see `energy_monitor.py`. `power_estimate.py`
instead derives propulsion power from momentum (actuator-disk) theory:

- **Hover induced power** (Rankine-Froude): `P = T^1.5 / sqrt(2*rho*A)`
  (Leishman, *Principles of Helicopter Aerodynamics*).
- **Forward-flight induced power**: Glauert's momentum theory (1926) —
  induced velocity solved via fixed-point iteration from
  `v_i * sqrt(V^2 + v_i^2) = v_i_hover^2`. Matters here because simulated
  cruise speeds (~2-5 m/s) are the same order of magnitude as this
  airframe's hover induced velocity (~4.7 m/s), so translational lift is
  non-negligible.
- **Parasite drag power**: `P = 0.5 * rho * (Cd*A) * v^3`, swept over
  `CDA_FRONTAL_GRID_M2 = (0.3, 0.4, 0.5) m^2` as a sensitivity parameter
  (cross-checked in magnitude against `physics.py`'s own `k2=0.04`
  quadratic drag coefficient, Hattenberger et al. 2023).

Ideal-aerodynamic power is converted to real electrical power via an
empirical hover efficiency `eta`, derived from real DJI FlyCart 30 spec-sheet
data (`dji.com/flycart-30/specs`): 65 kg empty mass, dual DB2000 batteries
(3968.8 Wh total), 29 min hover endurance at that weight.

Reference specs used throughout (`FC30_*` constants):

| Constant | Value |
|---|---|
| Empty mass | 65.0 kg (matches `physics.py`'s default `mass_kg`) |
| Rotor count / diameter | 8 x 1.3716 m (54 in) |
| Battery pack | 2x DB2000, 52.22 V x 38000 mAh = 3968.8 Wh total |
| Hover endurance (empty, dual battery) | 29.0 min |

## 3. gem5 measurement methodology (native/ape_ops/gem5_bench/)

`gem5_measured_latencies.py` is a GENERATED file — a real, regenerable
gem5 cycle-accurate measurement of the actual native APE planners
(`native/ape_ops/src/ape1_bug.c`, `ape2_dwa.c`, `ape3_vfh.c`), not
hand-authored numbers. There is no separate, higher-clocked
companion-computer measurement — always exactly one *active* profile.

**Benchmark harness** (`native/ape_ops/gem5_bench/bench/`): `src/main.c`
builds one fixed "open-corridor" LiDAR-style scan fixture (geometry/config
matching this repo's live defaults — `Lidar2D` in `main.py`,
`GoToConfig`/`AvoidCfg`/`RiskCfg`/`EventDecisionCfg`/`AlgoTuning` in
`nav_algorithm.py`) and calls the selected planner
(`ape1_bug_plan`/`ape2_dwa_plan`/`ape3_vfh_plan`) 200 times
(`ITERATIONS`), bracketed by `m5_reset_stats()`/`m5_dump_stats()` so only
the measured loop — not process/loader startup — is counted. One binary per
planner (`bench_ape1`/`bench_ape2`/`bench_ape3`), cross-compiled statically
with `arm-linux-gnueabihf-gcc -marm` (forcing plain ARM/A32 mode: this
toolchain defaults to Thumb with interworking disabled, which corrupts the
return address across the call into gem5's `m5op.S` magic-instruction
stubs under MinorCPU — observed as a page fault at a near-null PC — if
left at the default). This harness is CPU-profile-agnostic — the same
binaries run against any config in `configs/`.

**gem5 CPU profile registry** (`native/ape_ops/gem5_bench/configs/*.py`):
each profile is a thin, self-contained parameter module (CPU preset /
clock / memory type / chip / power-model constants), not full SimObject
wiring — the study runs gem5's own stock ARM Research Starter Kit example
(`configs/example/arm/starter_se.py`, part of any gem5 checkout) via
`scripts/run_gem5_study.py`, rather than re-deriving `MinorCPU` wiring
from scratch: `MinorCPU` (starter_se's "minor" preset), in-order,
single-issue, no speculation — an **approximation** of whichever
Cortex-M-class chip the profile targets. gem5 has no true M-profile CPU
model at all (SE mode itself requires a Linux-ABI userspace target, which
real M-profile MCUs don't run); this in-order/no-speculation shape is the
closest available architectural match. This caveat is inherited from
CANavigator's original methodology and is treated the same way here:
flagged, not hidden.

Available profiles:
- `cortex_m4_168mhz.py` — **active default.** STM32F405, Cortex-M4 @
  168 MHz, DS8626 Rev 9. The real chip in the Bitcraze Crazyflie 2.x
  nano-drone and common Betaflight/PX4 F4-class flight controllers.
- `cortex_m7_400mhz.py` — historical/comparison profile. STM32H743,
  Cortex-M7 @ 400 MHz, DS12110. Retained for comparison; no longer the
  live default (see "Is this realistic?" above for why).

`run_gem5_study.py` discovers profiles dynamically by globbing
`configs/*.py` — adding a new chip/clock is "drop in a new profile
module," no script changes needed.

One caveat worth stating plainly: gem5's `--cpu-freq` is not a purely
cosmetic post-hoc scalar on top of a fixed cycle count — `MinorCPU`'s
memory-access latency is modeled in real (ns) time and converted
internally to core-clock cycles based on the configured frequency, so in
principle `numCycles` is not perfectly frequency-invariant. Empirically,
rerunning `bench_ape1` at 168 MHz vs. 400 MHz shifted `numCycles` by only
about 0.13% (220526-220528 vs. 220817) — small for this benchmark, but
real, which is why each profile is measured with its own gem5 run rather
than hand-scaling one baseline's cycle counts by a frequency ratio.

**Regenerating the latency table** (requires a gem5 checkout with
`build/ARM/gem5.opt` built — `cd $GEM5_ROOT && scons build/ARM/gem5.opt`):

```
cd native/ape_ops/gem5_bench/bench
GEM5_ROOT=/path/to/gem5 make arm
cd ..
GEM5_ROOT=/path/to/gem5 python3 scripts/freeze_measured_latencies.py --profile cortex_m4_168mhz
```

`--profile` defaults to `cortex_m4_168mhz`; pass any other profile name
from `configs/` to switch which chip drives the live model (e.g.
`--profile cortex_m7_400mhz` to go back to the old baseline for
comparison). This rewrites `gem5_measured_latencies.py` at the repo root
— including `CHIP`/`ACTIVE_MA`/`SLEEP_MA`/`VDD`, so `mcu_cycle_model.py`'s
power constants (§1) always match whichever chip's cycles it's reading —
from a fresh 3-run study (one per planner), and archives a timestamped
copy under `native/ape_ops/gem5_bench/out/frozen/<profile>.py` so past
profiles' numbers stay comparable without rerunning gem5. Re-run after
any change to the native planner sources, a profile's parameters, or to
switch the active CPU/algorithm. Run outputs (`stats.txt` etc.) land in
`native/ape_ops/gem5_bench/out/mcu/<profile>/`.

## 4. Combined estimate (`analysis/statistics_analyzer.py`)

Per-run compute power + physically-grounded propulsion power are combined
into `total_power_w`, `energy_per_mission_kj`, and `endurance_min`
(`FC30_BATTERY_WH_TOTAL * 3600 / total_power_w / 60`), aggregated per
strategy (APE1/APE2/APE3/CA) alongside a sensitivity grid over the
efficiency (`ETA_GRID`) and drag-area (`CDA_FRONTAL_GRID_M2`) parameters.
