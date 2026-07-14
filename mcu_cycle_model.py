"""
mcu_cycle_model.py — APE compute latency (gem5-measured) and compute
energy model, both driven by ONE simulated processor: a low-power
flight-controller MCU, the kind of part a genuinely low-compute drone
would actually carry (not a separate, higher-power companion computer).
One gem5 measurement feeds both live deadline-feasibility timing and
energy accounting — there is no second, different CPU model anywhere in
this pipeline.

`gem5_measured_latencies.py` — an in-order MinorCPU, gem5's closest
available approximation of whichever chip its active profile targets
(see native/ape_ops/gem5_bench/configs/ for the available profiles and
their approximation caveats — gem5 has no true M-profile CPU model),
measuring the REAL native APE planners (ape1_bug_plan/ape2_dwa_plan/
ape3_vfh_plan — see native/ape_ops/) directly, not a synthetic op-count
proxy. Feeds `APE_LATENCY_US`, used for:

  1. `nav_algorithm.py`'s `budget_ms` — the live simulator's planning
     deadline / thread top-up sleep target.
  2. `McuCycleMeter`'s per-event compute-energy accounting below.

A small, checked-in, regenerable snapshot (regenerate via
native/ape_ops/gem5_bench/scripts/freeze_measured_latencies.py
--profile <name> after any change to the native planners, the gem5
config, or to switch CPU profile) so this module — and everything that
imports it, including the live navigator — never needs the gem5
toolchain installed just to know APE compute cost. The frozen file also
carries CHIP/ACTIVE_MA/SLEEP_MA/VDD, so the power-model constants below
always match whichever chip the cycle counts were actually measured
against — see docs/POWER_MODEL.md.
"""

from __future__ import annotations

from gem5_measured_latencies import (
    CHIP as _CHIP,
    ITERATIONS as _GEM5_ITERATIONS,
    TOTAL_CYCLES as _GEM5_TOTAL_CYCLES,
    CPU_CLOCK_HZ as _CPU_CLOCK_HZ,
    VDD as _VDD,
    ACTIVE_MA as _ACTIVE_MA,
    SLEEP_MA as _SLEEP_MA,
)

# TOTAL_CYCLES_SEARCH (search-and-rescue exploration workload) is only
# present once gem5_measured_latencies.py has been regenerated against
# the search_apeN fixtures (native/ape_ops/gem5_bench/bench/src/main.c's
# APE_BENCH_MODE=1, native/ape_ops/gem5_bench/scripts/
# freeze_measured_latencies.py) -- the checked-in frozen file predates
# that fixture, so this stays optional rather than crashing every import
# in the meantime.
try:
    from gem5_measured_latencies import TOTAL_CYCLES_SEARCH as _GEM5_TOTAL_CYCLES_SEARCH
except ImportError:
    _GEM5_TOTAL_CYCLES_SEARCH = None

# Cycle-time constant for the gem5 study's clock — converts the study's
# raw TOTAL_CYCLES into microseconds below.
_CYCLE_TIME_US = 1.0e6 / _CPU_CLOCK_HZ

# ---------------------------------------------------------------------------
# gem5-measured latency (µs) — THE SOLE SOURCE OF TRUTH for both live
# deadline-feasibility timing (nav_algorithm.py's budget_ms) and compute
# energy accounting (McuCycleMeter below).
# ---------------------------------------------------------------------------
APE_LATENCY_US: dict[str, float] = {
    "APE1": _GEM5_TOTAL_CYCLES["ape1"] / _GEM5_ITERATIONS * _CYCLE_TIME_US,
    "APE2": _GEM5_TOTAL_CYCLES["ape2"] / _GEM5_ITERATIONS * _CYCLE_TIME_US,
    "APE3": _GEM5_TOTAL_CYCLES["ape3"] / _GEM5_ITERATIONS * _CYCLE_TIME_US,
}

# Same measurement for each planner's search-and-rescue exploration code
# path (target undetected) -- a genuinely different workload per planner
# from APE_LATENCY_US above, not a different input to the same code (see
# native/ape_ops/src/ape{1_bug,2_dwa,3_vfh}.c's target_detected branches).
# None until gem5_measured_latencies.py has been regenerated against the
# search_apeN fixtures -- McuCycleMeter.record_tick() no-ops (contributes
# zero latency/energy) rather than guessing in the meantime.
APE_SEARCH_LATENCY_US: dict[str, float] | None = (
    {
        "APE1": _GEM5_TOTAL_CYCLES_SEARCH["ape1"] / _GEM5_ITERATIONS * _CYCLE_TIME_US,
        "APE2": _GEM5_TOTAL_CYCLES_SEARCH["ape2"] / _GEM5_ITERATIONS * _CYCLE_TIME_US,
        "APE3": _GEM5_TOTAL_CYCLES_SEARCH["ape3"] / _GEM5_ITERATIONS * _CYCLE_TIME_US,
    }
    if _GEM5_TOTAL_CYCLES_SEARCH is not None else None
)

# Which chip the active gem5 profile (gem5_measured_latencies.py) targets.
CHIP: str = _CHIP

# DEADLINE_SCALE: NOT a bare-metal hardware-overhead estimate. A genuinely
# idle Cortex-M4 running only the APE planner would clear even APE3's
# raw gem5 budget (~152µs) with room to spare against this sim's
# 147-650ms threat deadlines -- ISR entry (~tens of ns) and even a full
# FreeRTOS scheduling tick (~1ms) are both negligible against that
# window, so a physically-literal overhead multiplier would make the
# APE1/2/3 deadline race moot (APE3 wins essentially every time).
# DEADLINE_SCALE=1000 is instead a deliberate CONTENTION multiplier: it
# models the APE call competing for the same core with everything else a
# real flight-controller-class MCU is also running without a separate
# companion computer -- sensor fusion/filtering, telemetry, motor-mixer
# control loop, other RTOS tasks -- rather than running alone. That
# contention is what's unverified/unmeasured here (this sim doesn't
# simulate those other tasks), not the underlying gem5 compute numbers
# themselves. budget_ms = APE_LATENCY_US[name] * DEADLINE_SCALE / 1000
# (µs -> ms unit conversion is the trailing /1000; DEADLINE_SCALE is the
# contention factor). See docs/POWER_MODEL.md.
DEADLINE_SCALE: float = 1000.0

# ---------------------------------------------------------------------------
# Power model parameters — derived from the active gem5 profile's chip
# (CHIP/ACTIVE_MA/SLEEP_MA/VDD, frozen into gem5_measured_latencies.py
# alongside the cycle counts they were measured with — see that profile's
# module in native/ape_ops/gem5_bench/configs/ for the datasheet source).
# Cortex-M parts don't publish a "TDP" the way companion-computer SoCs
# do; ACTIVE_POWER_W below is the CPU-running figure, IDLE_FRAC is
# derived from the ratio of Sleep-mode (CPU clock-gated, peripherals on)
# to Run-mode current — Sleep, not Stop, since Stop-mode's near-zero
# draw isn't realistic for a controller sitting idle between
# back-to-back events on a live control loop.
# ---------------------------------------------------------------------------
_ACTIVE_POWER_W: float = _ACTIVE_MA * 1e-3 * _VDD
_IDLE_FRAC:      float = _SLEEP_MA / _ACTIVE_MA
_N_CORES:        int   = 1       # single core


def latency_to_energy_j(total_latency_us: float, wall_s: float) -> float:
    """
    Convert McuCycleMeter total latency to compute energy (Joules).

    Power model (Fan, Weber & Barroso, ISCA 2007), applied to a single
    low-power MCU core rather than a multi-core companion computer:
        P = P_idle + (P_active - P_idle) * U_eff
        E = P * wall_s

    where:
        U_eff = (total_latency_us * 1e-6) / (wall_s * N_cores)

    The frequency-scaling term (f/f_base)^1.5 equals 1.0 because gem5's
    cycle counts already reflect real execution at f_base = CPU_CLOCK_HZ
    (the active profile's clock, from gem5_measured_latencies.py) and is
    therefore omitted.

    Parameters
    ----------
    total_latency_us : total simulated CPU work from McuCycleMeter.end() [µs]
    wall_s           : wall-clock duration of the mission [s]
    """
    if wall_s <= 0.0:
        return 0.0
    p_idle   = _ACTIVE_POWER_W * _IDLE_FRAC
    active_s = total_latency_us * 1e-6
    u_eff    = min(1.0, active_s / (wall_s * _N_CORES))
    p_avg    = p_idle + (_ACTIVE_POWER_W - p_idle) * u_eff
    return p_avg * wall_s


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

class McuCycleMeter:
    """
    Computes total compute latency (µs) for APE workloads on the single
    modeled MCU (in-order core; see module-level CHIP for which chip the
    active profile targets), using the parallel-halt execution model:

      All APEs in a selector run start simultaneously.  When the selected APE
      finishes at T_sel, any still-running APE is halted.  Each APE
      contributes min(its_latency, T_sel).

      Solo modes (APE1/APE2/APE3): one APE runs, cost = that APE's full latency.

      CA parallel-halt cost when `sel` is selected:
        sum(min(APE_LATENCY_US[n], APE_LATENCY_US[sel]) for n in running)
      (not hardcoded here — see APE_LATENCY_US at import time, or call
      record_event() and inspect the result, to avoid this docstring
      drifting from the live model again.)

    This is deliberately the SAME table (APE_LATENCY_US) that
    nav_algorithm.py's budget_ms is derived from — one processor, one
    measurement, used for both live timing and energy accounting. This
    class' energy formula (latency_to_energy_j, above) is parameterized
    with a single low-power MCU core's active/idle power — realistic for
    a low-compute drone running its own navigation stack on its flight
    controller, with no separate companion computer.

    API:
        begin()
        record_event(selected: str, running: list)
        end() -> (total_latency_us: float, per_selected_us: dict[str, float])
    """

    def __init__(self) -> None:
        self._total_us: float = 0.0
        self._per_selected: dict[str, float] = {"APE1": 0.0, "APE2": 0.0, "APE3": 0.0}

    def begin(self) -> None:
        self._total_us = 0.0
        self._per_selected = {"APE1": 0.0, "APE2": 0.0, "APE3": 0.0}

    def record_event(self, selected: str, running: list) -> None:
        """
        Record one event's compute cost under the parallel-halt model.

        selected : APE whose plan was applied ("APE1", "APE2", or "APE3")
        running  : all APEs spawned for this event — ["APE1","APE2","APE3"]
                   for CA, [selected] for solo modes
        """
        t_sel = APE_LATENCY_US.get(selected, 0.0)
        cost  = sum(min(APE_LATENCY_US.get(n, 0.0), t_sel) for n in running)
        self._total_us += cost
        if selected in self._per_selected:
            self._per_selected[selected] += cost

    def record_tick(self, selected: str) -> None:
        """
        Record one search-mode tick's compute cost (nav_algorithm.py's
        _search_native_plan, called every tick while the target is
        undetected). Nominal, gem5-measured APE_SEARCH_LATENCY_US --
        same nominal-table-driven model as record_event, not a per-call
        wall-clock measurement, for consistency with the rest of this
        pipeline's energy accounting. No-op if APE_SEARCH_LATENCY_US
        hasn't been populated yet (see its definition above).

        selected : APE whose search logic ran this tick ("APE1", "APE2",
                   or "APE3")
        """
        if APE_SEARCH_LATENCY_US is None:
            return
        cost = APE_SEARCH_LATENCY_US.get(selected, 0.0)
        self._total_us += cost
        if selected in self._per_selected:
            self._per_selected[selected] += cost

    def end(self) -> tuple:
        return self._total_us, dict(self._per_selected)
