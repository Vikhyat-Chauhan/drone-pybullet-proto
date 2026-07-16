"""
mcu_cycle_model.py — APE compute latency (gem5-measured) and compute
energy model, both driven by ONE simulated processor: a low-power
flight-controller MCU (no separate companion computer). One gem5
measurement feeds both deadline-feasibility timing and energy accounting.

`gem5_measured_latencies.py` is an in-order MinorCPU approximation of the
active profile's chip (see native/ape_ops/gem5_bench/configs/), measuring
the REAL native APE planners directly, not a synthetic op-count proxy.
Feeds `APE_LATENCY_US`, used for (1) `algorithm.py`'s `budget_ms` deadline
and (2) `McuCycleMeter`'s per-event energy accounting below.

It's a checked-in, regenerable snapshot (regenerate via
native/ape_ops/gem5_bench/scripts/freeze_measured_latencies.py
--profile <name> after changing the native planners, gem5 config, or CPU
profile) so nothing here needs the gem5 toolchain installed at runtime.
It also carries CHIP/ACTIVE_MA/SLEEP_MA/VDD so the power constants below
always match the chip the cycles were measured against — see
docs/POWER_MODEL.md.
"""

from __future__ import annotations

from nav.gem5_measured_latencies import (
    CHIP as _CHIP,
    ITERATIONS as _GEM5_ITERATIONS,
    TOTAL_CYCLES as _GEM5_TOTAL_CYCLES,
    CPU_CLOCK_HZ as _CPU_CLOCK_HZ,
    VDD as _VDD,
    ACTIVE_MA as _ACTIVE_MA,
    SLEEP_MA as _SLEEP_MA,
)

# Cycle-time constant for the gem5 study's clock — converts the study's
# raw TOTAL_CYCLES into microseconds below.
_CYCLE_TIME_US = 1.0e6 / _CPU_CLOCK_HZ

# ---------------------------------------------------------------------------
# gem5-measured latency (µs) — THE SOLE SOURCE OF TRUTH for both live
# deadline-feasibility timing (algorithm.py's budget_ms) and compute
# energy accounting (McuCycleMeter below).
# ---------------------------------------------------------------------------
APE_LATENCY_US: dict[str, float] = {
    "APE1": _GEM5_TOTAL_CYCLES["ape1"] / _GEM5_ITERATIONS * _CYCLE_TIME_US,
    "APE2": _GEM5_TOTAL_CYCLES["ape2"] / _GEM5_ITERATIONS * _CYCLE_TIME_US,
    "APE3": _GEM5_TOTAL_CYCLES["ape3"] / _GEM5_ITERATIONS * _CYCLE_TIME_US,
}

# Which chip the active gem5 profile (gem5_measured_latencies.py) targets.
CHIP: str = _CHIP

# ---------------------------------------------------------------------------
# Power model parameters, from the active gem5 profile's chip
# (CHIP/ACTIVE_MA/SLEEP_MA/VDD, frozen alongside its cycle counts — see
# native/ape_ops/gem5_bench/configs/ for datasheet sources). Cortex-M parts
# don't publish a "TDP"; ACTIVE_POWER_W is the CPU-running figure, IDLE_FRAC
# is Sleep-mode/Run-mode current ratio (Sleep, not Stop-mode, since Stop's
# near-zero draw isn't realistic for a controller idling between events).
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
    modeled MCU, using the parallel-halt execution model: all APEs in a
    selector run start simultaneously; when the event is resolved at the
    real elapsed halt instant T_halt (a planner's own ready time, or a
    later deadline-driven fallback instant if resolution had to wait on
    a higher-priority planner that never became ready in time), any
    still-running APE is halted, contributing min(its_latency, T_halt).
    Solo modes cost that APE's full latency (T_halt == its own latency,
    since there is nothing else to wait on).

    Uses the same APE_LATENCY_US table algorithm.py's budget_ms derives
    from, and latency_to_energy_j's single-MCU-core power model.

    API:
        begin()
        record_event(selected: str, running: list, halt_us: float)
        end() -> (total_latency_us: float, per_selected_us: dict[str, float])
    """

    def __init__(self) -> None:
        self._total_us: float = 0.0
        self._per_selected: dict[str, float] = {"APE1": 0.0, "APE2": 0.0, "APE3": 0.0}

    def begin(self) -> None:
        self._total_us = 0.0
        self._per_selected = {"APE1": 0.0, "APE2": 0.0, "APE3": 0.0}

    def record_event(self, selected: str, running: list, halt_us: float) -> None:
        """
        Record one event's compute cost under the parallel-halt model.

        selected : APE whose plan was applied ("APE1", "APE2", or "APE3")
        running  : all APEs spawned for this event — ["APE1","APE2","APE3"]
                   for CA, [selected] for solo modes
        halt_us  : real elapsed time (arrival to resolution) at which any
                   still-running APE was halted — the caller's actual
                   simulated timeline, not a value re-derived from the
                   static latency table.
        """
        cost = sum(min(APE_LATENCY_US.get(n, 0.0), halt_us) for n in running)
        self._total_us += cost
        if selected in self._per_selected:
            self._per_selected[selected] += cost

    def end(self) -> tuple:
        return self._total_us, dict(self._per_selected)
