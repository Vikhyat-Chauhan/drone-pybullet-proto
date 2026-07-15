#!/usr/bin/env python3
"""HUD text formatting for the OS-level metrics overlay window
(viz.hud_overlay.MetricsOverlay).
"""
from nav.mcu_cycle_model import latency_to_energy_j


def format_hud_text(strategy: str, status: str, dist: float, sim_now: float,
                     nav, violations, energy) -> str:
    """All the metrics of the current run, for the OS-level overlay window
    (viz.hud_overlay.MetricsOverlay) -- not embedded in the 3D world at all
    (an earlier version anchored debug text to a fixed screen position by
    inverting the chase-cam's view matrix each tick; geometrically correct
    but looked bad in practice, so the panel now lives in its own window).
    Reads live private counters directly off nav/violations/energy
    (nav._events_handled etc., violations._total_violations,
    energy._energy_j) -- an established pattern in this port already.

    Compute energy uses the exact same formula nav/algorithm.py's
    end_mission() uses -- McuCycleMeter.end() is a pure getter (no reset;
    only begin_mission()/reset() clear it), so calling it mid-run is safe
    and gives a live running total, not just a final one.
    """
    handled = nav._events_handled
    violated = nav._events_violated
    deadline_v = nav._events_violated_deadline
    preempt_v = nav._events_violated_preemptive
    zone_v = violations._total_violations
    energy_j = energy._energy_j
    total_latency_us, _ = nav._cycle_meter.end()
    compute_energy_j = latency_to_energy_j(total_latency_us, max(1e-6, sim_now - nav._t_start))
    return (
        f"{strategy}\n"
        f"status: {status}\n"
        f"dist: {dist:6.1f} m\n"
        f"elapsed: {sim_now:6.1f} s\n"
        f"events: {handled}  violated: {violated}\n"
        f"  deadline: {deadline_v}  preempt: {preempt_v}\n"
        f"zone violations: {zone_v}\n"
        f"energy: {energy_j:8.0f} J\n"
        f"compute energy: {compute_energy_j:6.2f} J"
    )
