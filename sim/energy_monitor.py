#!/usr/bin/env python3
# Port of CANavigator's tools/energy_monitor.py::EnergyMonitor.
# E = EPM[J/m] * horizontal_distance. push_pose(x, y, z, t) replaces the ROS
# pose-subscription callback; experiment/orchestrator.py's drive loop calls
# it once per sim tick with t from SimClock.

import math
import logging
from typing import Optional, Tuple
from threading import Lock

# Corrected EPM presets (Transportation Research Part D corrigendum)
EPM_PRESETS = {
    "kirschstein_baseline": 208.9,        # large octocopter, baseline
    "kirschstein_avionics": 213.9,        # + avionics overhead
    "kirschstein_wind45": 489.2,          # strong 45 km/h wind
    "kirschstein_profile10wind": 255.7,   # full profile + 10 km/h wind + avionics
    "flykart30": 208.9,
}


class EnergyMonitor:
    """
    Paper-only energy estimator (pose-only):
      E += EPM[J/m] * d(horizontal distance)
    No aerodynamic/hover/climb terms. Deterministic with smoothing & debounce.
    """

    def __init__(
        self,
        sim_clock,
        *,
        min_step_m: float = 0.01,
        v_tau_s: float = 0.25,
        vz_tau_s: float = 0.25,
        epm_preset: Optional[str] = "flykart30",
        epm_j_per_m: Optional[float] = None,
    ):
        self._log = logging.getLogger(__name__)
        self._lock = Lock()
        self._sim_clock = sim_clock

        self._min_step = max(0.0, float(min_step_m))
        self._v_tau = max(1e-3, float(v_tau_s))
        self._vz_tau = max(1e-3, float(vz_tau_s))

        preset = (epm_preset or "").lower().strip()
        if preset and preset not in EPM_PRESETS:
            preset = "kirschstein_baseline"
        self.Epm = float(epm_j_per_m) if epm_j_per_m is not None else EPM_PRESETS.get(preset or "", 208.9)
        self._epm_preset = preset or None

        self._prev_t: Optional[float] = None
        self._prev_xyz: Optional[Tuple[float, float, float]] = None

        self._v_smooth = 0.0
        self._vz_smooth = 0.0

        self._energy_j: float = 0.0
        self._elapsed_s: float = 0.0
        self._segment_start_t_wall: float = 0.0
        self._segment_label: str = "run"

        self._last_pos = (0.0, 0.0, 0.0)
        self._last_speed = 0.0
        self._last_climb = 0.0
        self._last_power = 0.0

        self._log.info(
            {"epm_j_per_m": round(self.Epm, 1), "epm_preset": self._epm_preset,
             "mode": "paper_only_epm(log on demand)"},
            extra={"type": "LOADEDPARAMS"},
        )

    def push_pose(self, x: float, y: float, z: float, t: float) -> None:
        with self._lock:
            if self._prev_xyz is None:
                self._prev_xyz = (x, y, z)
                self._prev_t = t
                self._last_pos = (x, y, z)
                return

            dt = max(1e-3, t - (self._prev_t or t))
            dx, dy, dz = x - self._prev_xyz[0], y - self._prev_xyz[1], z - self._prev_xyz[2]
            step = math.sqrt(dx*dx + dy*dy + dz*dz)
            step_xy = math.hypot(dx, dy)

            if step < self._min_step:
                vx, vy, vz = 0.0, 0.0, 0.0
                dE = 0.0
            else:
                vx, vy, vz = dx / dt, dy / dt, dz / dt
                dE = self.Epm * max(0.0, step_xy)

            P_equiv = dE / dt if dt > 0 else 0.0

            alpha_v = 1.0 - math.exp(-dt / self._v_tau)
            alpha_vz = 1.0 - math.exp(-dt / self._vz_tau)
            v_xy = math.hypot(vx, vy)
            self._v_smooth = (1 - alpha_v) * self._v_smooth + alpha_v * v_xy
            self._vz_smooth = (1 - alpha_vz) * self._vz_smooth + alpha_vz * vz

            self._energy_j += dE
            self._elapsed_s += dt

            self._last_pos = (x, y, z)
            self._last_speed = self._v_smooth
            self._last_climb = self._vz_smooth
            self._last_power = P_equiv

            self._prev_xyz = (x, y, z)
            self._prev_t = t

    def mark_run_start(self, label: str = "run"):
        with self._lock:
            self._segment_label = label
            self._energy_j = 0.0
            self._elapsed_s = 0.0
            self._segment_start_t_wall = self._sim_clock.now()
            self._prev_xyz = None
            self._prev_t = None

    def log_and_reset(self, label: Optional[str] = None):
        with self._lock:
            lbl = label if label is not None else self._segment_label
            elapsed = max(0.0, self._elapsed_s)
            energy = max(0.0, self._energy_j)
            mean_power = (energy / elapsed) if elapsed > 1e-9 else 0.0

            summary = {
                "label": lbl,
                "wall_started_at": round(self._segment_start_t_wall, 3),
                "elapsed_s": round(elapsed, 3),
                "energy_j": round(energy, 3),
                "mean_power_w": round(mean_power, 1),
                "last": {
                    "pos": {"x": round(self._last_pos[0], 3), "y": round(self._last_pos[1], 3), "z": round(self._last_pos[2], 3)},
                    "speed_mps": round(self._last_speed, 3),
                    "climb_mps": round(self._last_climb, 3),
                    "inst_power_w": round(self._last_power, 1),
                },
                "model": {
                    "name": "epm_only",
                    "epm_j_per_m": round(self.Epm, 1),
                    "epm_preset": self._epm_preset,
                },
            }
            self._log.info(summary, extra={"type": "ENERGYSUMMARY"})

            self._energy_j = 0.0
            self._elapsed_s = 0.0
            self._segment_start_t_wall = self._sim_clock.now()
            self._segment_label = "run"
            return summary
