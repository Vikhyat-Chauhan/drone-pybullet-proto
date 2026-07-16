# config.py
from dataclasses import dataclass, field
import os

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))


@dataclass
class TeleopConfig:
    # --- Run Options ---
    simulation_runs: int = 1
    simulation_timeout: float = 200
    simulation_world_style: str = "city"  # "perlin|city"

    # --- World gen ---
    fixed_seed: bool = False
    world_gen_seed_offset: int = 7777
    # Matches CANavigator's original exactly (arena.ArenaCfg's domain is
    # back to -100..100 x -50..50, same as the Gazebo world).
    target_distance: float = 150

    # --- PyBullet sim ---
    flight_z: float = 3.0
    drone_radius_m: float = 0.7
    entity_name: str = "drone1"

    # --- Nav/teleop rate ---
    rate_hz: float = 100

    # --- Logging ---
    log_path: str = "logs/run_logs.json"

    # =======================
    # RESULTS & ANALYZER CFG
    # =======================
    results_csv_path: str = "logs/results/experiment_summary.csv"
    analyzer_out_dir: str = "logs/results"
    analyzer_strategies: list = field(default_factory=lambda: ["APE1", "APE2", "APE3", "CA"])

    # =======================
    # Generated Simulation & Algo Selector
    # =======================
    # Matches CANavigator's original fixed spawn point exactly.
    start_x: float = -95.0
    start_y: float = 0.0
    start_z: float = 1.0
    start_yaw: float = 0.0  # radians

    target_json_path: str = "models/generated/generated_target_meta.json"
    nofly_meta_path: str = "models/generated/generated_nofly_meta.json"

    # =======================
    # Event Generator (B-style)
    # =======================
    event_seed: int = 42
    event_deterministic: bool = True

    # Inter-event gap is drawn log-uniformly from [event_dt_min_ms, event_dt_max_ms].
    # event_dt_min_ms must stay <= rate_hz's tick period (in ms) so a fast burst of
    # events never silently skips a tick (see experiment/orchestrator.py's drive loop).
    # Config-facing units are ms; converted to seconds at the EventCfg boundary
    # (experiment/orchestrator.py) since the simulation engine itself (SimClock,
    # sim/event_source.py, nav/algorithm.py) runs entirely in seconds.
    event_dt_min_ms: float = 20.0
    event_dt_max_ms: float = 4000.0

    event_mix_enemy: float = 0.33
    event_mix_obstacle: float = 0.33
    event_mix_lane: float = 0.34

    event_log_csv_path: str = "logs/events_log.csv"

    # Deadline model: deadline = clamp(alpha * dt, [deadline_min_ms, deadline_max_ms]).
    # deadline_min_ms = sudden-obstacle reaction window = (1.2+0.7+0.3)/15.0 * 1000 ~= 147ms.
    deadline_alpha: float = 0.85
    deadline_min_ms: float = 147.0
    deadline_max_ms: float = 3500.0

    # =======================
    # Physics (feeds DronePhysics)
    # =======================
    cmd_latency_s: float = 0.10
    wind_level_0to1: float = 0.5
    wind_accel_std_base_mps2: float = 0.8
    physics_seed: int = 42  # seed for wind RNG; same seed -> identical wind gusts across runs

    mass_kg: float = 65.0
    zeta_lin: float = 0.9
    wn_lin_rad: float = 6.0
    zeta_yaw: float = 0.9
    wn_yaw_rad: float = 5.0
    jerk_max_lin_mps3: float = 20.0
    jerk_max_yaw_rps3: float = 40.0
    drag_lin_per_s: float = 0.20
    drag_quad_per_m: float = 0.04
    max_tilt_deg: float = 30.0
    a_z_up_max_mps2: float = 4.0
    a_z_down_max_mps2: float = 5.0
    vz_up_max_mps: float = 5.0
    vz_down_max_mps: float = 3.0
    v_horiz_max_mps: float = 15.0
    v_horiz_abs_max_mps: float = 20.0
    max_ang_speed_rps: float = 1.2
    yaw_acc_max_rps2: float = 6.0
    wind_tau_s: float = 1.5
