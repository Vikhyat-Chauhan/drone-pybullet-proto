# config.py
from dataclasses import dataclass, field
import os

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))


@dataclass
class TeleopConfig:
    # --- Run Options ---
    simulation_runs: int = 1
    simulation_timeout: float = 280
    simulation_world_style: str = "city"  # "perlin|city"

    # --- World gen ---
    fixed_seed: bool = False
    world_gen_seed_offset: int = 1234
    # Matches CANavigator's original exactly (arena.ArenaCfg's domain is
    # back to -100..100 x -50..50, same as the Gazebo world).
    target_distance: float = 50

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
    # Search-and-rescue detection
    # =======================
    # Omnidirectional signal (RSSI/thermal-style) detection radius -- the
    # target is unknown to nav until the drone comes within this range.
    # Widened from the original 15.0: APE1's search mode has no memory/
    # coverage bias by design (see ape1_bug.c), so on a large arena it can
    # wander past a 15m radius indefinitely without ever crossing it --
    # unrelated to threat-handling, but it starves the multi-strategy
    # pipeline of completed runs if every tier can't at least eventually
    # stumble onto the target within simulation_timeout.
    target_detect_radius_m: float = 28.0

    # =======================
    # Event Generator (B-style)
    # =======================
    event_seed: int = 42
    event_deterministic: bool = True

    # Inter-arrival gap for new threats (threats.py::ThreatManager),
    # drawn log-uniformly from [event_dt_min_s, event_dt_max_s]. Sized to
    # comfortably exceed every APE's gem5-measured compute budget (up to
    # ~150ms) plus the commit-hold window (commit_hold_s), so a new
    # threat's own deadline race is what determines success/failure --
    # not the previous threat's race getting preempted before it can
    # resolve. (A much smaller value was inherited from the old
    # purely-timing event model, which had no concept of a spatial
    # sensor-detection-triggered race and could tolerate a much faster,
    # non-physical cadence.)
    event_dt_min_s: float = 0.6
    event_dt_max_s: float = 2.5

    event_mix_enemy: float = 0.33
    event_mix_obstacle: float = 0.33
    event_mix_lane: float = 0.34

    event_log_csv_path: str = "logs/events_log.csv"

    # Deadline model (threats.py::ThreatManager.on_sensed): deadline_s =
    # clamp(sensed_range_m / sensed_closing_speed_mps, [kind's min, kind's
    # max]), computed at the moment a threat first enters the proxy
    # sensor's range/cone (threat_sensor_range_m), not at spawn -- so the
    # raw value is still a genuine reaction-time budget, not an arbitrary
    # distance. The clamp bounds are per-kind (not one shared pair):
    # with a single sensor cone/range for every kind, most encounters are
    # geometrically tangential regardless of the threat's own speed, so a
    # shared clamp washed out the kind signal entirely in practice (see
    # threats.py::ThreatCfg's comment). Calibrated against the three APE
    # compute budgets (roughly 7ms/73ms/152ms on the default gem5
    # profile, DEADLINE_SCALE=1000 -- see mcu_cycle_model.py) so the
    # fast/medium/slow tradeoff actually shows up every run: ENEMY's
    # window sits entirely below APE3's budget (APE3 always too slow for
    # a fast incoming threat), straddles APE2's (a coin flip), well above
    # APE1's (always makeable); SUDDEN_OBSTACLE straddles APE3's budget
    # (sometimes makeable); LANE_BLOCK's window sits above all three
    # (always makeable, even by APE3).
    deadline_alpha: float = 0.85
    enemy_deadline_min_s: float = 0.05
    enemy_deadline_max_s: float = 0.15
    obstacle_deadline_min_s: float = 0.15
    obstacle_deadline_max_s: float = 0.40
    lane_deadline_min_s: float = 0.40
    lane_deadline_max_s: float = 0.65

    # =======================
    # Moving threats (threats.py::ThreatManager) + proxy sensor
    # =======================
    # Forward "camera" proxy sensor -- threats.py's Threat objects have no
    # spatial existence to the navigator until they fall within this
    # range/cone (sim_adapters.ThreatSensorProvider).
    threat_sensor_range_m: float = 12.0
    threat_sensor_fov_deg: float = 180.0
    max_active_threats: int = 3

    threat_spawn_range_min_m: float = 10.0
    threat_spawn_range_max_m: float = 18.0
    threat_lead_time_s: float = 1.5
    crash_margin_m: float = 0.15

    threat_retire_margin_m: float = 5.0
    threat_max_lifetime_s: float = 20.0
    # See threats.py::ThreatCfg.rearm_cooldown_s -- a tradeoff between
    # staying below the collision buffer at enemy_speed_max_mps (wants
    # this small) and staying well above the nav tick period (wants this
    # large, or concurrent re-arms start spuriously preempting each other
    # within the same tick regardless of APE speed).
    threat_rearm_cooldown_s: float = 0.15

    # Per-kind kinematics. lead_bias in [0,1]: 0 = fully random heading
    # (guaranteed misses possible), 1 = full intercept-solution aim at the
    # drone's dead-reckoned future position.
    threat_enemy_speed_min_mps: float = 8.0
    threat_enemy_speed_max_mps: float = 16.0
    threat_enemy_radius_m: float = 0.6
    threat_enemy_lead_bias: float = 0.85

    threat_obstacle_speed_min_mps: float = 3.0
    threat_obstacle_speed_max_mps: float = 7.0
    threat_obstacle_radius_min_m: float = 1.2
    threat_obstacle_radius_max_m: float = 2.5
    threat_obstacle_lead_bias: float = 0.4

    threat_lane_speed_min_mps: float = 0.0
    threat_lane_speed_max_mps: float = 1.0
    threat_lane_radius_min_m: float = 2.0
    threat_lane_radius_max_m: float = 3.5
    threat_lane_lead_bias: float = 0.0

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
