# Configuration (Hydra)

Every field of `config.py::TeleopConfig` is composed from YAML config groups
under `conf/` and can be overridden from the command line, without editing any
Python. `run.py` (repo root) is the entrypoint that does this composition; it
converts the fully-resolved config into a real `TeleopConfig` instance and
hands it to `experiment.orchestrator.main()`.

Arena geometry (`sim/arena.py`'s `ArenaCfg`/`PerlinArenaCfg` -- block/lot
sizing, road width, Perlin density, etc.) is **not** exposed here; see "Known
limitation" at the bottom.

## TeleopConfig field reference

Grouped by the four Hydra config groups (`conf/<group>/default.yaml`). Values
below are each group's shipped default -- identical to `config.py`'s literal
defaults, so `python run.py` with no overrides reproduces today's behavior
exactly.

### `run` (`conf/run/`) -- run options, world gen, logging, results/analyzer

| Field | Default | Meaning |
|---|---|---|
| `simulation_runs` | `1` | Number of *good* runs (all strategies reached target) to collect before stopping. |
| `simulation_timeout` | `200` | Per-strategy wall-sim-time cap (s) before a run is abandoned as failed. |
| `simulation_world_style` | `city` | `city` (grid/blocks) or `perlin` (noise-field) arena generator. |
| `fixed_seed` | `false` | If `true`, every attempt reuses `world_gen_seed_offset` as-is instead of incrementing per attempt. |
| `world_gen_seed_offset` | `7777` | Base seed for arena/target generation; see `experiment/orchestrator.py::_attempt_seed()`. |
| `target_distance` | `150` | Minimum start→target distance (m) the target picker enforces. |
| `log_path` | `logs/run_logs.json` | Async JSON event log path (`applog/async_logger.py`). |
| `results_csv_path` | `logs/results/experiment_summary.csv` | Per-strategy-per-run CSV `experiment/results_io.py` appends to. |
| `analyzer_out_dir` | `logs/results` | Output folder for `analysis/statistics_analyzer.py`'s summary CSVs. |
| `analyzer_strategies` | `[APE1, APE2, APE3, CA]` | Which strategies to run per attempt, in order. |
| `start_x/y/z/yaw` | `-95.0 / 0.0 / 1.0 / 0.0` | Fixed drone spawn pose. |
| `target_json_path` | `models/generated/generated_target_meta.json` | Where the picked target is recorded. |
| `nofly_meta_path` | `models/generated/generated_nofly_meta.json` | Where NFZ rectangles are recorded (feeds `sim/violations.py`). |

### `sim` (`conf/sim/`) -- PyBullet sim + nav/teleop rate

| Field | Default | Meaning |
|---|---|---|
| `flight_z` | `3.0` | Cruise altitude (m). |
| `drone_radius_m` | `0.7` | Collision sphere radius. |
| `entity_name` | `drone1` | Cosmetic body name. |
| `rate_hz` | `100` | Nav decision-loop rate (`nav/algorithm.py::tick()` cadence). |

### `events` (`conf/events/`) -- event generator (B-style)

| Field | Default | Meaning |
|---|---|---|
| `event_seed` | `42` | RNG seed for event timing/kind/deadline draws. |
| `event_deterministic` | `true` | Pose-independent logical-clock event timing (the only mode `experiment/orchestrator.py` uses). |
| `event_dt_min_ms` / `event_dt_max_ms` | `20.0` / `4000.0` | Log-uniform inter-event-arrival window (ms). Config-facing unit; converted to seconds at the `EventCfg` boundary in `experiment/orchestrator.py` since the sim engine (`SimClock`, `sim/event_source.py`, `nav/algorithm.py`) runs in seconds throughout. |
| `event_mix_enemy/obstacle/lane` | `0.33 / 0.33 / 0.34` | Event-kind mixture weights. |
| `event_log_csv_path` | `logs/events_log.csv` | Raw per-event CSV log. |
| `deadline_alpha` | `0.85` | `deadline = clamp(alpha * dt, [deadline_min_ms, deadline_max_ms])` (dt in ms). |
| `deadline_min_ms` / `deadline_max_ms` | `147.0` / `3500.0` | Deadline clamp bounds (ms). |

### `physics` (`conf/physics/`) -- feeds `DronePhysics`

| Field | Default | Meaning |
|---|---|---|
| `cmd_latency_s` | `0.10` | Command-to-actuation delay. |
| `wind_level_0to1` | `0.5` | Wind gust intensity scale. |
| `wind_accel_std_base_mps2` | `0.8` | Base OU gust accel std-dev. |
| `wind_tau_s` | `1.5` | OU correlation time -- shorter = faster-varying gusts. |
| `physics_seed` | `42` | Wind RNG seed (same seed -> identical gusts run-to-run). |
| `mass_kg` | `65.0` | Vehicle mass. |
| `zeta_lin` / `wn_lin_rad` | `0.9` / `6.0` | Linear 2nd-order actuator damping/natural frequency. |
| `zeta_yaw` / `wn_yaw_rad` | `0.9` / `5.0` | Yaw 2nd-order actuator damping/natural frequency. |
| `jerk_max_lin_mps3` / `jerk_max_yaw_rps3` | `20.0` / `40.0` | Jerk (accel slew-rate) caps. |
| `drag_lin_per_s` / `drag_quad_per_m` | `0.20` / `0.04` | Linear/quadratic aerodynamic drag coefficients. |
| `max_tilt_deg` | `30.0` | Tilt cap -> lateral accel cap. |
| `a_z_up_max_mps2` / `a_z_down_max_mps2` | `4.0` / `5.0` | Vertical accel caps (asymmetric). |
| `vz_up_max_mps` / `vz_down_max_mps` | `5.0` / `3.0` | Ascent/descent rate caps. |
| `v_horiz_max_mps` / `v_horiz_abs_max_mps` | `15.0` / `20.0` | Horizontal speed cap (policy / absolute ceiling). |
| `max_ang_speed_rps` | `1.2` | Yaw rate cap. |
| `yaw_acc_max_rps2` | `6.0` | Yaw angular-acceleration cap. |

## Preset variants

Each group's `default.yaml` is the literal current behavior; these presets
demonstrate swapping a group wholesale. Add more by dropping a new
`conf/<group>/<name>.yaml` file (see any existing preset's comment for the
"extends default" pattern to copy).

| Preset | Group | What it changes |
|---|---|---|
| `run=smoke` | `run` | 1 short-hop run (`target_distance: 25`), 60s timeout -- fast pipeline sanity check, used by `make smoke` / CI. |
| `run=full` | `run` | 50 good runs -- a long benchmark sweep. |
| `sim=high_rate` | `sim` | `rate_hz: 200` -- finer nav control loop. |
| `sim=low_rate` | `sim` | `rate_hz: 50` -- coarser, cheaper control loop. |
| `events=sparse` | `events` | Wider inter-arrival window -- fewer threat events. |
| `events=dense` | `events` | Narrower inter-arrival window + lower `deadline_alpha` -- frequent events, tight deadlines. |
| `physics=calm` | `physics` | Low wind intensity/gust magnitude. |
| `physics=gusty` | `physics` | High wind intensity/gust magnitude, shorter gust correlation time. |

## CLI override syntax

```bash
# Single-field overrides (any TeleopConfig field name works directly):
python run.py rate_hz=200 wind_level_0to1=0.8

# Swap a whole config-group preset:
python run.py events=dense physics=gusty

# Combine group presets with individual overrides (individual overrides win):
python run.py physics=gusty wind_level_0to1=0.6

# Interactive PyBullet GUI demo instead of the headless default:
python run.py gui=true
```

## Multirun (sweep) syntax

Hydra's `-m`/`--multirun` flag runs one isolated job per value (or per
combination, for multiple swept fields):

```bash
# Reproduce a 5-seed experiment declaratively (no manual loop):
python run.py -m run=smoke world_gen_seed_offset=7778,7779,7780,7781,7782

# Cross product of presets -- 4 jobs (sparse+calm, sparse+gusty, dense+calm, dense+gusty):
python run.py -m events=sparse,dense physics=calm,gusty
```

Each job gets its own working directory (`conf/config.yaml` sets
`hydra.job.chdir: true`), under `outputs/<date>/<time>/` for a single run or
`multirun/<date>/<time>/<job-num>/` for a sweep -- so every job's
`logs/results/experiment_summary.csv`, `logs/run_logs.json`, and generated
arena/target JSON land isolated per job automatically, with no code changes
and no collisions between concurrent/sequential jobs. The procedurally
generated ground/window textures (`viz/world_builder.py`'s
`_ensure_ground_texture`/`_ensure_window_texture`) are the one thing that
*isn't* per-job -- they're cached at fixed, absolute, `__file__`-derived
paths, so every job shares the same cached textures regardless of its own
working directory.

## Known limitation

`sim/arena.py`'s `ArenaCfg`/`PerlinArenaCfg` geometry knobs (`major_m`,
`minor_m`, `road_w_m`, `lot_w_m`, `fill_prob`, `density`, `corr_len_m`,
`height_min/max`, ...) are hardcoded in `experiment/orchestrator.py`'s
`run_attempt()`, which only ever passes `seed=` through. They are
intentionally **not** exposed as Hydra fields -- tuning arena shape/density
still requires editing `sim/arena.py`/`experiment/orchestrator.py` directly.
