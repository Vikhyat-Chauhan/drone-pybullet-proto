# drone-pybullet-proto

A PyBullet port of "CANavigator" -- a drone nav-strategy benchmark comparing
four navigation approaches (APE1 reactive/bug, APE2 Dynamic Window Approach,
APE3 Vector Field Histogram, and CA, a selector that races all three and
takes whichever finishes in time) against a procedurally generated no-fly-zone
city/perlin arena. Each run is scored on time-to-target, no-fly-zone
violations, propulsion energy, and compute latency/energy modeled from
gem5-measured native-planner timing (see [docs/POWER_MODEL.md](docs/POWER_MODEL.md)).

## Quickstart

```bash
make setup   # create venv, install requirements.txt
make build   # build native/ape_ops/build/libape_ops.so (the real APE1/APE2/APE3 planners)
make run     # one headless run with default config, writes logs/results/*.csv
```

Or without Make:

```bash
python3 -m venv venv
./venv/bin/pip install -r requirements.txt
make -C native/ape_ops native
./venv/bin/python run.py
```

## Running the simulator

**`python run.py [overrides...]`** -- the sole entrypoint (see
[Configuration](#configuration-hydra) below). Defaults to headless batch mode
(`p.DIRECT`, no sleep pacing, `cfg.simulation_runs` good runs);
`python run.py gui=true` gives the interactive PyBullet GUI demo (real-time
paced, chase camera, live HUD overlay).

An experiment discards a whole attempt (regenerates a fresh arena + target
and tries again) if any strategy fails to reach the target within
`simulation_timeout`, and only counts a "good run" once every strategy in
`analyzer_strategies` succeeds. Results land in `logs/results/experiment_summary.csv`
(one row per strategy per good run) plus a per-strategy summary CSV from
`analysis/statistics_analyzer.py`, run automatically at the end of
`experiment.orchestrator.main()`.

### Interactive config picker

```bash
make configure
```

A small curses-based terminal UI (`scripts/tui.py`) for cycling through the
`run`/`sim`/`events`/`physics` presets and toggling `gui` with the arrow keys
(or `j`/`k`/`h`/`l`), previewing the resulting `run.py ...` command live, then
launching it on Enter. No extra dependency -- built on the stdlib `curses`
module. `q`/Esc cancels without running anything.

## Configuration (Hydra)

Every field of `config.py::TeleopConfig` is composed from YAML config groups
under `conf/` (`run`, `sim`, `events`, `physics`) and overridable from the
command line -- no Python edits needed for a parameter sweep. `run.py` is the
only file that touches Hydra; it converts the composed config into a plain
`TeleopConfig` instance and hands it to `experiment.orchestrator.main()`, so
every consumer downstream (`nav/algorithm.py`, `sim/physics.py`,
`sim/teleop.py`, ...) is unaffected.

```bash
python run.py                              # defaults (headless batch mode)
python run.py events=dense physics=gusty   # swap config-group presets
python run.py rate_hz=200 mass_kg=70       # ad-hoc field overrides
```

See **[docs/CONFIGURATION.md](docs/CONFIGURATION.md)** for the full field
reference, the preset list, and override syntax.

## Multirun sweeps

```bash
# The "5 seeds" experiment, declaratively -- one isolated job dir per seed:
python run.py -m run=smoke world_gen_seed_offset=7778,7779,7780,7781,7782

# Cross a couple of presets:
python run.py -m events=sparse,dense physics=calm,gusty
```

Each job's results land isolated under its own `outputs/<date>/<time>/` or
`multirun/<date>/<time>/<job-num>/` directory -- see
[docs/CONFIGURATION.md](docs/CONFIGURATION.md#multirun-sweep-syntax) for why
that's safe and how it interacts with the generated-texture cache.

## Makefile targets

| Target | Does |
|---|---|
| `make setup` | Create `venv/`, install `requirements.txt` (`scripts/bootstrap.sh`). |
| `make build` | Build `native/ape_ops/build/libape_ops.so`. |
| `make run` | `./venv/bin/python run.py $(ARGS)` -- pass overrides via `ARGS="..."`. |
| `make configure` | Interactive curses terminal UI to pick presets and launch `run.py`. |
| `make sweep` | `./venv/bin/python run.py -m $(ARGS)` -- multirun. |
| `make analyze` | Re-run `analysis/statistics_analyzer.py` against the repo-root `logs/results/experiment_summary.csv`, without re-running the sim. `run.py`/Hydra runs already get this called automatically, against their own isolated job-dir CSV. |
| `make smoke` | Fast single-run sanity check (`run=smoke` preset) -- what CI runs. |
| `make clean` | Remove `venv/`, native build artifacts, Hydra `outputs/`/`multirun/`, `__pycache__`. |
| `make ci` | `setup build smoke` chained -- reproduce a CI run locally. |
| `make switch-processor PROFILE=<name>` | Switch the simulated flight-controller CPU (gem5 profile) and regenerate `nav/gem5_measured_latencies.py` -- see `docs/POWER_MODEL.md` §3. Requires `GEM5_ROOT`. |

## CI

`.github/workflows/ci.yml` runs on every push/PR: bootstrap (venv + deps +
native build, cached), a `timeout`-wrapped headless `run=smoke` run, then
uploads `experiment_summary.csv` and `strategy_summary_zone-mean.csv` as a
build artifact. It reuses `scripts/bootstrap.sh`, so local and CI setup never
drift apart.

## Repo layout

`config.py` (shared `TeleopConfig` dataclass) and `run.py` (Hydra entrypoint)
sit at the repo root; everything else is grouped into packages by layer:

- `sim/` -- the physical/domain simulation substrate, no navigation logic or
  rendering: `arena.py` (procedural NFZ arena generation), `physics.py`
  (drone dynamics), `teleop.py`/`velocity_sink.py` (command-application
  layer), `sim_adapters.py`/`lidar.py` (PyBullet-backed sensor/pose
  plumbing), `event_source.py` (synthetic event generator), `violations.py`/
  `energy_monitor.py` (NFZ/energy accounting).
- `nav/` -- the navigation decision layer: `algorithm.py` (APE1/2/3 + CA
  strategy dispatch, event racing, NFZ repulsion), `ape_native.py` (ctypes
  bridge to the real C planners in `native/ape_ops/`), `mcu_cycle_model.py`/
  `gem5_measured_latencies.py` (compute latency/energy modeling).
- `viz/` -- pure presentation, never touches simulation state:
  `world_builder.py` (PyBullet arena/drone/target/texture construction),
  `hud_text.py` (HUD metrics formatting), `hud_overlay.py` (the OS-level
  overlay window).
- `experiment/` -- orchestration: `orchestrator.py` (the per-strategy drive
  loop, per-attempt loop, and `main()` that `run.py` calls into),
  `results_io.py` (results CSV I/O).
- `native/ape_ops/` -- the real C planners (APE1/APE2/APE3), called from
  Python via `nav/ape_native.py`'s ctypes bridge; `gem5_bench/` is the
  offline gem5 cycle-measurement harness feeding `nav/gem5_measured_latencies.py`.
- `analysis/` -- post-hoc CSV/JSON log analysis (`statistics_analyzer.py`,
  `log_transformer.py`, `power_estimate.py`).
- `conf/` -- Hydra config groups (`run`, `sim`, `events`, `physics`); see
  [docs/CONFIGURATION.md](docs/CONFIGURATION.md).
- `scripts/verify/` -- ad hoc, standalone manual verification scripts
  (`verify_step2_physics.py` through `verify_step4_single_ape1.py`), each
  exercising progressively more of the stack. Not pytest tests, not wired
  into CI.
- `docs/POWER_MODEL.md` -- the compute/propulsion energy model and gem5
  measurement methodology.
