# Automated setup/build/run/analyze pipeline for drone-pybullet-proto.
# See README.md for a walkthrough and docs/CONFIGURATION.md for the Hydra
# config reference (conf/run|sim|events|physics groups, override/sweep
# syntax).
#
# Pass extra CLI args (Hydra overrides) via ARGS, e.g.:
#   make run ARGS="events=dense physics=gusty"
#   make sweep ARGS="world_gen_seed_offset=7778,7779,7780,7781,7782"

.PHONY: setup build run sweep analyze smoke clean ci configure switch-processor

setup:
	./scripts/bootstrap.sh

build:
	$(MAKE) -C native/ape_ops native

run:
	./venv/bin/python run.py $(ARGS)

# Interactive terminal UI (arrow keys) to pick config-group presets and
# launch run.py -- see scripts/tui.py.
configure:
	./venv/bin/python scripts/tui.py

sweep:
	./venv/bin/python run.py -m $(ARGS)

# Switches the simulated flight-controller CPU and regenerates
# nav/gem5_measured_latencies.py (latency + energy numbers) against it --
# see scripts/switch_processor.py. Requires GEM5_ROOT.
#   make switch-processor PROFILE=cortex_m7_400mhz
#   make switch-processor ARGS=--list
switch-processor:
	./venv/bin/python scripts/switch_processor.py $(if $(PROFILE),--profile $(PROFILE)) $(ARGS)

# Re-analyzes the repo-root logs/results/experiment_summary.csv.
# experiment.orchestrator.main() already calls this automatically at the end
# of every run (whose CSV instead lands inside that job's own
# outputs/<date>/<time>/ or multirun/.../<job-num>/ directory thanks to
# hydra.job.chdir) -- this target exists for re-analyzing without re-running
# the sim, e.g. after tweaking zone_metric.
analyze:
	./venv/bin/python -c "from analysis.statistics_analyzer import run_analysis; run_analysis(zone_metric='mean')"

smoke:
	./venv/bin/python run.py run=smoke

clean:
	rm -rf venv native/ape_ops/build outputs multirun
	find . -name __pycache__ -exec rm -rf {} +

# The exact sequence CI runs -- reproduce a CI failure locally with `make ci`.
ci: setup build smoke
