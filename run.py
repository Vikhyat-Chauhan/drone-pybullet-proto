#!/usr/bin/env python3
"""Hydra-driven entrypoint -- the sole way to run an experiment in this repo.

This file is the ONLY place Hydra/OmegaConf gets touched: it composes the
YAML config tree under conf/ (config groups: run/sim/events/physics -- see
conf/config.yaml and docs/CONFIGURATION.md), converts it into a real
config.TeleopConfig instance, and hands it to experiment.orchestrator.main().
Downstream consumers keep working against a plain, freely-mutable
dataclass, never a Hydra-wrapped one.

Usage:
    python run.py                                   # defaults (headless batch mode)
    python run.py gui=true                           # interactive PyBullet demo window
    python run.py workers=8                           # fan attempts out across 8 processes
    python run.py events=dense physics=gusty          # swap config-group presets
    python run.py rate_hz=200 wind_level_0to1=0.8      # ad-hoc field overrides
    python run.py -m world_gen_seed_offset=7778,7779,7780,7781,7782
                                                       # multirun sweep, one isolated
                                                       # job dir per seed (see
                                                       # conf/config.yaml's hydra.job.chdir)

See docs/CONFIGURATION.md for the full field reference, preset list, and
override/sweep syntax.
"""
from __future__ import annotations

import dataclasses
import logging

import hydra
from omegaconf import DictConfig, OmegaConf

from config import TeleopConfig
from experiment.orchestrator import main as run_experiment

log = logging.getLogger(__name__)

_TELEOP_FIELDS = {f.name for f in dataclasses.fields(TeleopConfig)}


@hydra.main(version_base=None, config_path="conf", config_name="config")
def run(cfg_hydra: DictConfig) -> None:
    plain = OmegaConf.to_container(cfg_hydra, resolve=True)

    # `gui`/`workers` are experiment.orchestrator.main() arguments, not
    # TeleopConfig fields -- pulled out separately rather than filtered into
    # tc_kwargs below.
    gui = bool(plain.get("gui", False))
    workers = int(plain.get("workers", 1))

    # Only pass through keys that are actual TeleopConfig fields -- anything
    # else in the composed config (gui, Hydra's own bookkeeping) is ignored
    # here rather than raising, so conf/config.yaml can carry non-TeleopConfig
    # settings without this call site needing to know about each one.
    tc_kwargs = {k: v for k, v in plain.items() if k in _TELEOP_FIELDS}
    cfg = TeleopConfig(**tc_kwargs)

    log.info("Resolved TeleopConfig: %s", cfg)
    run_experiment(gui=gui, cfg=cfg, workers=workers)


if __name__ == "__main__":
    run()
