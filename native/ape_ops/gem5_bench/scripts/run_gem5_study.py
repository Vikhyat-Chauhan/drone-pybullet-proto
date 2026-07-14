#!/usr/bin/env python3
"""
run_gem5_study.py — runs one bench_apeN binary under gem5 SE mode with a
named CPU study from ../configs/ (e.g. cortex_m4_168mhz.py,
cortex_m7_400mhz.py), via gem5's own ARM Research Starter Kit example
config (configs/example/arm/starter_se.py) — reused as-is rather than
re-implementing MinorCPU SimObject wiring from scratch.

Studies are discovered dynamically from ../configs/*.py (any module
exposing CPU_PRESET/CPU_FREQ/MEM_TYPE) — add a new profile file there to
model a different CPU/clock; no changes needed here or in
freeze_measured_latencies.py.

Env vars required:
    GEM5_ROOT   — path to a gem5 checkout with build/ARM/gem5.opt built
                  (see ../../../../.gem5-build/gem5 in this repo, after
                  `scons build/ARM/gem5.opt`)
"""
from __future__ import annotations

import argparse
import importlib
import os
import subprocess
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_CONFIGS_DIR = _HERE.parent / "configs"


def available_profiles() -> list[str]:
    """Names of all CPU study profiles in ../configs/ (module stems, sorted)."""
    return sorted(
        p.stem for p in _CONFIGS_DIR.glob("*.py")
        if not p.stem.startswith("__")
    )


def run_study(study_name: str, binary_path: Path, outdir: Path) -> Path:
    """Runs `binary_path` under gem5 SE mode per `study_name`'s params.
    Returns the path to the produced stats.txt."""
    gem5_root = os.environ.get("GEM5_ROOT")
    if not gem5_root:
        raise RuntimeError("GEM5_ROOT must be set to a gem5 checkout with build/ARM/gem5.opt")
    gem5_bin = Path(gem5_root) / "build" / "ARM" / "gem5.opt"
    if not gem5_bin.exists():
        raise FileNotFoundError(f"{gem5_bin} not found — build it with: cd {gem5_root} && scons build/ARM/gem5.opt")

    starter_se = Path(gem5_root) / "configs" / "example" / "arm" / "starter_se.py"
    if not starter_se.exists():
        raise FileNotFoundError(f"{starter_se} not found in this gem5 checkout")

    sys.path.insert(0, str(_CONFIGS_DIR))
    study = importlib.import_module(study_name)

    outdir.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(gem5_bin),
        f"--outdir={outdir}",
        str(starter_se),
        "--cpu", study.CPU_PRESET,
        "--cpu-freq", study.CPU_FREQ,
        "--mem-type", study.MEM_TYPE,
        str(binary_path),
    ]
    print("+", " ".join(cmd))
    subprocess.run(cmd, check=True)

    stats_path = outdir / "stats.txt"
    if not stats_path.exists():
        raise FileNotFoundError(f"gem5 did not produce {stats_path}")
    return stats_path


def parse_num_cycles(stats_path: Path) -> int:
    """Extracts the CPU's numCycles stat from a gem5 stats.txt, ROI-only.

    bench/src/main.c brackets only the measured loop with
    m5_reset_stats()/m5_dump_stats(), so gem5 writes one
    "Begin/End Simulation Statistics" block for the ROI, immediately
    followed by a second block for the (unwanted) tail up to process
    exit — this takes the FIRST numCycles occurrence, i.e. the ROI one.

    starter_se.py names the stat `system.cpu_cluster.cpus.numCycles`
    (single-core cluster) rather than `system.cpu.numCycles` — matched
    by suffix so this isn't sensitive to which CPU-cluster naming a
    given starter_se.py preset uses.
    """
    with stats_path.open() as f:
        for line in f:
            key = line.split(maxsplit=1)[0] if line.strip() else ""
            if key.endswith(".numCycles"):
                return int(line.split()[1])
    raise ValueError(f"*.numCycles not found in {stats_path}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("study", choices=available_profiles())
    ap.add_argument("binary", type=Path)
    ap.add_argument("outdir", type=Path)
    args = ap.parse_args()

    stats_path = run_study(args.study, args.binary, args.outdir)
    cycles = parse_num_cycles(stats_path)
    print(f"numCycles = {cycles}")


if __name__ == "__main__":
    main()
