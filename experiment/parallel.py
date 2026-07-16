#!/usr/bin/env python3
"""Multiprocessing fan-out for experiment.orchestrator.run_batch.

Each attempt is independent (seeded purely from attempt_idx -- see
orchestrator._attempt_seed), so attempts parallelize cleanly across worker
processes. PyBullet state isn't shareable across processes anyway (each
p.connect(p.DIRECT) client is process-local), so "one worker = one process =
one PyBullet client" is the natural boundary -- no in-process threading here.

Worker w walks the interleaved attempt-index stream w+1, w+1+n, w+1+2n, ...
so the union of seeds explored across all workers matches what a single
sequential run over the same attempt indices would have used
(_attempt_seed() cycles mod 100 regardless), just executed out of order.

Per-attempt side-effect files (nofly_meta_path, target_json_path,
event_log_csv_path) are opened in truncate ("w") mode and read back
within the same attempt -- see sim/arena.py's write_nofly_meta/
write_target_meta and sim/event_source.py's EventEmitter. Sharing those
paths across worker processes would race (one worker's write clobbered by
another's before it's read back), so every worker gets its own copies of
all path fields, merged back into the caller's single-process paths once
every worker finishes.
"""
import csv
import dataclasses
import multiprocessing
import os
import tempfile
from typing import List, Tuple

from config import TeleopConfig
from experiment.results_io import CSV_FIELDNAMES

# TeleopConfig fields that name a file a worker process writes to (and, for
# the nofly/target meta files, reads back within the same attempt) -- these
# must be distinct per worker, never the caller's shared path.
_WORKER_PATH_FIELDS = [
    "results_csv_path", "log_path", "event_log_csv_path",
    "nofly_meta_path", "target_json_path",
]


def _worker_cfg(cfg: TeleopConfig, worker_id: int, tmp_dir: str) -> TeleopConfig:
    overrides = {}
    for field in _WORKER_PATH_FIELDS:
        base = getattr(cfg, field)
        root, ext = os.path.splitext(os.path.basename(base))
        overrides[field] = os.path.join(tmp_dir, f"{root}_w{worker_id}{ext}")
    return dataclasses.replace(cfg, **overrides)


def _worker_entry(cfg: TeleopConfig, strategies, quota: int, worker_id: int,
                   n_workers: int) -> Tuple[int, int]:
    # Imported inside the worker process: the "spawn" start method gives
    # each worker a fresh interpreter, so this is the first time this
    # process touches pybullet/orchestrator -- nothing inherited from the
    # parent (which never calls p.connect() itself in the workers>1 path).
    from experiment.orchestrator import run_batch
    return run_batch(
        cfg, gui=False, strategies=strategies, quota=quota,
        attempt_start=worker_id + 1, attempt_step=n_workers,
        label=f"[w{worker_id}] ",
    )


def _split_quota(total: int, n_workers: int) -> List[int]:
    base, rem = divmod(total, n_workers)
    return [base + (1 if i < rem else 0) for i in range(n_workers)]


def run_parallel(cfg: TeleopConfig, strategies, n_workers: int) -> Tuple[int, int]:
    """Fans cfg.simulation_runs good runs out across n_workers processes,
    then merges their per-worker results CSVs/logs back into cfg's own
    results_csv_path/log_path. Returns (good_runs, attempts_tried), summed
    across all workers."""
    quotas = _split_quota(cfg.simulation_runs, n_workers)
    tmp_dir = tempfile.mkdtemp(prefix="parallel_run_")
    worker_cfgs = [_worker_cfg(cfg, w, tmp_dir) for w in range(n_workers)]

    ctx = multiprocessing.get_context("spawn")
    with ctx.Pool(n_workers) as pool:
        async_results = [
            pool.apply_async(_worker_entry, (worker_cfgs[w], strategies, quotas[w], w, n_workers))
            for w in range(n_workers) if quotas[w] > 0
        ]
        results = [r.get() for r in async_results]

    good_runs_total = sum(g for g, _ in results)
    attempts_total = sum(a for _, a in results)

    active_cfgs = [wc for wc, q in zip(worker_cfgs, quotas) if q > 0]
    _merge_csvs([wc.results_csv_path for wc in active_cfgs], cfg.results_csv_path)
    _merge_logs([wc.log_path for wc in active_cfgs], cfg.log_path)

    return good_runs_total, attempts_total


def _merge_csvs(worker_csv_paths: List[str], out_path: str) -> None:
    """Concatenates per-worker results CSVs into one file, renumbering the
    `run` column sequentially (each worker numbered its own good runs
    starting from 1, so they collide without this)."""
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    run_idx = 0
    with open(out_path, "w", newline="") as out_f:
        writer = csv.DictWriter(out_f, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()
        for path in worker_csv_paths:
            if not os.path.exists(path):
                continue
            with open(path, newline="") as in_f:
                reader = csv.DictReader(in_f)
                rows_by_run = {}
                for row in reader:
                    rows_by_run.setdefault(row["run"], []).append(row)
                for run_key in sorted(rows_by_run, key=int):
                    run_idx += 1
                    for row in rows_by_run[run_key]:
                        row["run"] = run_idx
                        writer.writerow(row)


def _merge_logs(worker_log_paths: List[str], out_path: str) -> None:
    """Concatenates per-worker JSON-lines logs -- these back an offline
    verification path (analysis/log_transformer.py), not the CSV results
    used for analysis, so simple concatenation (not chronological
    interleaving across workers) is sufficient."""
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w") as out_f:
        for path in worker_log_paths:
            if not os.path.exists(path):
                continue
            with open(path) as in_f:
                out_f.write(in_f.read())
