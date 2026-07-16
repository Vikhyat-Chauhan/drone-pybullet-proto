# Parallelization

`experiment.orchestrator.main()` can fan the experiment's attempt loop out
across multiple OS processes instead of running attempts one after another.
This document covers the architecture, why it's shaped the way it is, and
measured speedup.

## Why processes, not threads

The physics/rendering backend (PyBullet) holds its simulation state inside a
C extension tied to one `physicsClientId` (`p.connect(p.DIRECT)`), and that
state isn't shareable across threads. So the natural unit of parallelism is
**one worker = one OS process = one PyBullet client** — there wasn't really
a lighter-weight option.

## Why this is parallelizable at all

Every attempt (one generated arena + target, then all `analyzer_strategies`
run against it in sequence) is fully independent of every other attempt: its
only input is an integer, `attempt_idx`, via `_attempt_seed()`
(`experiment/orchestrator.py`):

```python
def _attempt_seed(cfg, attempt_idx):
    return cfg.world_gen_seed_offset + (0 if cfg.fixed_seed else ((attempt_idx - 1) % 100) + 1)
```

Nothing about attempt 5 depends on attempt 4 having run. That's what makes a
process-per-worker fan-out safe rather than just fast.

## Architecture

```
run.py (workers=N)
  -> experiment.orchestrator.main(workers=N)
       -> experiment.parallel.run_parallel(cfg, strategies, N)
            |
            |-- split cfg.simulation_runs into N quotas (divmod, remainder
            |   spread across the first few workers)
            |
            |-- for each worker w in 0..N-1:
            |     - clone cfg with 5 path fields redirected into a temp dir
            |       (see "Per-worker file isolation" below)
            |     - spawn a process (multiprocessing, "spawn" context)
            |     - that process calls experiment.orchestrator.run_batch(),
            |       walking attempt_idx = w+1, w+1+N, w+1+2N, ...
            |       until its quota of *good* runs is collected
            |
            |-- pool.apply_async(...).get() on all workers (blocks until
            |   every worker's quota is met)
            |
            |-- merge all workers' results CSVs into cfg.results_csv_path,
            |   renumbering the `run` column sequentially
            |-- concatenate all workers' JSON logs into cfg.log_path
            |
       -> run_analysis() once, on the merged CSV
```

### `run_batch()` — the reusable unit of work

`experiment/orchestrator.py::run_batch(cfg, gui, strategies, quota,
attempt_start, attempt_step, label)` is what used to be `main()`'s inline
loop, pulled out so both the single-process path (`workers=1`, `attempt_start=1,
attempt_step=1`) and every worker process (`workers=N`) can call the exact
same code. It owns one PyBullet client end to end (`p.connect` /
`p.disconnect`), retries discarded attempts (any strategy missing the
target) until its `quota` of good runs lands in its own results CSV, and
returns `(good_runs, attempts_tried)`.

### Interleaved attempt-index assignment

Worker `w` (of `N`) walks `attempt_idx = w+1, w+1+N, w+1+2N, ...` rather than
being handed a contiguous block. Since `_attempt_seed()` cycles the seed
`mod 100`, a contiguous split (worker 0 = attempts 1..125, worker 1 =
126..250, ...) risks two workers computing the *same* seed at the *same*
wall-clock time — wasted, duplicate work. The interleaved assignment means
the union of attempt indices explored across all workers is exactly the
sequence a single sequential run would have walked over the same total
attempt count, just executed out of order — so a parallel run's seed
coverage matches a sequential run's, not some other distribution.

### Per-worker file isolation

Three files are opened in truncate (`"w"`) mode and **read back within the
same attempt**:

| Field | Written by | Read back by |
|---|---|---|
| `nofly_meta_path` | `sim/arena.py::write_nofly_meta()` | `sim/violations.py::ViolationMonitor` (same attempt) |
| `target_json_path` | `sim/arena.py::write_target_meta()` | (diagnostic; also same-attempt) |
| `event_log_csv_path` | `sim/event_source.py::EventEmitter` (per strategy) | — |

If two worker processes shared these paths, one worker's write could be
clobbered by another's before the first reads it back — silently feeding a
`ViolationMonitor` the wrong attempt's no-fly zones. `experiment/parallel.py`
avoids this by giving every worker its own copies of all five path-bearing
`TeleopConfig` fields (the three above plus `results_csv_path` and
`log_path`), redirected into a temp directory
(`experiment/parallel.py::_worker_cfg()`). No path is ever shared between
two live processes.

### Why `spawn`, not `fork`

`multiprocessing.get_context("spawn")` gives every worker a fresh Python
interpreter that imports `pybullet`/`experiment.orchestrator` for the first
time inside that process. Linux's default `fork` would instead duplicate the
*parent* process's memory — including any already-initialized C-extension
state — into every child, which is a well-known source of hard-to-debug
crashes with C-extension-backed libraries. Since the parent process in the
`workers>1` path never touches PyBullet itself, `spawn`'s extra process-start
cost is the only downside, and it's negligible relative to one attempt's
runtime.

### Merge step

`experiment/parallel.py::_merge_csvs()` concatenates every worker's results
CSV but renumbers the `run` column sequentially — each worker numbers its
own good runs starting from 1, so worker 0's "run 1" and worker 1's "run 1"
would otherwise collide once merged. `_merge_logs()` just concatenates the
per-worker JSON-lines logs; that log is a redundant offline-verification
artifact (see `experiment/orchestrator.py`'s comment on
`analysis/log_transformer.py`), not the CSV results pipeline, so exact
chronological interleaving across workers doesn't matter.

## Usage

```
python run.py workers=8                    # headless, 8 worker processes
python run.py run=full workers=8            # 1000-run benchmark, 8 workers
make configure                              # interactive picker (workers row, 1/2/4/8/16/32)
```

`workers` is a `run.py`-level argument (like `gui`), not a `TeleopConfig`
field — see `conf/config.yaml`. `gui=true` and `workers>1` are mutually
exclusive (interactive mode is inherently single-process);
`experiment.orchestrator.main()` raises if both are set, and `scripts/tui.py`
keeps the two picker rows mutually exclusive so you can't build that command
in the first place.

## Expected speedup (analytical — no clean sequential-vs-parallel
measurement taken yet)

One data point exists: `run=smoke` (10 good runs, city arena,
`target_distance=150`) with `workers=8` completed in **354.4s wall time**,
no discarded attempts. A matched `workers=1` run over the same 10 runs was
started to get a direct measured comparison but was stopped before
completion, so there's no apples-to-apples measured speedup number here —
what follows is reasoning from the architecture, not a benchmark result.

**Why speedup should track worker count closely, but not exceed it:**
- Attempts are CPU-bound (physics stepping dominates wall time — see
  `docs/CONFIGURATION.md` / `experiment/orchestrator.py`'s `phys_dt=1/1000`
  comment) and fully independent, with no shared resource contention once
  per-worker file isolation is in place (see above) — nothing serializes
  workers against each other mid-run.
- Process-spawn and the final CSV/log-merge are one-time, few-second costs,
  negligible against any realistic `simulation_runs` wall time.
- The wall-clock critical path is set by whichever worker gets the largest
  quota. `_split_quota()` spreads the remainder across the first few
  workers, so speedup is close to linear (~`N`x for `N` workers) when
  `simulation_runs` divides evenly (e.g. `run=full`'s 1000 runs / 8 workers
  = 125 each, exactly), but falls short of `N`x when it doesn't — e.g. 10
  runs / 8 workers gives quotas `[2,2,1,1,1,1,1,1]`, so the critical path is
  set by the two 2-attempt workers, capping speedup at
  `simulation_runs / ceil(simulation_runs / workers) = 10/2 = 5x` for that
  particular smoke-scale run, not 8x.
- The machine has 32 CPU cores, well above the 8 workers used in the smoke
  test, so there's no core oversubscription at that worker count — headroom
  exists to go to `workers=16` or `32` for further gains, modulo
  memory-footprint-per-worker, which hasn't been characterized.

A real measured speedup number (matched `workers=1` vs `workers=8` run over
identical `simulation_runs`) is worth capturing before relying on this
section for planning purposes.
