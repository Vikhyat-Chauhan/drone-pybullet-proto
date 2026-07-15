#!/usr/bin/env python3
"""Per-strategy-per-run results CSV I/O."""
import csv
import os

CSV_FIELDNAMES = [
    "run", "strategy", "elapse_time", "zone_violations",
    "compute_latency_us", "compute_energy_j",
    "propulsion_energy_j", "propulsion_mean_power_w",
    "events_handled", "event_violated", "event_violated_deadline",
    "event_violated_preemptive", "event_violation_rate",
]


def init_csv(path: str) -> None:
    if not os.path.exists(path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", newline="") as f:
            csv.DictWriter(f, fieldnames=CSV_FIELDNAMES).writeheader()


def flush_csv(path: str, run_idx: int, buffered) -> None:
    with open(path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDNAMES)
        for rec in buffered:
            writer.writerow({
                "run": run_idx,
                "strategy": rec["strategy"],
                "elapse_time": round(rec["elapsed"], 2),
                "zone_violations": rec["violations"],
                "compute_latency_us": round(rec["compute_latency_us"], 2),
                "compute_energy_j": round(rec["compute_energy_j"], 2),
                "propulsion_energy_j": round(rec["energy_j"], 2),
                "propulsion_mean_power_w": round(rec["mean_power_w"], 2),
                "events_handled": rec["events_handled"],
                "event_violated": rec["events_violated"],
                "event_violated_deadline": rec["events_violated_deadline"],
                "event_violated_preemptive": rec["events_violated_preemptive"],
                "event_violation_rate": round(
                    rec["events_violated"] / rec["events_handled"] if rec["events_handled"] else 0.0, 2),
            })
