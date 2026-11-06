#!/usr/bin/env python3
# Copyright 2026 TIER IV, Inc.
# SPDX-License-Identifier: Apache-2.0

"""Validate and summarize a saved, manually collected `stores` comparison (no GPU execution)."""

import argparse
import json
import math
from pathlib import Path
import re
import statistics

SCENES = (
    "FreeSpaceCached",
    "Hairpin256Cached",
    "StaticObstacles64Cached",
    "StaticObstacleGeometryUpdates",
    "GridBoundaryCrossings",
    "ReferenceGeometryUpdates",
    "ShortProjectionPath",
    "OverlappingProjectionGroups",
)


def signature(report):
    """Exclude timings and the intended store selection, retaining all call diagnostics."""
    ignored = {
        "dynamics_output_store",
        "environment_note",
        "wall",
        "process_cpu",
        "phase_summary",
        "first_optimizer_call",
        "samples",
    }
    result = {key: value for key, value in report.items() if key not in ignored}
    for key, samples in (
        ("first_optimizer_call", [report["first_optimizer_call"]]),
        ("samples", report["samples"]),
    ):
        result[key] = [
            {k: v for k, v in sample.items() if k not in ("wall_ms", "process_cpu_ms", "phases")}
            for sample in samples
        ]
    return result


def summarize(directory):
    reports = {}
    for path in sorted(directory.glob("pair_*/*.json")):
        match = re.fullmatch(r"pair_([1-4])_phase_([01])_stores_([01])", path.parent.name)
        if not match:
            raise ValueError(f"Unexpected report directory: {path.parent}")
        pair, phase, stores = map(int, match.groups())
        report = json.loads(path.read_text())
        key = (report["scenario"], pair, phase, stores)
        if key in reports:
            raise ValueError(f"Duplicate report: {key}")
        expected = {
            "measurements_valid": True,
            "unexpected_outcomes_including_warmup": 0,
            "rollout_kernel": "split",
            "projection_pruning_enabled": True,
            "projection_pruning_math_supported": True,
            "projection_bounds_valid": True,
            "dynamics_output_store": "cooperative" if stores else "legacy",
            "mode": "instrumented" if phase else "latency_only",
            "measured_calls": 100,
            "warmup_calls_including_first": 10,
            "iterations": 10,
            "horizon": 80,
            "rollouts": 16384,
            "seed": 1,
        }
        for field, value in expected.items():
            if report.get(field) != value:
                raise ValueError(f"{path}: invalid {field}: {report.get(field)!r}")
        if [sample["cycle"] for sample in report["samples"]] != list(range(10, 110)):
            raise ValueError(f"{path}: incomplete measured calls")
        for sample in [report["first_optimizer_call"], *report["samples"]]:
            for field, value in {
                "iterations_completed": 10,
                "workload_complete": True,
                "outcome_expected": True,
                "rejected": False,
                "failed_iteration": -1,
                "validation": "none",
                "optimized_points": 9 if report["scenario"] == "ShortProjectionPath" else 80,
            }.items():
                if sample.get(field) != value:
                    raise ValueError(f"{path}: cycle {sample['cycle']}: invalid {field}")
            if phase:
                for name in ("rollout_dynamics", "rollout_cost"):
                    timing = sample["phases"]["MPPI/" + name]
                    if timing["calls"] != 10 or timing["gpu_calls"] != 10:
                        raise ValueError(f"{path}: incomplete {name} timing")
        reports[key] = report
    expected_keys = {
        (scene, pair, phase, stores)
        for scene in SCENES
        for pair in range(1, 5)
        for phase in (0, 1)
        for stores in (0, 1)
    }
    if reports.keys() != expected_keys:
        raise ValueError(
            "Expected exactly 128 reports: eight scenes, four pairs, two timing/store modes"
        )

    summary = {}
    for scene in SCENES:
        # Compare every process within each timing mode, not just the two members of a pair.
        for phase in (0, 1):
            reference = signature(reports[scene, 1, phase, 0])
            for pair in range(1, 5):
                for stores in (0, 1):
                    candidate = signature(reports[scene, pair, phase, stores])
                    changed = [
                        key
                        for key in reference.keys() | candidate.keys()
                        if reference.get(key) != candidate.get(key)
                    ]
                    if changed:
                        raise ValueError(
                            f"{scene}, phase={phase}, pair={pair}, stores={stores}: "
                            f"metadata/diagnostics differ: {changed}; retain raw reports, "
                            "but do not interpret these as a matched-workload store comparison"
                        )
        metrics = {}
        for label, phase in (
            ("wall_median_ms", 0),
            ("wall_p95_ms", 0),
            ("cpu_mean_ms", 0),
            ("dynamics_gpu_ms", 1),
            ("cost_gpu_ms", 1),
            ("dynamics_plus_cost_gpu_ms", 1),
        ):
            values = []
            for stores in (0, 1):
                processes = []
                for pair in range(1, 5):
                    report = reports[scene, pair, phase, stores]
                    if label.startswith("wall_"):
                        value = report["wall"][label.removeprefix("wall_")]
                    elif label == "cpu_mean_ms":
                        value = report["process_cpu"]["mean_ms"]
                    else:
                        if label == "dynamics_plus_cost_gpu_ms":
                            names = ("rollout_dynamics", "rollout_cost")
                        elif label == "dynamics_gpu_ms":
                            names = ("rollout_dynamics",)
                        else:
                            names = ("rollout_cost",)
                        value = sum(
                            report["phase_summary"]["MPPI/" + name]["gpu"]["mean_ms"]
                            for name in names
                        )
                    if (
                        not isinstance(value, (int, float))
                        or not math.isfinite(value)
                        or value <= 0
                    ):
                        raise ValueError(f"{scene}, pair={pair}, stores={stores}: invalid {label}")
                    processes.append(value)
                values.append(processes)
            legacy, cooperative = map(statistics.mean, values)
            metrics[label] = {
                "legacy": legacy,
                "cooperative": cooperative,
                "reduction_pct": 100 * (legacy - cooperative) / legacy,
                "legacy_processes": values[0],
                "cooperative_processes": values[1],
                "paired_deltas_ms": [b - a for a, b in zip(*values)],
            }
        summary[scene] = metrics
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    try:
        summary = summarize(args.directory)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f"Store comparison rejected: {error}\n")
    output = args.directory / "store_comparison.json"
    output.write_text(
        json.dumps(
            {
                "diagnostics_match": True,
                "scope": "process-level summaries; matching diagnostics are not full-trajectory hashes",
                "scenes": summary,
            },
            indent=2,
        )
        + "\n"
    )
    print("Validated 128 reports; all stored call diagnostics match within each scene/timing mode.")
    print("Scene | mean process median: legacy -> cooperative (ms) | reduction")
    for scene, metrics in summary.items():
        wall = metrics["wall_median_ms"]
        print(
            f"{scene} | {wall['legacy']:.3f} -> {wall['cooperative']:.3f} | {wall['reduction_pct']:.2f}%"
        )
    print(f"Store comparison: {output}")


if __name__ == "__main__":
    main()
