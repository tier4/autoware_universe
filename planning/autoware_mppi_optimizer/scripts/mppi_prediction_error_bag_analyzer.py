#!/usr/bin/env python3
# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Analyze MPPI plant-model prediction error from recorded MCAP/rosbag2 bags.

Reads Float64Stamped topics published by trajectory_processor:
  .../debug/mppi/prediction/pos_error_m
  .../debug/mppi/prediction/yaw_error_rad
  .../debug/mppi/prediction/vel_error_mps
  .../debug/mppi/prediction/elapsed_s

Example:
  ros2 run autoware_mppi_optimizer mppi_prediction_error_bag_analyzer.py -- \\
    --bag ~/Downloads/Bags/my_run.mcap --plot --out-dir /tmp/mppi_pred_error
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple

import matplotlib

if "--no-show" in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag2_py
from autoware_internal_debug_msgs.msg import Float64Stamped
from rclpy.serialization import deserialize_message


DEFAULT_TOPIC_PREFIX = (
    "/planning/trajectory_generator/trajectory_processor/debug/mppi/prediction"
)

METRIC_SUFFIXES = (
    "pos_error_m",
    "yaw_error_rad",
    "vel_error_mps",
    "elapsed_s",
)


@dataclass
class TimeSeries:
    name: str
    unit: str
    timestamps_s: List[float]
    values: List[float]

    @property
    def count(self) -> int:
        return len(self.values)


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def default_topics(prefix: str) -> Dict[str, str]:
    return {suffix: f"{prefix}/{suffix}" for suffix in METRIC_SUFFIXES}


def open_bag_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)
    return reader


def discover_topic_map(reader: rosbag2_py.SequentialReader, prefix: str) -> Dict[str, str]:
    bag_topics = {item.name for item in reader.get_all_topics_and_types()}
    topic_map: Dict[str, str] = {}
    for suffix in METRIC_SUFFIXES:
        candidate = f"{prefix}/{suffix}"
        if candidate in bag_topics:
            topic_map[suffix] = candidate
    return topic_map


def read_float64_stamped_series(
    bag_path: Path, topic: str, suffix: str
) -> TimeSeries:
    reader = open_bag_reader(bag_path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    unit_by_suffix = {
        "pos_error_m": "m",
        "yaw_error_rad": "rad",
        "vel_error_mps": "m/s",
        "elapsed_s": "s",
    }
    series = TimeSeries(
        name=suffix,
        unit=unit_by_suffix.get(suffix, ""),
        timestamps_s=[],
        values=[],
    )

    while reader.has_next():
        _, raw, timestamp_ns = reader.read_next()
        msg = deserialize_message(raw, Float64Stamped)
        stamp_s = stamp_to_seconds(msg.stamp)
        if stamp_s <= 0.0:
            stamp_s = timestamp_ns * 1.0e-9
        series.timestamps_s.append(stamp_s)
        series.values.append(float(msg.data))

    return series


def percentile(values: Sequence[float], pct: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * pct / 100.0
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def summarize_series(series: TimeSeries) -> Dict[str, float]:
    values = series.values
    if not values:
        return {}
    abs_values = [abs(v) for v in values]
    return {
        "count": float(len(values)),
        "mean": statistics.fmean(values),
        "mean_abs": statistics.fmean(abs_values),
        "std": statistics.pstdev(values) if len(values) > 1 else 0.0,
        "min": min(values),
        "max": max(values),
        "p50_abs": percentile(abs_values, 50.0),
        "p95_abs": percentile(abs_values, 95.0),
        "p99_abs": percentile(abs_values, 99.0),
    }


def align_series_by_index(
    series_map: Dict[str, TimeSeries],
) -> Tuple[List[float], Dict[str, List[float]]]:
    """Align by sample index using pos_error_m as the reference length."""
    reference = series_map.get("pos_error_m")
    if reference is None or reference.count == 0:
        first = next(iter(series_map.values()))
        length = first.count
        time_s = first.timestamps_s
    else:
        length = reference.count
        time_s = reference.timestamps_s

    aligned: Dict[str, List[float]] = {}
    for suffix, series in series_map.items():
        if series.count == length:
            aligned[suffix] = series.values
        else:
            aligned[suffix] = series.values[:length]
    return time_s[:length], aligned


def write_csv(out_dir: Path, time_s: Sequence[float], aligned: Dict[str, List[float]]) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "prediction_error.csv"
    columns = ["time_s", *METRIC_SUFFIXES]
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        row_count = len(time_s)
        for idx in range(row_count):
            row = [time_s[idx]]
            for suffix in METRIC_SUFFIXES:
                values = aligned.get(suffix, [])
                row.append(values[idx] if idx < len(values) else "")
            writer.writerow(row)
    return csv_path


def plot_series(
    out_dir: Path,
    time_s: Sequence[float],
    aligned: Dict[str, List[float]],
    show: bool,
) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    plot_path = out_dir / "prediction_error.png"

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    plot_specs = (
        ("pos_error_m", "Position error [m]", "tab:blue"),
        ("yaw_error_rad", "Yaw error [rad]", "tab:orange"),
        ("vel_error_mps", "Velocity error [m/s]", "tab:green"),
        ("elapsed_s", "Elapsed since previous MPPI cycle [s]", "tab:gray"),
    )

    if time_s:
        t0 = time_s[0]
        rel_time = [t - t0 for t in time_s]
    else:
        rel_time = []

    for axis, (suffix, ylabel, color) in zip(axes, plot_specs):
        values = aligned.get(suffix, [])
        if rel_time and values:
            axis.plot(rel_time, values, color=color, linewidth=1.0)
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time since bag start [s]")
    fig.suptitle("MPPI plant prediction error (open-loop replay vs measured ego)")
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150)
    if show:
        plt.show()
    else:
        plt.close(fig)
    return plot_path


def print_summary(bag_path: Path, topic_map: Dict[str, str], series_map: Dict[str, TimeSeries]) -> None:
    print(f"Bag: {bag_path}")
    print("Topics:")
    for suffix, topic in topic_map.items():
        count = series_map.get(suffix, TimeSeries(suffix, "", [], [])).count
        print(f"  {suffix:16s} {topic} ({count} msgs)")

    print("\nSummary (abs percentiles use |error|):")
    header = (
        f"{'metric':16s} {'n':>6s} {'mean':>10s} {'mean|x|':>10s} "
        f"{'p50|x|':>10s} {'p95|x|':>10s} {'p99|x|':>10s} {'max|x|':>10s}"
    )
    print(header)
    for suffix in METRIC_SUFFIXES:
        series = series_map.get(suffix)
        if series is None or series.count == 0:
            continue
        stats = summarize_series(series)
        print(
            f"{suffix:16s} {int(stats['count']):6d} "
            f"{stats['mean']:10.4f} {stats['mean_abs']:10.4f} "
            f"{stats['p50_abs']:10.4f} {stats['p95_abs']:10.4f} "
            f"{stats['p99_abs']:10.4f} {max(abs(stats['min']), abs(stats['max'])):10.4f}"
        )

    pos = series_map.get("pos_error_m")
    yaw = series_map.get("yaw_error_rad")
    vel = series_map.get("vel_error_mps")
    if pos and yaw and vel and pos.count == yaw.count == vel.count and pos.count > 0:
        worst_idx = max(range(pos.count), key=lambda i: abs(pos.values[i]))
        print("\nWorst position-error sample:")
        print(
            f"  t={pos.timestamps_s[worst_idx]:.3f}s "
            f"pos={pos.values[worst_idx]:.4f} m "
            f"yaw={yaw.values[worst_idx]:.4f} rad "
            f"vel={vel.values[worst_idx]:.4f} m/s"
        )


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path, help="Path to MCAP/rosbag2 file")
    parser.add_argument(
        "--topic-prefix",
        default=DEFAULT_TOPIC_PREFIX,
        help="Prefix for prediction debug topics",
    )
    parser.add_argument("--out-dir", type=Path, help="Directory for CSV/plot output")
    parser.add_argument("--plot", action="store_true", help="Save/show prediction error plot")
    parser.add_argument("--no-show", action="store_true", help="Do not open an interactive plot window")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    bag_path = args.bag.expanduser().resolve()
    if not bag_path.exists():
        print(f"Bag not found: {bag_path}", file=sys.stderr)
        return 1

    reader = open_bag_reader(bag_path)
    topic_map = discover_topic_map(reader, args.topic_prefix.rstrip("/"))
    if not topic_map:
        expected = ", ".join(f"{args.topic_prefix}/{suffix}" for suffix in METRIC_SUFFIXES)
        print(
            "No prediction error topics found in bag.\n"
            f"Expected topics like:\n  {expected}",
            file=sys.stderr,
        )
        return 1

    series_map: Dict[str, TimeSeries] = {}
    for suffix, topic in topic_map.items():
        series_map[suffix] = read_float64_stamped_series(bag_path, topic, suffix)

    print_summary(bag_path, topic_map, series_map)

    if args.out_dir is not None or args.plot:
        time_s, aligned = align_series_by_index(series_map)
        out_dir = args.out_dir.expanduser().resolve() if args.out_dir else bag_path.parent / f"{bag_path.stem}_pred_error"
        csv_path = write_csv(out_dir, time_s, aligned)
        print(f"\nWrote CSV: {csv_path}")
        if args.plot:
            plot_path = plot_series(out_dir, time_s, aligned, show=not args.no_show)
            print(f"Wrote plot: {plot_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
