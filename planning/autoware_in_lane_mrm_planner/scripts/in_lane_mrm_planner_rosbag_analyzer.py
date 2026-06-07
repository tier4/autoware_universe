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

"""Offline analyzer for in-lane MRM rosbag2 recordings (D1 bridged or D3 local topics).

Usage:
  source /path/to/install/setup.bash
  ros2 run autoware_in_lane_mrm_planner in_lane_mrm_planner_rosbag_analyzer /path/to/rosbag_dir
  ros2 run autoware_in_lane_mrm_planner in_lane_mrm_planner_rosbag_analyzer \\
    /path/to/rosbag_dir --output-dir ./mrm_report --no-show
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple

from autoware_control_msgs.msg import Control
from autoware_internal_debug_msgs.msg import Float32MultiArrayStamped
from autoware_planning_msgs.msg import Trajectory
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry

# Installed next to this executable under lib/autoware_in_lane_mrm_planner/
from in_lane_mrm_rosbag_common import ANOMALY_MOVING_ALL_ZERO
from in_lane_mrm_rosbag_common import ANOMALY_MOVING_EARLY_ZERO
from in_lane_mrm_rosbag_common import ANOMALY_MOVING_NEGATIVE_STOP_DIST
from in_lane_mrm_rosbag_common import ANOMALY_STOPPED_NONZERO_TRAJ
from in_lane_mrm_rosbag_common import CONTROL_CMD_TOPIC_D1
from in_lane_mrm_rosbag_common import CONTROL_CMD_TOPIC_D3
from in_lane_mrm_rosbag_common import CONTROL_STATE_LABELS
from in_lane_mrm_rosbag_common import DEFAULT_MOVING_ODOM_THRESHOLD
from in_lane_mrm_rosbag_common import DEFAULT_NEGATIVE_STOP_DIST_THRESHOLD
from in_lane_mrm_rosbag_common import DEFAULT_STOPPED_ODOM_THRESHOLD
from in_lane_mrm_rosbag_common import DOMAIN1_ANALYZE_TOPICS
from in_lane_mrm_rosbag_common import LEGACY_ANALYZE_TOPICS
from in_lane_mrm_rosbag_common import GATE_CONTROL_TOPIC
from in_lane_mrm_rosbag_common import LONGITUDINAL_DEBUG_CONTROL_STATE
from in_lane_mrm_rosbag_common import LONGITUDINAL_DEBUG_STOP_DIST
from in_lane_mrm_rosbag_common import LONGITUDINAL_DIAG_TOPIC_D1
from in_lane_mrm_rosbag_common import LONGITUDINAL_DIAG_TOPIC_D3
from in_lane_mrm_rosbag_common import ODOM_TOPIC_D1
from in_lane_mrm_rosbag_common import ODOM_TOPIC_D3
from in_lane_mrm_rosbag_common import AnomalyEvent
from in_lane_mrm_rosbag_common import TimeSeries
from in_lane_mrm_rosbag_common import TrajectorySample
from in_lane_mrm_rosbag_common import classify_sample
from in_lane_mrm_rosbag_common import detect_storage_id
from in_lane_mrm_rosbag_common import export_trajectory_profile
from in_lane_mrm_rosbag_common import format_topic_help
from in_lane_mrm_rosbag_common import get_topics
from in_lane_mrm_rosbag_common import resolve_bag_uri
from in_lane_mrm_rosbag_common import resolve_odom_topic
from in_lane_mrm_rosbag_common import resolve_trajectory_topic
from in_lane_mrm_rosbag_common import summarize_trajectory
from in_lane_mrm_rosbag_common import write_anomalies_csv
from in_lane_mrm_rosbag_common import write_timeline_csv


class InLaneMrmPlannerRosbagAnalyzer:
    def __init__(
        self,
        rosbag_path: Path,
        output_dir: Optional[Path],
        show_plots: bool,
        *,
        moving_odom_threshold: float,
        stopped_odom_threshold: float,
        negative_stop_dist_threshold: float,
        export_max_samples: int,
        export_all_anomalies: bool,
    ):
        self.rosbag_path = rosbag_path
        self.output_dir = output_dir
        self.show_plots = show_plots
        self.moving_odom_threshold = moving_odom_threshold
        self.stopped_odom_threshold = stopped_odom_threshold
        self.negative_stop_dist_threshold = negative_stop_dist_threshold
        self.export_max_samples = export_max_samples
        self.export_all_anomalies = export_all_anomalies
        self.storage_id = detect_storage_id(rosbag_path)

        self.trajectory_samples: List[TrajectorySample] = []
        self.trajectory_messages: List[Tuple[float, Trajectory]] = []
        self.odom_velocity = TimeSeries()
        self.stop_dist = TimeSeries()
        self.control_state = TimeSeries()
        self.control_acc = TimeSeries()
        self.gate_acc = TimeSeries()
        self.anomaly_spans: List[Tuple[float, float, str]] = []

    def run(self) -> None:
        print("===== in_lane_mrm_planner rosbag analyzer =====")
        print(f"Bag path   : {self.rosbag_path}")
        print(f"Storage id : {self.storage_id}")

        analyze_topics = tuple(
            dict.fromkeys(DOMAIN1_ANALYZE_TOPICS + LEGACY_ANALYZE_TOPICS)
        )
        topics, missing = get_topics(self.rosbag_path, analyze_topics, self.storage_id)

        traj_topic = resolve_trajectory_topic(topics)
        if traj_topic:
            self._load_trajectory(topics[traj_topic])

        odom_topic = resolve_odom_topic(topics)
        if odom_topic:
            self._load_odometry(topics[odom_topic])

        diag_topic = (
            LONGITUDINAL_DIAG_TOPIC_D1
            if LONGITUDINAL_DIAG_TOPIC_D1 in topics
            else LONGITUDINAL_DIAG_TOPIC_D3
        )
        if diag_topic in topics:
            self._load_longitudinal_diagnostic(topics[diag_topic])

        cmd_topic = (
            CONTROL_CMD_TOPIC_D1 if CONTROL_CMD_TOPIC_D1 in topics else CONTROL_CMD_TOPIC_D3
        )
        if cmd_topic in topics:
            self._load_control(topics[cmd_topic], self.control_acc)
        if GATE_CONTROL_TOPIC in topics:
            self._load_control(topics[GATE_CONTROL_TOPIC], self.gate_acc)

        self._enrich_samples()
        events = self._collect_anomaly_events()
        self._build_anomaly_spans(events)
        self._print_summary(missing, events)
        self._write_exports(events)
        self._plot()

        if self.output_dir:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            output_path = self.output_dir / "in_lane_mrm_planner_analysis.png"
            plt.savefig(output_path, dpi=150, bbox_inches="tight")
            print(f"Saved plot: {output_path}")

        if self.show_plots:
            plt.show()

    def _load_trajectory(self, messages: list) -> None:
        for time_sec, msg in messages:
            if isinstance(msg, Trajectory):
                sample = summarize_trajectory(msg, time_sec)
                self.trajectory_samples.append(sample)
                self.trajectory_messages.append((time_sec, msg))

    def _load_odometry(self, messages: list) -> None:
        for time_sec, msg in messages:
            if isinstance(msg, Odometry):
                self.odom_velocity.append(time_sec, float(msg.twist.twist.linear.x))

    def _load_longitudinal_diagnostic(self, messages: list) -> None:
        for time_sec, msg in messages:
            if not isinstance(msg, Float32MultiArrayStamped):
                continue
            data = list(msg.data)
            if len(data) > LONGITUDINAL_DEBUG_CONTROL_STATE:
                self.control_state.append(
                    time_sec, float(data[LONGITUDINAL_DEBUG_CONTROL_STATE])
                )
            if len(data) > LONGITUDINAL_DEBUG_STOP_DIST:
                self.stop_dist.append(time_sec, float(data[LONGITUDINAL_DEBUG_STOP_DIST]))

    @staticmethod
    def _load_control(messages: list, series: TimeSeries) -> None:
        for time_sec, msg in messages:
            if isinstance(msg, Control):
                series.append(time_sec, float(msg.longitudinal.acceleration))

    def _enrich_samples(self) -> None:
        for sample in self.trajectory_samples:
            sample.odom_vx = self.odom_velocity.nearest(sample.time_sec)
            sample.stop_dist = self.stop_dist.nearest(sample.time_sec)
            state = self.control_state.nearest(sample.time_sec)
            sample.control_state = None if state is None else int(round(state))
            gate = self.gate_acc.nearest(sample.time_sec)
            sample.gate_acc = gate
            follower = self.control_acc.nearest(sample.time_sec)
            sample.follower_acc = follower
            sample.anomaly_tags = classify_sample(
                sample,
                moving_odom_threshold=self.moving_odom_threshold,
                stopped_odom_threshold=self.stopped_odom_threshold,
                negative_stop_dist_threshold=self.negative_stop_dist_threshold,
            )

    def _collect_anomaly_events(self) -> List[AnomalyEvent]:
        events: List[AnomalyEvent] = []
        for sample in self.trajectory_samples:
            for tag in sample.anomaly_tags:
                events.append(
                    AnomalyEvent(
                        tag=tag,
                        time_sec=sample.time_sec,
                        odom_vx=sample.odom_vx,
                        v0=sample.v0,
                        v_max=sample.v_max,
                        stop_dist=sample.stop_dist,
                        control_state=sample.control_state,
                        all_zero=sample.all_zero,
                        early_zero=sample.early_zero,
                    )
                )
        return events

    def _build_anomaly_spans(self, events: Sequence[AnomalyEvent]) -> None:
        if not self.trajectory_samples:
            return
        times = [sample.time_sec for sample in self.trajectory_samples if sample.anomaly_tags]
        if not times:
            return
        gap_threshold = 0.5
        span_start = times[0]
        span_end = times[0]
        for time_sec in times[1:]:
            if time_sec - span_end > gap_threshold:
                self.anomaly_spans.append((span_start, span_end, "suspicious"))
                span_start = time_sec
            span_end = time_sec
        self.anomaly_spans.append((span_start, span_end, "suspicious"))

    def _print_summary(self, missing_topics: Sequence[str], events: List[AnomalyEvent]) -> None:
        print("\n===== Summary =====")
        if missing_topics:
            unique_missing = sorted(set(missing_topics))
            print("Missing topics:", ", ".join(unique_missing))

        if self.trajectory_samples:
            all_zero = sum(1 for sample in self.trajectory_samples if sample.all_zero)
            early_zero = sum(1 for sample in self.trajectory_samples if sample.early_zero)
            print(f"Trajectory messages : {len(self.trajectory_samples)}")
            print(f"  all_zero count    : {all_zero}")
            print(f"  not_all_zero      : {len(self.trajectory_samples) - all_zero}")
            print(f"  early_zero count  : {early_zero}  (v0≈0 but vmax>0)")
            sample = self.trajectory_samples[0]
            print(
                f"  example v0,v1,vmin,vmax: {sample.v0:.3f}, {sample.v1:.3f}, "
                f"{sample.v_min:.3f}, {sample.v_max:.3f}"
            )

        if self.odom_velocity.values:
            print(f"Odom messages       : {len(self.odom_velocity.values)}")
            print(f"  odom vx max       : {max(self.odom_velocity.values):.3f} m/s")
            print(f"  odom vx min       : {min(self.odom_velocity.values):.3f} m/s")

        self._print_correlation_summary()

        if events:
            print("\n===== Anomaly counts (time-aligned) =====")
            counts = Counter(event.tag for event in events)
            for tag, count in counts.most_common():
                print(f"  {tag}: {count}")
            print(f"  total anomaly samples: {len(events)}")

        if self.control_state.values:
            states = {int(round(value)) for value in self.control_state.values}
            labels = ", ".join(CONTROL_STATE_LABELS.get(state, str(state)) for state in sorted(states))
            print(f"\nLongitudinal state  : {labels}")

        if self.stop_dist.values:
            print(
                f"stop_dist range     : [{min(self.stop_dist.values):.3f}, "
                f"{max(self.stop_dist.values):.3f}] m"
            )
            negative = sum(1 for value in self.stop_dist.values if value < 0.0)
            print(f"  negative samples  : {negative} / {len(self.stop_dist.values)}")

        if self.gate_acc.values:
            unique_acc = sorted(set(round(value, 3) for value in self.gate_acc.values))
            print(f"Gate acc unique     : {unique_acc} ({len(unique_acc)} values)")

        if self.control_acc.values:
            unique_acc = sorted(set(round(value, 3) for value in self.control_acc.values))
            print(f"Follower acc unique : {len(unique_acc)} distinct values")
            if len(unique_acc) <= 12:
                print(f"  values: {unique_acc}")
            else:
                print(f"  min/median/max: {unique_acc[0]}, {unique_acc[len(unique_acc) // 2]}, {unique_acc[-1]}")

    def _print_correlation_summary(self) -> None:
        if not self.trajectory_samples:
            return

        all_zero_moving = sum(
            1
            for sample in self.trajectory_samples
            if sample.all_zero
            and sample.odom_vx is not None
            and sample.odom_vx >= self.moving_odom_threshold
        )
        all_zero_stopped = sum(
            1
            for sample in self.trajectory_samples
            if sample.all_zero
            and sample.odom_vx is not None
            and sample.odom_vx <= self.stopped_odom_threshold
        )
        moving_all_zero = sum(
            1 for sample in self.trajectory_samples if ANOMALY_MOVING_ALL_ZERO in sample.anomaly_tags
        )
        moving_early_zero = sum(
            1
            for sample in self.trajectory_samples
            if ANOMALY_MOVING_EARLY_ZERO in sample.anomaly_tags
        )
        moving_neg_stop = sum(
            1
            for sample in self.trajectory_samples
            if ANOMALY_MOVING_NEGATIVE_STOP_DIST in sample.anomaly_tags
        )
        stopped_nonzero = sum(
            1
            for sample in self.trajectory_samples
            if ANOMALY_STOPPED_NONZERO_TRAJ in sample.anomaly_tags
        )

        print("\n===== Correlation (odom aligned to trajectory stamps) =====")
        print(f"  traj all_zero while moving : {all_zero_moving}")
        print(f"  traj all_zero while stopped: {all_zero_stopped}")
        print(f"  moving odom >= {self.moving_odom_threshold:.2f} m/s:")
        print(f"    traj all_zero           : {moving_all_zero}")
        print(f"    traj early_zero (v0≈0)  : {moving_early_zero}")
        print(f"    follower stop_dist < 0  : {moving_neg_stop}")
        print(f"  stopped odom <= {self.stopped_odom_threshold:.2f} m/s:")
        print(f"    traj has vmax > 0       : {stopped_nonzero}")

        if (
            self.odom_velocity.values
            and max(self.odom_velocity.values) > self.moving_odom_threshold
            and all(sample.all_zero for sample in self.trajectory_samples)
        ):
            count = len(self.trajectory_samples)
            print(
                "\n*** Likely velocity planner issue: odom shows motion but "
                f"ALL {count} trajectory samples are zero-filled. ***"
            )
            print(
                "    Check planner logs for 'zero velocity fallback' while v0 > 0 "
                "(infeasible constraint)."
            )
        elif moving_all_zero > 0:
            print(
                f"\n*** Partial zero-fill while moving: {moving_all_zero} trajectory samples "
                f"with odom >= {self.moving_odom_threshold:.2f} m/s and all_zero. ***"
            )

    def _write_exports(self, events: List[AnomalyEvent]) -> None:
        if not self.output_dir:
            return

        self.output_dir.mkdir(parents=True, exist_ok=True)
        write_timeline_csv(self.output_dir / "timeline.csv", self.trajectory_samples)
        write_anomalies_csv(self.output_dir / "anomalies.csv", events)
        print(f"Saved timeline: {self.output_dir / 'timeline.csv'}")
        print(f"Saved anomalies: {self.output_dir / 'anomalies.csv'}")

        export_dir = self.output_dir / "trajectory_exports"
        msg_map = {time_sec: msg for time_sec, msg in self.trajectory_messages}
        export_times: List[float] = []

        if self.export_all_anomalies:
            export_times = sorted({event.time_sec for event in events})
        else:
            priority = [
                ANOMALY_MOVING_ALL_ZERO,
                ANOMALY_MOVING_EARLY_ZERO,
                ANOMALY_MOVING_NEGATIVE_STOP_DIST,
            ]
            for tag in priority:
                for event in events:
                    if event.tag == tag and event.time_sec not in export_times:
                        export_times.append(event.time_sec)
                    if len(export_times) >= self.export_max_samples:
                        break
                if len(export_times) >= self.export_max_samples:
                    break

        if not export_times and self.trajectory_samples:
            # Fallback: export first/last/max-vmax samples for manual inspection
            export_times = [
                self.trajectory_samples[0].time_sec,
                self.trajectory_samples[-1].time_sec,
            ]

        export_times = export_times[: self.export_max_samples]
        sample_map = {sample.time_sec: sample for sample in self.trajectory_samples}
        exported = 0
        for time_sec in export_times:
            msg = msg_map.get(time_sec)
            sample = sample_map.get(time_sec)
            if msg is None or sample is None:
                continue
            path = export_trajectory_profile(export_dir, time_sec, msg, sample)
            exported += 1
            print(f"Exported trajectory profile: {path}")
        if exported == 0:
            print("No trajectory profiles exported (no matching messages).")

    def _plot(self) -> None:
        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        fig.suptitle(str(self.rosbag_path.name), fontsize=11)

        for start, end, label in self.anomaly_spans:
            for axis in axes:
                axis.axvspan(start, end, color="tab:red", alpha=0.08, label=label)

        if self.odom_velocity.time_sec:
            axes[0].plot(
                self.odom_velocity.time_sec,
                self.odom_velocity.values,
                label="odom vx",
                color="tab:blue",
            )
            axes[0].axhline(
                self.moving_odom_threshold,
                color="tab:orange",
                linestyle=":",
                linewidth=0.8,
                label=f"moving thr {self.moving_odom_threshold:.1f}",
            )
        axes[0].set_ylabel("odom vx [m/s]")
        axes[0].grid(True)
        axes[0].legend(loc="upper right", fontsize=8)

        if self.trajectory_samples:
            times = [sample.time_sec for sample in self.trajectory_samples]
            axes[1].plot(times, [sample.v0 for sample in self.trajectory_samples], label="traj v0")
            axes[1].plot(times, [sample.v1 for sample in self.trajectory_samples], label="traj v1")
            axes[1].plot(
                times,
                [sample.v_min for sample in self.trajectory_samples],
                label="traj vmin",
                alpha=0.7,
            )
            axes[1].plot(
                times,
                [sample.v_max for sample in self.trajectory_samples],
                label="traj vmax",
                alpha=0.7,
            )
        axes[1].set_ylabel("traj v [m/s]")
        axes[1].grid(True)
        axes[1].legend(loc="upper right", fontsize=8)

        if self.stop_dist.time_sec:
            axes[2].plot(
                self.stop_dist.time_sec,
                self.stop_dist.values,
                label=f"debug[{LONGITUDINAL_DEBUG_STOP_DIST}] stop_dist",
                color="tab:cyan",
            )
        if self.control_state.time_sec:
            ax_state = axes[2].twinx()
            ax_state.plot(
                self.control_state.time_sec,
                self.control_state.values,
                label=f"debug[{LONGITUDINAL_DEBUG_CONTROL_STATE}] state",
                color="tab:orange",
            )
            ax_state.set_ylabel("control state")
            ax_state.set_yticks([0, 1, 2, 3])
            ax_state.set_yticklabels(["DRIVE", "STOPPING", "STOPPED", "EMERGENCY"])
            ax_state.legend(loc="upper left", fontsize=8)
        axes[2].set_ylabel("stop_dist [m]")
        axes[2].axhline(0.0, color="gray", linestyle="--", linewidth=0.8)
        axes[2].grid(True)
        axes[2].legend(loc="upper right", fontsize=8)

        if self.gate_acc.time_sec:
            axes[3].plot(
                self.gate_acc.time_sec,
                self.gate_acc.values,
                label="gate control acc",
                color="tab:green",
            )
        if self.control_acc.time_sec:
            axes[3].plot(
                self.control_acc.time_sec,
                self.control_acc.values,
                label="follower control_cmd acc",
                color="tab:red",
                alpha=0.6,
            )
        axes[3].set_ylabel("longitudinal acc")
        axes[3].set_xlabel("bag time [s]")
        axes[3].grid(True)
        axes[3].legend(loc="upper right", fontsize=8)

        plt.tight_layout()


def parse_args() -> argparse.Namespace:
    epilog = (
        "Bag path must be a rosbag2 directory (split bags supported via metadata.yaml).\n"
        "A single .db3 file is accepted if it sits under a directory with metadata.yaml.\n\n"
        "Record on Domain 1 only (see in_lane_mrm_planner_rosbag_record).\n\n"
        "Topics:\n"
        f"{format_topic_help()}\n"
    )
    parser = argparse.ArgumentParser(
        description="Analyze in-lane MRM planner / follower data from a rosbag2 directory.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=epilog,
    )
    parser.add_argument("path", type=Path, help="Path to rosbag2 directory (or .db3 under it)")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Save PNG, timeline.csv, anomalies.csv, and trajectory_exports/",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open matplotlib window",
    )
    parser.add_argument(
        "--moving-odom-threshold",
        type=float,
        default=DEFAULT_MOVING_ODOM_THRESHOLD,
        help="odom vx threshold for 'moving' correlation checks [m/s]",
    )
    parser.add_argument(
        "--stopped-odom-threshold",
        type=float,
        default=DEFAULT_STOPPED_ODOM_THRESHOLD,
        help="odom vx threshold for 'stopped' checks [m/s]",
    )
    parser.add_argument(
        "--negative-stop-dist-threshold",
        type=float,
        default=DEFAULT_NEGATIVE_STOP_DIST_THRESHOLD,
        help="stop_dist below this is flagged while moving [m]",
    )
    parser.add_argument(
        "--export-max-samples",
        type=int,
        default=20,
        help="Max trajectory CSV exports under output-dir/trajectory_exports/",
    )
    parser.add_argument(
        "--export-all-anomalies",
        action="store_true",
        help="Export every anomalous trajectory (can be large)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        bag_path = resolve_bag_uri(args.path)
    except ValueError as error:
        print(f"ERROR: {error}")
        sys.exit(1)

    analyzer = InLaneMrmPlannerRosbagAnalyzer(
        rosbag_path=bag_path,
        output_dir=args.output_dir,
        show_plots=not args.no_show,
        moving_odom_threshold=args.moving_odom_threshold,
        stopped_odom_threshold=args.stopped_odom_threshold,
        negative_stop_dist_threshold=args.negative_stop_dist_threshold,
        export_max_samples=args.export_max_samples,
        export_all_anomalies=args.export_all_anomalies,
    )
    analyzer.run()


if __name__ == "__main__":
    main()
