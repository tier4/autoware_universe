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

"""Shared helpers for in-lane MRM rosbag record / analyze scripts."""

from __future__ import annotations

import bisect
import csv
import json
import re
import subprocess
import sys
from dataclasses import asdict
from dataclasses import dataclass
from dataclasses import field
from pathlib import Path
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple

from autoware_planning_msgs.msg import Trajectory
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

# --- Domain 1 recording (vehicle + planning_simulator on main ECU) ---
# Bridged MRM topics use /mrm/... prefix on Domain 1 (reversed domain_bridge entries).
TRAJECTORY_TOPIC_D1 = "/mrm/planning/in_lane_mrm_planner/output/trajectory"
TRIGGER_TOPIC_D1 = "/mrm/planning/in_lane_mrm_planner/input/trigger"
ODOM_TOPIC_D1 = "/mrm/localization/kinematic_state"
ACCEL_TOPIC_D1 = "/mrm/localization/acceleration"
# psim: main ECU publishes these on D1; MRM D3 consumes the bridged copy (no /mrm/ echo on D1).
ODOM_TOPIC_D1_MAIN = "/localization/kinematic_state"
ACCEL_TOPIC_D1_MAIN = "/localization/acceleration"
LONGITUDINAL_DIAG_TOPIC_D1 = "/mrm/control/trajectory_follower/longitudinal/diagnostic"
LATERAL_DIAG_TOPIC_D1 = "/mrm/control/trajectory_follower/lateral/diagnostic"
CONTROL_CMD_TOPIC_D1 = "/mrm/control/trajectory_follower/control_cmd"
MRM_DIAGNOSTICS_TOPIC_D1 = "/mrm/diagnostics"
GATE_CONTROL_TOPIC = "/control/control_command_gate/inputs/in_lane_stop/control"
GATE_GEAR_TOPIC = "/control/control_command_gate/inputs/in_lane_stop/gear"
# Domain 1 topic name (main -> mrm bridge source side)
JERK_TRIGGER_TOPIC_D1 = "/control/jerk_constant_deceleration_trigger"
OPERATION_MODE_TOPIC_D1 = "/system/operation_mode/state"

DOMAIN1_RECORD_TOPICS: Tuple[str, ...] = (
    TRAJECTORY_TOPIC_D1,
    TRIGGER_TOPIC_D1,
    ODOM_TOPIC_D1,
    ODOM_TOPIC_D1_MAIN,
    ACCEL_TOPIC_D1,
    ACCEL_TOPIC_D1_MAIN,
    LONGITUDINAL_DIAG_TOPIC_D1,
    LATERAL_DIAG_TOPIC_D1,
    CONTROL_CMD_TOPIC_D1,
    MRM_DIAGNOSTICS_TOPIC_D1,
    GATE_CONTROL_TOPIC,
    GATE_GEAR_TOPIC,
    JERK_TRIGGER_TOPIC_D1,
    OPERATION_MODE_TOPIC_D1,
)

DOMAIN1_ANALYZE_TOPICS: Tuple[str, ...] = DOMAIN1_RECORD_TOPICS

# Legacy D3 local names (analyzer only, for older bags recorded on MRM domain)
TRAJECTORY_TOPIC_D3 = "/planning/in_lane_mrm_planner/output/trajectory"
TRIGGER_TOPIC_D3 = "/planning/in_lane_mrm_planner/input/trigger"
ODOM_TOPIC_D3 = "/localization/kinematic_state"
LONGITUDINAL_DIAG_TOPIC_D3 = "/control/trajectory_follower/longitudinal/diagnostic"
CONTROL_CMD_TOPIC_D3 = "/control/trajectory_follower/control_cmd"
LEGACY_ANALYZE_TOPICS: Tuple[str, ...] = (
    TRAJECTORY_TOPIC_D3,
    TRIGGER_TOPIC_D3,
    ODOM_TOPIC_D3,
    LONGITUDINAL_DIAG_TOPIC_D3,
    CONTROL_CMD_TOPIC_D3,
)

# Backward-compatible aliases used by the analyzer
TRAJECTORY_TOPIC = TRAJECTORY_TOPIC_D1
TRAJECTORY_TOPIC_FALLBACK = TRAJECTORY_TOPIC_D3
ODOM_TOPIC = ODOM_TOPIC_D1
LONGITUDINAL_DIAG_TOPIC = LONGITUDINAL_DIAG_TOPIC_D1
CONTROL_CMD_TOPIC = CONTROL_CMD_TOPIC_D1

VELOCITY_EPS = 0.01
DEFAULT_MOVING_ODOM_THRESHOLD = 0.5
DEFAULT_STOPPED_ODOM_THRESHOLD = 0.1
DEFAULT_NEGATIVE_STOP_DIST_THRESHOLD = -0.05

LONGITUDINAL_DEBUG_CONTROL_STATE = 13
LONGITUDINAL_DEBUG_STOP_DIST = 28

CONTROL_STATE_LABELS = {
    0: "DRIVE",
    1: "STOPPING",
    2: "STOPPED",
    3: "EMERGENCY",
}

ANOMALY_MOVING_ALL_ZERO = "moving_all_zero"
ANOMALY_MOVING_EARLY_ZERO = "moving_early_zero"
ANOMALY_MOVING_NEGATIVE_STOP_DIST = "moving_negative_stop_dist"
ANOMALY_STOPPED_NONZERO_TRAJ = "stopped_but_traj_nonzero"


@dataclass
class TimeSeries:
    time_sec: List[float] = field(default_factory=list)
    values: List[float] = field(default_factory=list)

    def append(self, time_sec: float, value: float) -> None:
        self.time_sec.append(time_sec)
        self.values.append(value)

    def nearest(self, query_time_sec: float) -> Optional[float]:
        if not self.time_sec:
            return None
        index = bisect.bisect_left(self.time_sec, query_time_sec)
        if index == 0:
            return self.values[0]
        if index >= len(self.time_sec):
            return self.values[-1]
        before_time = self.time_sec[index - 1]
        after_time = self.time_sec[index]
        if abs(query_time_sec - before_time) <= abs(after_time - query_time_sec):
            return self.values[index - 1]
        return self.values[index]


@dataclass
class TrajectorySample:
    time_sec: float
    num_points: int
    v0: float
    v1: float
    v_min: float
    v_max: float
    first_zero_idx: Optional[int]
    all_zero: bool
    early_zero: bool
    odom_vx: Optional[float] = None
    stop_dist: Optional[float] = None
    control_state: Optional[int] = None
    gate_acc: Optional[float] = None
    follower_acc: Optional[float] = None
    anomaly_tags: List[str] = field(default_factory=list)


@dataclass
class AnomalyEvent:
    tag: str
    time_sec: float
    odom_vx: Optional[float]
    v0: float
    v_max: float
    stop_dist: Optional[float]
    control_state: Optional[int]
    all_zero: bool
    early_zero: bool


def resolve_bag_uri(path: Path) -> Path:
    """Return rosbag2 directory URI (directory or parent of a single .db3)."""
    path = path.expanduser().resolve()
    if path.is_dir():
        return path
    if path.is_file() and path.suffix == ".db3":
        parent = path.parent
        if (parent / "metadata.yaml").is_file():
            return parent
        return path
    raise ValueError(f"Not a rosbag2 directory or .db3 file: {path}")


def detect_storage_id(rosbag_path: Path) -> str:
    metadata_path = rosbag_path / "metadata.yaml"
    if not metadata_path.is_file():
        return "sqlite3"

    text = metadata_path.read_text(encoding="utf-8")
    match = re.search(r"storage_id:\s*['\"]?(\w+)['\"]?", text)
    if match:
        return match.group(1)
    return "sqlite3"


def bag_time_to_sec(timestamp_ns: int) -> float:
    return timestamp_ns * 1e-9


def get_topics(
    rosbag_path: Path,
    topic_names: Sequence[str],
    storage_id: str,
    *,
    quiet_missing: bool = False,
) -> Tuple[Dict[str, list], List[str]]:
    storage_options = rosbag2_py.StorageOptions(uri=str(rosbag_path), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    available = {info.name for info in reader.get_all_topics_and_types()}
    missing = [name for name in topic_names if name not in available]
    if missing and not quiet_missing:
        print("WARNING: topics not found in bag (skipped):")
        for name in missing:
            print(f"  - {name}")

    active_topics = [name for name in topic_names if name in available]
    if not active_topics:
        print("ERROR: none of the requested topics exist in the bag.")
        print("Available topics:")
        for name in sorted(available):
            print(f"  {name}")
        sys.exit(1)

    topic_filter = rosbag2_py.StorageFilter(topics=active_topics)
    reader.set_filter(topic_filter)

    topic_types = reader.get_all_topics_and_types()
    type_map = {info.name: info.type for info in topic_types}

    topics: Dict[str, list] = {name: [] for name in active_topics}

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        topics[topic].append((bag_time_to_sec(timestamp_ns), msg))

    return topics, missing


def resolve_trajectory_topic(topics: Dict[str, list]) -> Optional[str]:
    for candidate in (TRAJECTORY_TOPIC_D1, TRAJECTORY_TOPIC_D3):
        if candidate in topics:
            return candidate
    return None


def resolve_odom_topic(topics: Dict[str, list]) -> Optional[str]:
    for candidate in (ODOM_TOPIC_D1, ODOM_TOPIC_D1_MAIN, ODOM_TOPIC_D3):
        if candidate in topics:
            return candidate
    return None


def summarize_trajectory(msg: Trajectory, time_sec: float) -> TrajectorySample:
    if not msg.points:
        return TrajectorySample(
            time_sec=time_sec,
            num_points=0,
            v0=0.0,
            v1=0.0,
            v_min=0.0,
            v_max=0.0,
            first_zero_idx=None,
            all_zero=True,
            early_zero=False,
        )

    velocities = [float(point.longitudinal_velocity_mps) for point in msg.points]
    first_zero_idx = next(
        (index for index, velocity in enumerate(velocities) if velocity <= VELOCITY_EPS),
        None,
    )
    v1 = velocities[1] if len(velocities) > 1 else velocities[0]
    all_zero = all(velocity <= VELOCITY_EPS for velocity in velocities)
    early_zero = (
        not all_zero
        and velocities[0] <= VELOCITY_EPS
        and max(velocities) > VELOCITY_EPS
    )
    return TrajectorySample(
        time_sec=time_sec,
        num_points=len(velocities),
        v0=velocities[0],
        v1=v1,
        v_min=min(velocities),
        v_max=max(velocities),
        first_zero_idx=first_zero_idx,
        all_zero=all_zero,
        early_zero=early_zero,
    )


def trajectory_velocities_csv_rows(msg: Trajectory) -> List[Tuple[int, float, float]]:
    rows: List[Tuple[int, float, float]] = []
    arc_length = 0.0
    for index, point in enumerate(msg.points):
        if index > 0:
            previous = msg.points[index - 1]
            dx = float(point.pose.position.x - previous.pose.position.x)
            dy = float(point.pose.position.y - previous.pose.position.y)
            arc_length += (dx * dx + dy * dy) ** 0.5
        rows.append((index, arc_length, float(point.longitudinal_velocity_mps)))
    return rows


def classify_sample(
    sample: TrajectorySample,
    *,
    moving_odom_threshold: float,
    stopped_odom_threshold: float,
    negative_stop_dist_threshold: float,
) -> List[str]:
    tags: List[str] = []
    odom = sample.odom_vx
    if odom is None:
        return tags

    if odom >= moving_odom_threshold and sample.all_zero:
        tags.append(ANOMALY_MOVING_ALL_ZERO)
    if odom >= moving_odom_threshold and sample.early_zero:
        tags.append(ANOMALY_MOVING_EARLY_ZERO)
    if (
        odom >= moving_odom_threshold
        and sample.stop_dist is not None
        and sample.stop_dist < negative_stop_dist_threshold
    ):
        tags.append(ANOMALY_MOVING_NEGATIVE_STOP_DIST)
    if odom <= stopped_odom_threshold and not sample.all_zero and sample.v_max > VELOCITY_EPS:
        tags.append(ANOMALY_STOPPED_NONZERO_TRAJ)
    return tags


def write_timeline_csv(path: Path, samples: Sequence[TrajectorySample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "time_sec",
                "num_points",
                "v0",
                "v1",
                "v_min",
                "v_max",
                "first_zero_idx",
                "all_zero",
                "early_zero",
                "odom_vx",
                "stop_dist",
                "control_state",
                "control_state_label",
                "gate_acc",
                "follower_acc",
                "anomaly_tags",
            ]
        )
        for sample in samples:
            state_label = ""
            if sample.control_state is not None:
                state_label = CONTROL_STATE_LABELS.get(
                    int(round(sample.control_state)), str(int(round(sample.control_state)))
                )
            writer.writerow(
                [
                    f"{sample.time_sec:.6f}",
                    sample.num_points,
                    f"{sample.v0:.6f}",
                    f"{sample.v1:.6f}",
                    f"{sample.v_min:.6f}",
                    f"{sample.v_max:.6f}",
                    "" if sample.first_zero_idx is None else sample.first_zero_idx,
                    int(sample.all_zero),
                    int(sample.early_zero),
                    "" if sample.odom_vx is None else f"{sample.odom_vx:.6f}",
                    "" if sample.stop_dist is None else f"{sample.stop_dist:.6f}",
                    "" if sample.control_state is None else int(round(sample.control_state)),
                    state_label,
                    "" if sample.gate_acc is None else f"{sample.gate_acc:.6f}",
                    "" if sample.follower_acc is None else f"{sample.follower_acc:.6f}",
                    "|".join(sample.anomaly_tags),
                ]
            )


def write_anomalies_csv(path: Path, events: Sequence[AnomalyEvent]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "tag",
                "time_sec",
                "odom_vx",
                "v0",
                "v_max",
                "stop_dist",
                "control_state",
                "control_state_label",
                "all_zero",
                "early_zero",
            ],
        )
        writer.writeheader()
        for event in events:
            state_label = ""
            if event.control_state is not None:
                state_label = CONTROL_STATE_LABELS.get(event.control_state, str(event.control_state))
            writer.writerow(
                {
                    "tag": event.tag,
                    "time_sec": f"{event.time_sec:.6f}",
                    "odom_vx": "" if event.odom_vx is None else f"{event.odom_vx:.6f}",
                    "v0": f"{event.v0:.6f}",
                    "v_max": f"{event.v_max:.6f}",
                    "stop_dist": "" if event.stop_dist is None else f"{event.stop_dist:.6f}",
                    "control_state": "" if event.control_state is None else event.control_state,
                    "control_state_label": state_label,
                    "all_zero": int(event.all_zero),
                    "early_zero": int(event.early_zero),
                }
            )


def export_trajectory_profile(
    export_dir: Path,
    time_sec: float,
    msg: Trajectory,
    sample: TrajectorySample,
) -> Path:
    export_dir.mkdir(parents=True, exist_ok=True)
    stamp_label = f"{time_sec:.3f}".replace(".", "p")
    csv_path = export_dir / f"trajectory_{stamp_label}s.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["index", "accumulated_distance_m", "longitudinal_velocity_mps"])
        for index, distance, velocity in trajectory_velocities_csv_rows(msg):
            writer.writerow([index, f"{distance:.6f}", f"{velocity:.6f}"])

    meta_path = export_dir / f"trajectory_{stamp_label}s_meta.json"
    meta_path.write_text(
        json.dumps(asdict(sample), indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return csv_path


def record_rosbag(output_path: Path, topics: Sequence[str]) -> subprocess.Popen:
    command = ["ros2", "bag", "record", "-o", str(output_path), *topics]
    return subprocess.Popen(command, stdin=subprocess.PIPE)


def validate_rosbag(output_path: Path, required_topics: Sequence[str]) -> bool:
    try:
        result = subprocess.run(
            ["ros2", "bag", "info", str(output_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True,
        )
    except subprocess.CalledProcessError as error:
        print(f"ERROR: ros2 bag info failed: {error.stderr.strip()}")
        return False

    output = result.stdout
    missing: List[str] = []
    for topic_name in required_topics:
        pattern = rf"Topic: {re.escape(topic_name)}\s+\|.+?\| Count: (\d+)"
        match = re.search(pattern, output)
        if not match or int(match.group(1)) == 0:
            missing.append(topic_name)

    if missing:
        print("Recorded bag is missing messages on:")
        for topic_name in missing:
            print(f"  - {topic_name}")
        return False
    return True


def format_topic_help() -> str:
    return "\n".join(f"  - {topic}" for topic in DOMAIN1_RECORD_TOPICS)
