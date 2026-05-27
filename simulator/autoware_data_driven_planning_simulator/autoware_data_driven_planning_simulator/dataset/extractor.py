"""ROS bag to versioned dataset extraction."""

from __future__ import annotations

import csv
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from autoware_data_driven_planning_simulator.baseline import Command, State
from autoware_data_driven_planning_simulator.dataset.filters import (
    classify_maneuver,
    split_by_segment,
    validate_maneuver_coverage,
)
from autoware_data_driven_planning_simulator.dataset.rosbag_reader import read_bag


def _stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _yaw_from_quaternion(q: Any) -> float:
    from math import atan2

    return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def state_from_msg(msg: Any) -> State:
    pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
    twist = msg.twist.twist if hasattr(msg, "twist") and hasattr(msg.twist, "twist") else None
    state = State(x=pose.position.x, y=pose.position.y, yaw=_yaw_from_quaternion(pose.orientation))
    if twist is not None:
        state.vx = twist.linear.x
        state.vy = twist.linear.y
        state.wz = twist.angular.z
    return state


def command_from_msg(msg: Any) -> Command:
    if hasattr(msg, "longitudinal") and hasattr(msg, "lateral"):
        return Command(
            velocity=float(msg.longitudinal.velocity),
            acceleration=float(msg.longitudinal.acceleration),
            steer=float(msg.lateral.steering_tire_angle),
            vx=float(msg.longitudinal.velocity),
        )
    if hasattr(msg, "linear") and hasattr(msg, "angular"):
        return Command(vx=float(msg.linear.x), vy=float(msg.linear.y), wz=float(msg.angular.z))
    return Command()


def _nearest_before(series: list[tuple[float, Any]], t: float) -> Any | None:
    candidate = None
    for stamp, value in series:
        if stamp <= t:
            candidate = value
        else:
            break
    return candidate


def extract_dataset(
    bag_uri: str,
    vehicle_profile: dict,
    filter_config: dict,
    output_dir: str,
) -> dict:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)

    state_topic = vehicle_profile["topics"]["state"]
    command_topic = vehicle_profile["topics"]["command"]
    topics = {state_topic, command_topic}
    states: list[tuple[float, State]] = []
    commands: list[tuple[float, Command]] = []

    for item in read_bag(bag_uri, topics):
        t = item.timestamp_ns * 1.0e-9
        if item.topic == state_topic:
            states.append((t, state_from_msg(item.message)))
        elif item.topic == command_topic:
            commands.append((t, command_from_msg(item.message)))

    if len(states) < 2:
        raise RuntimeError(f"not enough state samples in {state_topic}")
    if not commands:
        raise RuntimeError(f"no command samples in {command_topic}")

    dt = float(filter_config.get("sampling_period_s", 0.1))
    start = max(states[0][0], commands[0][0])
    stop = min(states[-1][0], commands[-1][0])
    thresholds = filter_config.get("maneuver_thresholds", {})
    rows: list[dict] = []
    prev_state: State | None = None

    sample_index = 0
    t = start
    while t + dt <= stop:
        state = _nearest_before(states, t)
        next_state = _nearest_before(states, t + dt)
        command = _nearest_before(commands, t)
        if state is None or next_state is None or command is None:
            t += dt
            continue
        row = {
            "t": t,
            **{f"state_{k}": v for k, v in asdict(state).items()},
            **{f"command_{k}": v for k, v in asdict(command).items()},
            **{f"next_{k}": v for k, v in asdict(next_state).items()},
            "segment_id": "segment_0000",
            "bag_id": Path(bag_uri).name,
        }
        if prev_state is not None:
            row["state_ax"] = (state.vx - prev_state.vx) / dt
        row["vx"] = state.vx
        row["vy"] = state.vy
        row["wz"] = state.wz
        row["ax"] = row.get("state_ax", state.ax)
        row["maneuver"] = classify_maneuver(row, thresholds)
        rows.append(row)
        prev_state = state
        sample_index += 1
        t = start + sample_index * dt

    counts, failures = validate_maneuver_coverage(rows, vehicle_profile.get("required_maneuvers", {}))
    report = {
        "bag_uri": bag_uri,
        "state_topic": state_topic,
        "command_topic": command_topic,
        "sample_count": len(rows),
        "maneuver_counts": counts,
        "failures": failures,
        "passed": not failures,
        "velocity_label_source": "estimator_twist_preferred",
    }

    with (output / "data_validation_report.yaml").open("w") as f:
        yaml.safe_dump(report, f, sort_keys=False)
    with (output / "metadata.yaml").open("w") as f:
        yaml.safe_dump({"vehicle_profile": vehicle_profile, "filter_config": filter_config}, f, sort_keys=False)

    if rows:
        fieldnames = list(rows[0].keys())
        with (output / "dataset.csv").open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        splits = split_by_segment(
            rows,
            float(filter_config.get("split", {}).get("train_ratio", 0.7)),
            float(filter_config.get("split", {}).get("validation_ratio", 0.15)),
        )
        for name, split_rows in splits.items():
            with (output / f"{name}.csv").open("w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(split_rows)
        np.savez(output / "dataset.npz", rows=np.array([[r.get("state_x", 0.0), r.get("state_y", 0.0)] for r in rows]))
    if failures:
        raise RuntimeError("dataset validation failed: " + "; ".join(failures))
    return report

