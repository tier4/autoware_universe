#!/usr/bin/env python3
"""Create tiny sqlite3 or MCAP bags for extractor integration tests.

This script is not run by default because it requires a sourced ROS 2 environment with rosbag2_py.
"""

import argparse
from pathlib import Path

import rosbag2_py
from autoware_control_msgs.msg import Control
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message


def _topic_metadata(name: str, type_name: str):
    return rosbag2_py.TopicMetadata(name=name, type=type_name, serialization_format="cdr")


def create_bag(output: str, storage_id: str) -> None:
    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer.create_topic(_topic_metadata("/localization/kinematic_state", "nav_msgs/msg/Odometry"))
    writer.create_topic(_topic_metadata("/control/command/control_cmd", "autoware_control_msgs/msg/Control"))

    for i in range(20):
        t_ns = i * 100_000_000
        odom = Odometry()
        odom.header.stamp.sec = t_ns // 1_000_000_000
        odom.header.stamp.nanosec = t_ns % 1_000_000_000
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = 0.1 * i
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 1.0
        writer.write("/localization/kinematic_state", serialize_message(odom), t_ns)

        control = Control()
        control.stamp = odom.header.stamp
        control.longitudinal.stamp = odom.header.stamp
        control.longitudinal.velocity = 1.0
        control.longitudinal.acceleration = 0.0
        control.lateral.stamp = odom.header.stamp
        control.lateral.steering_tire_angle = 0.0
        writer.write("/control/command/control_cmd", serialize_message(control), t_ns)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--storage-id", choices=["sqlite3", "mcap"], required=True)
    args = parser.parse_args()
    # Force message imports to fail early if the test environment is incomplete.
    get_message("nav_msgs/msg/Odometry")
    get_message("autoware_control_msgs/msg/Control")
    create_bag(args.output, args.storage_id)


if __name__ == "__main__":
    main()
