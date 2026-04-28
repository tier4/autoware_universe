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

import argparse
from pathlib import Path
import subprocess
import time

from ament_index_python import get_package_share_directory
from geometry_msgs.msg import AccelWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import psutil
import rclpy
from rclpy.node import Node
from rosbag2_interfaces.srv import Pause
from rosbag2_interfaces.srv import Resume
from std_srvs.srv import Empty

ROSBAG_REMAP_TOPICS = [
    "/control/command/control_cmd",
    "/planning/trajectory",
    "/localization/acceleration",
    "/localization/kinematic_state",
    "/tf",
]


def get_regex_filtered_topic(rosbag_path, regexes):
    topics = set()
    for regex in regexes:
        result = subprocess.run(
            f"ros2 bag info {rosbag_path} | awk '{{print $2}}' | grep \"{regex}\"",
            shell=True,
            stdout=subprocess.PIPE,
            text=True,
        )
        for topic in str(result.stdout).split("\n"):
            topics.add(topic)

    topics.remove("")
    return topics


def kill_subprocess(proc):
    # subprocess.kill(), .terminate() does not kill ros2 process
    p = psutil.Process(proc.pid)
    try:
        for child in p.children(recursive=True):
            child.kill()
        p.kill()
    except Exception:
        pass


def create_topic_relay_process(src, dst):
    proc = subprocess.Popen(f"ros2 run topic_tools relay {src} {dst}", shell=True)
    return proc


class RollOut(Node):
    def __init__(self, args):
        super().__init__("rollout")

        self.args = args

        self.pause_trigger = self.create_service(Empty, "/rollout/pause", self.pause_sim)

        self.kinematic_state_sub = self.create_subscription(
            Odometry, "/rosbag/localization/kinematic_state", self.kinematic_state_cb, 1
        )
        self.acceleration_sub = self.create_subscription(
            AccelWithCovarianceStamped, "/rosbag/localization/acceleration", self.acceleration_cb, 1
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose3d", 10
        )
        self.initial_acceleration_pub = self.create_publisher(
            TwistStamped, "/simulation/input/initialtwist", 10
        )

        self.start()

    def start(self):
        remap_topics = get_regex_filtered_topic(
            self.args.rosbag_dir,
            ["^/planning/trajectory_generator/*", "^/control/*", "^/vehicle/*"],
        )
        for topic in ROSBAG_REMAP_TOPICS:
            remap_topics.add(topic)

        # start rosbag
        cmd = [
            "ros2",
            "bag",
            "play",
            "-s",
            "sqlite3",
            "-r",
            str(self.args.rate),
            str(self.args.rosbag_dir),
            "--remap",
        ]
        cmd.extend([f"{topic}:=/rosbag{topic}" for topic in remap_topics])
        self.rosbag_player = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
        )

        # start simple_planning_simulator
        cmd = f'ros2 launch autoware_simple_planning_simulator simple_planning_simulator.launch.py \
        vehicle_info_param_file:={get_package_share_directory(self.args.vehicle_model + "_description")}/config/vehicle_info.param.yaml \
        simulator_model_param_file:={get_package_share_directory(self.args.vehicle_model + "_description")}/config/simulator_model.param.yaml \
        initial_engage_state:=true \
        raw_vehicle_cmd_converter_param_path:={get_package_share_directory("autoware_launch")}/config/vehicle/raw_vehicle_cmd_converter/raw_vehicle_cmd_converter.param.yaml \
        motion_publish_mode:="pose_only" \
        rollout:=true'
        self.planning_simulator_proc = subprocess.Popen(cmd, shell=True)

        self.remap_before_rollout()

    def remap_before_rollout(self):
        self.remapping_procs_before_rollout = [
            create_topic_relay_process("/rosbag/tf", "/tf"),
            create_topic_relay_process(
                "/rosbag/localization/kinematic_state", "/localization/kinematic_state"
            ),
            create_topic_relay_process(
                "/rosbag/localization/acceleration", "/localization/acceleration"
            ),
            create_topic_relay_process(
                "/rosbag/vehicle/status/velocity_status", "/vehicle/status/velocity_status"
            ),
            create_topic_relay_process(
                "/rosbag/vehicle/status/steering_status", "/vehicle/status/steering_status"
            ),
            create_topic_relay_process(
                "/rosbag/vehicle/status/turn_indicators_status",
                "/vehicle/status/turn_indicators_status",
            ),
            create_topic_relay_process(
                "/rosbag/vehicle/status/control_mode", "/vehicle/status/control_mode"
            ),
            create_topic_relay_process(
                "/rosbag/vehicle/status/actuation_status", "/vehicle/status/actuation_status"
            ),
        ]

    def remap_after_rollout(self):
        for remapping_proc_before_rollout in self.remapping_procs_before_rollout:
            kill_subprocess(remapping_proc_before_rollout)

        self.remapping_procs_after_rollout = [
            create_topic_relay_process("/simulation/tf", "/tf"),
            create_topic_relay_process(
                "/simulation/localization/kinematic_state", "/localization/kinematic_state"
            ),
            create_topic_relay_process(
                "/simulation/localization/acceleration", "/localization/acceleration"
            ),
            create_topic_relay_process(
                "/simulation/vehicle/status/velocity_status", "/vehicle/status/velocity_status"
            ),
            create_topic_relay_process(
                "/simulation/vehicle/status/steering_status", "/vehicle/status/steering_status"
            ),
            create_topic_relay_process(
                "/simulation/vehicle/status/turn_indicators_status",
                "/vehicle/status/turn_indicators_status",
            ),
            create_topic_relay_process(
                "/simulation/vehicle/status/control_mode", "/vehicle/status/control_mode"
            ),
            create_topic_relay_process(
                "/simulation/vehicle/status/actuation_status", "/vehicle/status/actuation_status"
            ),
        ]

    def pause_sim(self, req, res):
        # stop sim
        pause_rosbag = self.create_client(Pause, "/rosbag2_player/pause")
        pause_rosbag.wait_for_service(timeout_sec=5.0)
        pause_rosbag.call_async(Pause.Request())

        self.remap_after_rollout()

        # resume
        time.sleep(self.args.pause_duration)
        resume_rosbag = self.create_client(Resume, "/rosbag2_player/resume")
        resume_rosbag.wait_for_service(timeout_sec=5.0)
        resume_rosbag.call_async(Resume.Request())

        return res

    def kinematic_state_cb(self, msg):
        self.kinematic_state = msg
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.pose = self.kinematic_state.pose
        initial_pose.header = self.kinematic_state.header
        # initial_pose.header.frame_id = self.kinematic_state.child_frame_id
        self.initial_pose_pub.publish(initial_pose)

    def acceleration_cb(self, msg):
        self.acceleration = msg
        initial_acceleration = TwistStamped()
        initial_acceleration.twist.linear = self.acceleration.accel.accel.linear
        initial_acceleration.twist.angular = self.acceleration.accel.accel.angular
        initial_acceleration.header = self.acceleration.header
        self.initial_acceleration_pub.publish(initial_acceleration)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--rollout-start-time", type=float, required=True, help="rollout start time"
    )
    parser.add_argument(
        "--pause-duration",
        type=float,
        default=5.0,
        help="pause duration before switching",
    )
    parser.add_argument("--rate", type=float, default=1.0, help="rosbag replay rate")
    parser.add_argument("--rosbag-dir", type=Path, required=True, help="Path to rosbag directory")
    parser.add_argument("--vehicle-model", type=str, required=True, help="autoware vehicle_model")
    args = parser.parse_args()

    rclpy.init()
    rollout_node = RollOut(args)
    rclpy.spin(rollout_node)
    rclpy.shutdown()
