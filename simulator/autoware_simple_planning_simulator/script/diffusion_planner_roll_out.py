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
import signal
import subprocess
import time

from ament_index_python import get_package_share_directory
from geometry_msgs.msg import AccelWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rosbag2_interfaces.srv import Pause
from rosbag2_interfaces.srv import Resume
from std_srvs.srv import Empty


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
        # start rosbag
        REMAP_TOPICS = [
            "/control/command/control_cmd",
            "/planning/trajectory",
            "/localization/acceleration",
            "/localization/kinematic_state",
            "/tf",
        ]
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
        cmd.extend([f"{topic}:=/rosbag{topic}" for topic in REMAP_TOPICS])
        self.rosbag_player = subprocess.Popen(cmd)

        self.remap_before_rollout()
        """
        NOTE
        /localization/kinematic_stateを/rosbag/localization/kinematic_stateにremapしているのに，lsim(rollout前)でdiffusion_plannerがplanningできるのはなぜ…？
        """

    def remap_before_rollout(self):
        proc = subprocess.Popen("ros2 run topic_tools relay /rosbag/tf /tf", shell=True)
        self.remapping_procs_before_rollout = [proc]

    def pause_sim(self, req, res):
        # stop sim
        pause_rosbag = self.create_client(Pause, "/rosbag2_player/pause")
        pause_rosbag.wait_for_service(timeout_sec=5.0)
        pause_rosbag.call_async(Pause.Request())

        # start simple_planning_simulator
        cmd = f'ros2 launch autoware_simple_planning_simulator simple_planning_simulator.launch.py \
        vehicle_info_param_file:={get_package_share_directory(self.args.vehicle_model + "_description")}/config/vehicle_info.param.yaml \
        simulator_model_param_file:={get_package_share_directory(self.args.vehicle_model + "_description")}/config/simulator_model.param.yaml \
        initial_engage_state:=true \
        raw_vehicle_cmd_converter_param_path:={get_package_share_directory("autoware_launch")}/config/vehicle/raw_vehicle_cmd_converter/raw_vehicle_cmd_converter.param.yaml \
        motion_publish_mode:="pose_only" \
        rollout:=true'
        self.result = subprocess.Popen(cmd, shell=True)

        # wait for simple_planning_simulator to become active
        time.sleep(self.args.pause_duration)

        # resume sim
        self.resume_sim()

        return res

    def resume_sim(self):
        self.remap_after_rollout()

        # resume
        resume_rosbag = self.create_client(Resume, "/rosbag2_player/resume")
        resume_rosbag.wait_for_service(timeout_sec=5.0)
        resume_rosbag.call_async(Resume.Request())

    def remap_after_rollout(self):
        for remapping_proc_before_rollout in self.remapping_procs_before_rollout:
            remapping_proc_before_rollout.send_signal(signal.SIGKILL)
            remapping_proc_before_rollout.wait()

        proc = subprocess.Popen("ros2 run topic_tools relay /simulation/tf /tf", shell=True)
        self.remapping_procs_after_rollout = [proc]

    def kinematic_state_cb(self, msg):
        self.kinematic_state = msg
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.pose = self.kinematic_state.pose
        self.initial_pose_pub.publish(initial_pose)

    def acceleration_cb(self, msg):
        self.acceleration = msg
        initial_acceleration = TwistStamped()
        initial_acceleration.twist.linear = self.acceleration.accel.accel.linear
        initial_acceleration.twist.angular = self.acceleration.accel.accel.angular
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
        help="pause duration to wait for simple_planning_simulator become active",
    )
    parser.add_argument("--rate", type=float, default=1.0, help="rosbag replay rate")
    parser.add_argument("--rosbag-dir", type=Path, required=True, help="Path to rosbag directory")
    parser.add_argument("--vehicle-model", type=str, required=True, help="autoware vehicle_model")
    args = parser.parse_args()

    rclpy.init()
    rollout_node = RollOut(args)
    rclpy.spin(rollout_node)
    rclpy.shutdown()
