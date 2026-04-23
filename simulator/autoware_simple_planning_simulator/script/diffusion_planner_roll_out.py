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
import subprocess

import rclpy
from rclpy.node import Node
from rosbag2_interfaces.srv import Pause
from rosbag2_interfaces.srv import Resume
from std_srvs.srv import Empty


class RollOut(Node):
    def __init__(self):
        super().__init__("rollout")

        self.pause_rosbag = self.create_client(Pause, "/rosbag2_player/pause")
        self.resume_rosbag = self.create_client(Resume, "/rosbag2_player/resume")

        self.pause_trigger = self.create_service(Empty, "/rollout/pause", self.pause_sim)
        self.resume_trigger = self.create_service(Empty, "/rollout/resume", self.resume_sim)

        self.start()

    def start(self):
        # start rosbag
        REMAP_TOPICS = [
            "/control/command/control_cmd",
            "/planning/trajectory",
            "/localization/acceleration",
            "/localization/kinematic_state",
        ]
        cmd = [
            "ros2",
            "bag",
            "play",
            "-s",
            "sqlite3",
            "-r",
            "1.0",  # TODO: argparse
            "/home/msobue/workspace/rosbags/e2e",  # TODO: argparse
            "--remap",
        ]
        cmd.extend([f"{topic}:=/rosbag{topic}" for topic in REMAP_TOPICS])
        self.rosbag_player = subprocess.Popen(cmd)
        """
        NOTE
        /localization/kinematic_stateを/rosbag/localization/kinematic_stateにremapしているのに，lsimでdiffusion_plannerがplanningできるのはなぜ…？

        /tfを変更しないと可視化したJ6の位置はズレたままになる(trajectoryが始まる位置だけRollOutでズレ始める)のが期待値だが…？
        """

    def pause_sim(self, req, res):
        # stop sim
        self.pause_rosbag.wait_for_service(timeout_sec=5.0)
        self.pause_rosbag.call_async(Pause.Request())

        # start simple_planning_simulator
        """
        ここでsimple_planning_simulatorを立ち上げつつ，現時点でのkinematic_stateとaccelerationを保存して
        initial_pose/initial_acceleartionをpubしてsimulatorに渡す必要がある？

        cmd = ['ros2', 'launch', 'autoware_simple_planning_simulator', 'simple_planning_simulator.launch.py',
               'vehicle_info_param_file:="$(ros2 pkg prefix --share j6_gen2_description)/config/vehicle_info.param.yaml"',
               'simulator_model_param_file:="$(ros2 pkg prefix --share j6_gen2_description)/config/simulator_model.param.yaml"',
               'initial_engage_state:=true',
               'raw_vehicle_cmd_converter_param_path:="$(ros2 pkg prefix --share autoware_launch)/config/vehicle/raw_vehicle_cmd_converter/raw_vehicle_cmd_converter.param.yaml"',
               'motion_publish_mode:="pose_only"'
               ]
        self.result = subprocess.Popen(cmd,
                                       shell=True, # for subst
                                       # stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE
                                       )
        """

        return res

    def resume_sim(self, req, res):
        # resume
        self.resume_rosbag.wait_for_service(timeout_sec=5.0)
        self.resume_rosbag.call_async(Resume.Request())

        return res


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    rclpy.init()
    rollout_node = RollOut()
    rclpy.spin(rollout_node)
    rclpy.shutdown()
