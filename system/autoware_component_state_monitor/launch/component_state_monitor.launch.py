# Copyright 2022 TIER IV, Inc.
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


from collections import defaultdict
from pathlib import Path

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def create_diagnostic_name(row):
    return "{}_topic_status".format(row["module"])


def create_topic_monitor_name(row):
    diag_name = create_diagnostic_name(row)
    return "topic_state_monitor_{}: {}".format(row["args"]["node_name_suffix"], diag_name)


def create_topic_monitor_node(row):
    diag_name = create_diagnostic_name(row)
    args = row["args"]
    is_tf = "topic_type" not in args
    node_name = "topic_state_monitor_" + str(args["node_name_suffix"])

    params = {
        "topic": str(args["topic"]),
        "transient_local": bool(args.get("transient_local", False)),
        "best_effort": bool(args.get("best_effort", False)),
        "diag_name": diag_name,
        "warn_rate": float(args["warn_rate"]),
        "error_rate": float(args["error_rate"]),
        "timeout": float(args["timeout"]),
        "window_size": int(args.get("window_size", 10)),
    }

    if is_tf:
        params["frame_id"] = str(args["frame_id"])
        params["child_frame_id"] = str(args["child_frame_id"])
    else:
        params["topic_type"] = str(args["topic_type"])

    return Node(
        package="autoware_topic_state_monitor",
        executable="autoware_topic_state_monitor_agnocast_node",
        name=node_name,
        namespace="",
        parameters=[params],
        output="screen",
        additional_env={
            "LD_PRELOAD": "/opt/ros/humble/lib/libagnocast_heaphook.so",
        },
    )


def launch_setup(context, *args, **kwargs):
    # create topic monitors
    mode = LaunchConfiguration("mode").perform(context)
    rows = yaml.safe_load(Path(LaunchConfiguration("file").perform(context)).read_text())
    rows = [row for row in rows if mode in row["mode"]]
    topic_monitor_nodes = [create_topic_monitor_node(row) for row in rows]
    topic_monitor_names = [create_topic_monitor_name(row) for row in rows]
    topic_monitor_param = defaultdict(lambda: defaultdict(list))
    for row in rows:
        topic_monitor_param[row["type"]][row["module"]].append(create_topic_monitor_name(row))
    topic_monitor_param = {name: dict(module) for name, module in topic_monitor_param.items()}

    # create component_state_monitor as standalone node
    component = Node(
        package="autoware_component_state_monitor",
        executable="autoware_component_state_monitor_agnocast_node",
        name="component",
        namespace="component_state_monitor",
        parameters=[{"topic_monitor_names": topic_monitor_names}, topic_monitor_param],
        output="screen",
        additional_env={
            "LD_PRELOAD": "/opt/ros/humble/lib/libagnocast_heaphook.so",
        },
    )

    return [component, *topic_monitor_nodes]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument("file"),
            DeclareLaunchArgument("mode"),
            OpaqueFunction(function=launch_setup),
        ]
    )
