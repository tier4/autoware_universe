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

import os

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    def load_composable_node_param(param_path):
        with open(LaunchConfiguration(param_path).perform(context), "r") as f:
            return yaml.safe_load(f)["/**"]["ros__parameters"]

    ns = ""
    pkg = "autoware_euclidean_cluster"
    use_agnocast = os.environ.get("ENABLE_AGNOCAST", "0") == "1"

    # The node stays an rclcpp::Node and only Agnocast-wraps its output publisher (Method 1), so it
    # is always loaded as a composable node into a container. When Agnocast is enabled the container
    # must be an Agnocast component container with the heaphook preloaded so the Agnocast publisher
    # gets zero-copy; otherwise a plain rclcpp component container is used.
    component = ComposableNode(
        package=pkg,
        namespace=ns,
        plugin="autoware::euclidean_cluster::LabelBasedEuclideanClusterNode",
        name="label_based_euclidean_cluster",
        remappings=[
            ("input", LaunchConfiguration("input_pointcloud")),
            ("output", LaunchConfiguration("output_objects")),
        ],
        parameters=[
            load_composable_node_param("param_path"),
            {"shape_policy": LaunchConfiguration("shape_policy")},
        ],
    )

    if use_agnocast:
        container_package = "agnocast_components"
        container_executable = "agnocast_component_container"
        container_env = {
            "LD_PRELOAD": "/opt/ros/{}/lib/libagnocast_heaphook.so".format(
                os.environ.get("ROS_DISTRO", "humble")
            )
            + ":"
            + os.environ.get("LD_PRELOAD", ""),
        }
    else:
        container_package = "rclcpp_components"
        container_executable = "component_container"
        container_env = {}

    container = ComposableNodeContainer(
        name="label_based_euclidean_cluster_container",
        namespace=ns,
        package=container_package,
        executable=container_executable,
        composable_node_descriptions=[],
        output="screen",
        additional_env=container_env,
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
    )

    target_container = (
        LaunchConfiguration("pointcloud_container_name")
        if IfCondition(LaunchConfiguration("use_pointcloud_container")).evaluate(context)
        else container
    )

    loader = LoadComposableNodes(
        composable_node_descriptions=[component],
        target_container=target_container,
    )

    return [container, loader]


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

    return launch.LaunchDescription(
        [
            add_launch_arg("input_pointcloud", "/perception/ptv3/segmented/pointcloud"),
            add_launch_arg("output_objects", "objects"),
            add_launch_arg("use_pointcloud_container", "false"),
            add_launch_arg("pointcloud_container_name", "pointcloud_container"),
            add_launch_arg("shape_policy"),
            add_launch_arg(
                "param_path",
                [
                    FindPackageShare("autoware_euclidean_cluster"),
                    "/config/label_based_euclidean_cluster.param.yaml",
                ],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
