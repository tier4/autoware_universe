# Copyright 2020 Tier IV, Inc. All rights reserved.
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile


def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    car_traffic_light_classifier_model_param = ParameterFile(
        param_file=LaunchConfiguration("car_classifier_param_path").perform(context),
        allow_substs=True,
    )
    pedestrian_traffic_light_classifier_model_param = ParameterFile(
        param_file=LaunchConfiguration("pedestrian_classifier_param_path").perform(context),
        allow_substs=True,
    )

    container = ComposableNodeContainer(
        name="traffic_light_node_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            ComposableNode(
                package="traffic_light_classifier",
                plugin="traffic_light::TrafficLightClassifierNodelet",
                name="car_traffic_light_classifier",
                namespace="classification",
                parameters=[car_traffic_light_classifier_model_param],
                remappings=[
                    ("~/input/image", LaunchConfiguration("input/image")),
                    ("~/input/rois", LaunchConfiguration("output/rois")),
                    ("~/output/traffic_signals", "classified/car/traffic_signals"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
            ComposableNode(
                package="traffic_light_classifier",
                plugin="traffic_light::TrafficLightClassifierNodelet",
                name="pedestrian_traffic_light_classifier",
                namespace="classification",
                parameters=[pedestrian_traffic_light_classifier_model_param],
                remappings=[
                    ("~/input/image", LaunchConfiguration("input/image")),
                    ("~/input/rois", LaunchConfiguration("output/rois")),
                    ("~/output/traffic_signals", "classified/pedestrian/traffic_signals"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
            ComposableNode(
                package="autoware_traffic_light_signals_merger",
                plugin="autoware::traffic_light::TrafficLightSignalsMergerNode",
                name="traffic_light_signals_merger",
                namespace="classification",
                remappings=[
                    ("input/car_signals", "classified/car/traffic_signals"),
                    ("input/pedestrian_signals", "classified/pedestrian/traffic_signals"),
                    (
                        "input/expect_rois",
                        f"/perception/traffic_light_recognition/{namespace}/detection/expect/rois",
                    ),
                    ("output/traffic_light_signals", "traffic_signals"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
            ComposableNode(
                package="traffic_light_visualization",
                plugin="traffic_light::TrafficLightRoiVisualizerNodelet",
                name="traffic_light_roi_visualizer",
                parameters=[create_parameter_dict("enable_fine_detection")],
                remappings=[
                    ("~/input/image", LaunchConfiguration("input/image")),
                    ("~/input/rois", LaunchConfiguration("output/rois")),
                    ("~/input/rough/rois", "detection/rough/rois"),
                    (
                        "~/input/traffic_signals",
                        LaunchConfiguration("output/traffic_signals"),
                    ),
                    ("~/output/image", "debug/rois"),
                    ("~/output/image/compressed", "debug/rois/compressed"),
                    ("~/output/image/compressedDepth", "debug/rois/compressedDepth"),
                    ("~/output/image/theora", "debug/rois/theora"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
        ],
        output="both",
    )

    return [GroupAction([PushRosNamespace(namespace), container])]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    classifier_share_dir = get_package_share_directory("autoware_traffic_light_classifier")
    add_launch_arg("all_camera_namespaces", "[camera6, camera7]")
    add_launch_arg("enable_image_decompressor", "True")
    add_launch_arg("enable_fine_detection", "True")
    add_launch_arg("use_image_transport", "True")

    # traffic_light_classifier
    add_launch_arg(
        "car_classifier_param_path",
        os.path.join(
            classifier_share_dir, "config", "car_traffic_light_classifier_efficientNet.param.yaml"
        ),
    )
    add_launch_arg(
        "pedestrian_classifier_param_path",
        os.path.join(
            classifier_share_dir,
            "config",
            "pedestrian_traffic_light_classifier_efficientNet.param.yaml",
        ),
    )

    add_launch_arg("use_intra_process", "False")
    add_launch_arg("use_multithread", "False")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return LaunchDescription(
        [
            *launch_arguments,
            set_container_executable,
            set_container_mt_executable,
            OpaqueFunction(function=launch_setup),
        ]
    )
