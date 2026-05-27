# Copyright 2026 The Autoware Foundation.
#
# Licensed under the Apache License, Version 2.0 (the "License");

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    param_file = LaunchConfiguration("param_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "param_file",
                default_value=[
                    FindPackageShare("autoware_data_driven_planning_simulator"),
                    "/param/data_driven_planning_simulator.param.yaml",
                ],
                description="Path to data-driven planning simulator parameters.",
            ),
            Node(
                package="autoware_data_driven_planning_simulator",
                executable="autoware_data_driven_planning_simulator_node",
                namespace="simulation",
                output="screen",
                parameters=[ParameterFile(param_file, allow_substs=True)],
                remappings=[
                    ("input/initialpose", "/initialpose3d"),
                    ("input/ackermann_control_command", "/control/command/control_cmd"),
                    ("input/cmd_vel", "/cmd_vel"),
                    ("output/odometry", "/localization/kinematic_state"),
                    ("output/twist", "/vehicle/status/velocity_status"),
                    ("output/steering", "/vehicle/status/steering_status"),
                ],
            ),
        ]
    )
