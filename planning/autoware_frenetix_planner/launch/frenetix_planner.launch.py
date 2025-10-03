from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="autoware_frenetix_planner",
            executable="frenetix_planner_node",
            name="frenetix_planner",
            output="screen",
            remappings=[
                ("~/input/reference_path", "/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path"),
                ("~/input/odometry", "/localization/kinematic_state"),
                ("~/input/acceleration", "/localization/acceleration"),
                ("~/input/control_cmd", "/control/command/control_cmd"),
                ("~/input/objects", "/perception/object_recognition/objects"),
                ("~/input/vector_map", "/map/vector_map"),
                ("~/output/trajectory", "/frenetix_planner/trajectory"),
            ],
            parameters=[
                {"update_rate": 10.0}
            ],
            arguments=["--ros-args", "--log-level", "INFO"]
        )
    ])