from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="drivable_area_visual_filter",
                executable="drivable_area_visual_filter_node",
                name="drivable_area_visual_filter",
                output="screen",
                parameters=[
                    {
                        "input_objects_topic": "/perception/object_recognition/objects",
                        "output_objects_topic": "/perception/object_recognition/lane_filtered/objects",
                        "map_topic": "/map/vector_map",
                        "api_objects_topic": "/api/perception/lane_filtered/objects",
                    }
                ],
            )
        ]
    )
