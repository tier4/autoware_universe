from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='septentrio_velocity_converter',
            executable='septentrio_velocity_converter_node',
            name='septentrio_velocity_converter_node',
            output='screen',
            parameters=[
                {'coordinate_system': 'Cartesian'}  # Geodetic or Cartesian
            ]
        )
    ])