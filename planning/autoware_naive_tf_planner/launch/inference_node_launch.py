from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(
            get_package_share_directory('autoware_naive_tf_planner'),
            'models/best_model.pth'
        ),
        description='Path to the trained model file'
    )
    
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='',
        description='Path to the lanelet2 map file (.osm)'
    )
    
    trace_back_step_arg = DeclareLaunchArgument(
        'trace_back_step',
        default_value='10',
        description='Number of history steps to consider'
    )
    
    look_ahead_steps_arg = DeclareLaunchArgument(
        'look_ahead_steps',
        default_value='30',
        description='Number of future steps to predict'
    )
    
    prediction_horizon_arg = DeclareLaunchArgument(
        'prediction_horizon',
        default_value='3.0',
        description='Prediction horizon in seconds'
    )
    
    prediction_dt_arg = DeclareLaunchArgument(
        'prediction_dt',
        default_value='0.1',
        description='Time step between prediction points in seconds'
    )
    
    local_map_range_arg = DeclareLaunchArgument(
        'local_map_range',
        default_value='80.0',
        description='Range to consider for local map elements in meters'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device to run inference on (cuda or cpu)'
    )
    
    # Create node
    inference_node = Node(
        package='autoware_naive_tf_planner',
        executable='autoware_naive_tf_planner_node',
        name='naive_tf_inference_node',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'map_path': LaunchConfiguration('map_path'),
            'trace_back_step': LaunchConfiguration('trace_back_step'),
            'look_ahead_steps': LaunchConfiguration('look_ahead_steps'),
            'prediction_horizon': LaunchConfiguration('prediction_horizon'),
            'prediction_dt': LaunchConfiguration('prediction_dt'),
            'local_map_range': LaunchConfiguration('local_map_range'),
            'device': LaunchConfiguration('device'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        model_path_arg,
        map_path_arg,
        trace_back_step_arg,
        look_ahead_steps_arg,
        prediction_horizon_arg,
        prediction_dt_arg,
        local_map_range_arg,
        device_arg,
        inference_node
    ])