cmake_minimum_required(VERSION 3.5)
project(mission_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(mission_planner_node SHARED
  lib/mission_planner_base.cpp
  src/mission_planner_lanelet2/mission_planner_lanelet2.cpp
  src/mission_planner_lanelet2/utility_functions.cpp
)

rclcpp_components_register_node(mission_planner_node
  PLUGIN "mission_planner::MissionPlannerLanelet2"
  EXECUTABLE mission_planner
)

get_target_property(lanelet2_core_INCLUDE_DIRECTORIES lanelet2_core::lanelet2_core INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(mission_planner_node
  SYSTEM PRIVATE
    ${lanelet2_core_INCLUDE_DIRECTORIES}
)

ament_auto_add_library(goal_pose_visualizer_node SHARED
  src/goal_pose_visualizer/goal_pose_visualizer.cpp
)

rclcpp_components_register_node(goal_pose_visualizer_node
  PLUGIN "mission_planner::GoalPoseVisualizer"
  EXECUTABLE goal_pose_visualizer
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
