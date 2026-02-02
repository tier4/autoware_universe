cmake_minimum_required(VERSION 3.14)
project(autoware_static_freespace_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(static_freespace_planner_node SHARED
  src/autoware_static_freespace_planner/debug_publisher.cpp
  src/autoware_static_freespace_planner/static_freespace_planner_node.cpp
  src/autoware_static_freespace_planner/route_index_loader.cpp
  src/autoware_static_freespace_planner/route_matcher.cpp
  src/autoware_static_freespace_planner/trajectory_generator.cpp
  src/autoware_static_freespace_planner/waypoint_loader.cpp
)

rclcpp_components_register_node(static_freespace_planner_node
  PLUGIN "autoware::static_freespace_planner::StaticFreespacePlannerNode"
  EXECUTABLE static_freespace_planner
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  map_path
)