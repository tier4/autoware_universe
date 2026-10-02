cmake_minimum_required(VERSION 3.14)
project(autoware_manual_lane_change_handler)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_agnocast_wrapper REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/manual_lane_change_handler.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  autoware_planning_msgs
  autoware_map_msgs
  autoware_route_handler
  autoware_mission_planner_universe
  autoware_agnocast_wrapper
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::manual_lane_change_handler::ManualLaneChangeHandler"
  EXECUTABLE ${PROJECT_NAME}_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlySingleThreadedExecutor
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
