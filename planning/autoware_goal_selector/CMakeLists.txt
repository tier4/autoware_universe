cmake_minimum_required(VERSION 3.14)
project(autoware_goal_selector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/debug_marker.cpp
  src/goal_selector_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::goal_selector::GoalSelectorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
