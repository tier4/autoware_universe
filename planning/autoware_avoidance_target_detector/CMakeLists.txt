cmake_minimum_required(VERSION 3.14)
project(autoware_avoidance_target_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
  src/impl.cpp
  src/two_class_filter.cpp
  src/drivable_area.cpp
  src/traffic_rules.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::avoidance_target_detector::AvoidanceTargetDetectorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
