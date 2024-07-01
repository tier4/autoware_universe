cmake_minimum_required(VERSION 3.14)
project(dummy_operation_mode_publisher)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(dummy_operation_mode_publisher SHARED
  src/dummy_operation_mode_publisher.cpp
)
ament_target_dependencies(dummy_operation_mode_publisher)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "dummy_operation_mode_publisher::DummyOperationModePublisher"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
