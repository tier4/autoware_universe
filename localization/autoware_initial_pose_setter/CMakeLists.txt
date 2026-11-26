cmake_minimum_required(VERSION 3.14)
project(autoware_initial_pose_setter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/initial_pose_setter.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::initial_pose_setter::InitialPoseSetter"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
