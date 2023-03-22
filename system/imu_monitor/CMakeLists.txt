cmake_minimum_required(VERSION 3.14)
project(imu_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(imu_monitor SHARED
  src/imu_monitor.cpp
)
ament_target_dependencies(imu_monitor)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "imu_monitor::ImuMonitor"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
