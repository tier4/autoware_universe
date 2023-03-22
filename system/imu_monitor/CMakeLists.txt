cmake_minimum_required(VERSION 3.14)
project(imu_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(imu_monitor
  src/imu_monitor_node.cpp
  src/imu_monitor.cpp
)
ament_target_dependencies(imu_monitor)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
