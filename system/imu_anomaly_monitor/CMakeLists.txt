cmake_minimum_required(VERSION 3.14)
project(imu_anomaly_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(imu_anomaly_monitor
  src/imu_anomaly_monitor_node.cpp
  src/imu_anomaly_monitor.cpp
)
ament_target_dependencies(imu_anomaly_monitor)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
