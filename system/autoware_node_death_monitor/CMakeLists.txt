cmake_minimum_required(VERSION 3.14)
project(autoware_node_death_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/autoware_node_death_monitor.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::node_death_monitor::NodeDeathMonitor"
  EXECUTABLE ${PROJECT_NAME}_node)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
