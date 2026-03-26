cmake_minimum_required(VERSION 3.14)
project(autoware_processing_time_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(rclcpp REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
    src/processing_time_checker.cpp
)

ament_target_dependencies(${PROJECT_NAME} agnocastlib)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::processing_time_checker::ProcessingTimeChecker"
  EXECUTABLE processing_time_checker_node
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::processing_time_checker::ProcessingTimeChecker"
  EXECUTABLE processing_time_checker_agnocast_node
  EXECUTOR AgnocastOnlySingleThreadedExecutor
  RESOURCE_INDEX agnocast_components
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
