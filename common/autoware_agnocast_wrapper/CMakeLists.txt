cmake_minimum_required(VERSION 3.14)
project(autoware_agnocast_wrapper)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(rclcpp REQUIRED)
find_package(agnocastlib REQUIRED)

include_directories(
  include
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  agnocastlib
  autoware_component_interface_utils
  autoware_object_recognition_utils
  autoware_utils_diagnostics
  autoware_utils_logging
  autoware_utils_rclcpp
  autoware_utils_tf
  message_filters
  tf2
  tf2_ros
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  agnocastlib
)

install(
  FILES cmake/agnocast_wrapper_config_extras.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(
  FILES cmake/autoware_agnocast_wrapper_register_node.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(
  DIRECTORY templates
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(
  CONFIG_EXTRAS "cmake/agnocast_wrapper_config_extras.cmake"
)
