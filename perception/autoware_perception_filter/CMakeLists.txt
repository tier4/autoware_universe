cmake_minimum_required(VERSION 3.14)
project(autoware_perception_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(autoware_utils REQUIRED)
find_package(autoware_rtc_interface REQUIRED)
find_package(tier4_rtc_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(autoware_universe_utils REQUIRED)

# Include directories
include_directories(include)

# Create executable
ament_auto_add_executable(${PROJECT_NAME}_node
  src/perception_filter_node.cpp
)

# Install
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
