cmake_minimum_required(VERSION 3.8)

set(PROJECT_NAME autoware_remote_interfaces)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)

# Project-wide include path
include_directories(include)

add_subdirectory(src)

# install launch and config
install(
    DIRECTORY 
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp autoware_adapi_v1_msgs autoware_perception_msgs autoware_planning_msgs)
ament_package()
