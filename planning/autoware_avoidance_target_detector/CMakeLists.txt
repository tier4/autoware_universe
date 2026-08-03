cmake_minimum_required(VERSION 3.14)
project(autoware_avoidance_target_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/object_filtering.cpp
  src/boundary.cpp
  src/parameter.cpp
)

ament_auto_package()
