cmake_minimum_required(VERSION 3.14)
project(autoware_rule_based_planner_common)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/dummy.cpp
)

ament_auto_package()
