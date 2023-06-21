cmake_minimum_required(VERSION 3.5)
project(route_handler)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(route_handler SHARED
  src/route_handler.cpp
)

get_target_property(lanelet2_core_INCLUDE_DIRECTORIES lanelet2_core::lanelet2_core INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(route_handler
  SYSTEM PRIVATE
    ${lanelet2_core_INCLUDE_DIRECTORIES}
)

ament_auto_package()
