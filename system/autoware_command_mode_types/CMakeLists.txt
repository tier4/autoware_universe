cmake_minimum_required(VERSION 3.8)
project(autoware_command_mode_types)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_types SHARED
  "src/adapters/command_mode_status.cpp"
  "src/types/command_mode_status.cpp"
)

if(BUILD_TESTING)
endif()

ament_auto_package()
