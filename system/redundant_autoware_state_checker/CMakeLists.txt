cmake_minimum_required(VERSION 3.14)
project(redundant_autoware_state_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(redundant_autoware_state_checker
  src/redundant_autoware_state_checker.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
