cmake_minimum_required(VERSION 3.14)
project(constant_acceleration_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(CONSTANT_LON_CON_LIB ${PROJECT_NAME}_lib)
ament_auto_add_library(${CONSTANT_LON_CON_LIB} SHARED
  src/constant_acceleration_controller.cpp
)

install(PROGRAMS
  scripts/constant_acceleration_tester.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
)
