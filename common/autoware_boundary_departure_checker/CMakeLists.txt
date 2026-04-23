cmake_minimum_required(VERSION 3.14)
project(autoware_boundary_departure_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(utils SHARED
  src/conversion.cpp
  src/utils.cpp
  src/debug.cpp
  src/footprints_generator.cpp
)

ament_auto_add_library(uncrossable_boundary_checker SHARED
  src/uncrossable_boundary_checker.cpp
)

target_link_libraries(uncrossable_boundary_checker
  utils
)

ament_auto_package()
