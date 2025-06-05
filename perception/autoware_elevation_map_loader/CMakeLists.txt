cmake_minimum_required(VERSION 3.14)
project(autoware_elevation_map_loader)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS io)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  data
)
