cmake_minimum_required(VERSION 3.5)
project(elevation_map_loader)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)

find_package(PCL REQUIRED COMPONENTS io)

ament_auto_add_library(elevation_map_loader_node SHARED
  src/elevation_map_loader_node.cpp
)
target_link_libraries(elevation_map_loader_node ${PCL_LIBRARIES})

rclcpp_components_register_node(elevation_map_loader_node
  PLUGIN "ElevationMapLoaderNode"
  EXECUTABLE elevation_map_loader
)

ament_target_dependencies(elevation_map_loader agnocastlib)
target_include_directories(elevation_map_loader PRIVATE
  ${agnocastlib_include_dirs}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  data
)
