cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_multi_camera_fusion)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

include_directories(
  SYSTEM
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_multi_camera_fusion_node.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  agnocastlib
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::traffic_light::MultiCameraFusion"
  EXECUTABLE traffic_light_multi_camera_fusion_node
  EXECUTOR AgnocastOnlySingleThreadedExecutor
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
