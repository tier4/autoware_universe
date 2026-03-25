cmake_minimum_required(VERSION 3.14)
project(autoware_lane_departure_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lane_departure_checker_node/lane_departure_checker_node.cpp
  src/lane_departure_checker_node/parameters.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::lane_departure_checker::LaneDepartureCheckerNode"
  EXECUTABLE lane_departure_checker_node
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::lane_departure_checker::LaneDepartureCheckerNode"
  EXECUTABLE lane_departure_checker_node_agnocast
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
