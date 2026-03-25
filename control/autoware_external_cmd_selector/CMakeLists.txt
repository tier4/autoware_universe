cmake_minimum_required(VERSION 3.14)
project(autoware_external_cmd_selector)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/autoware_external_cmd_selector/external_cmd_selector_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::external_cmd_selector::ExternalCmdSelector"
  EXECUTABLE autoware_external_cmd_selector
)

agnocast_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::external_cmd_selector::ExternalCmdSelector"
  EXECUTABLE autoware_external_cmd_selector_agnocast
  EXECUTOR AgnocastOnlySingleThreadedExecutor
  RESOURCE_INDEX agnocast_components
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
