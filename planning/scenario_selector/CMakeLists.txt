cmake_minimum_required(VERSION 3.14)
project(scenario_selector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(scenario_selector_node SHARED
  src/scenario_selector_node/scenario_selector_node.cpp
)

rclcpp_components_register_node(scenario_selector_node
  PLUGIN "ScenarioSelectorNode"
  EXECUTABLE scenario_selector
)

get_target_property(lanelet2_core_INCLUDE_DIRECTORIES lanelet2_core::lanelet2_core INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(scenario_selector_node
  SYSTEM PRIVATE
    ${lanelet2_core_INCLUDE_DIRECTORIES}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
