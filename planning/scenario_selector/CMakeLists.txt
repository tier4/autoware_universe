cmake_minimum_required(VERSION 3.5)
project(scenario_selector)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Target

## Target executable
set(SCENARIO_SELECTOR_SRC
  src/scenario_selector_node/scenario_selector_node.cpp
)

## scenario_selector_node
ament_auto_add_library(scenario_selector_node SHARED
  ${SCENARIO_SELECTOR_SRC}
)

rclcpp_components_register_node(scenario_selector_node
  PLUGIN "ScenarioSelectorNode"
  EXECUTABLE scenario_selector
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
