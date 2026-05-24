cmake_minimum_required(VERSION 3.14)
project(autoware_stack_evaluator)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_stack_evaluator plugins.xml)

find_package(pluginlib REQUIRED)

# Host node library
ament_auto_add_library(autoware_stack_evaluator_component SHARED
  src/stack_evaluator_node.cpp
)

rclcpp_components_register_node(autoware_stack_evaluator_component
  PLUGIN "autoware::stack_evaluator::StackEvaluatorNode"
  EXECUTABLE stack_evaluator
)

# Plugin library
ament_auto_add_library(autoware_stack_evaluator_plugins SHARED
  src/plugins/control_metrics_plugin.cpp
  src/plugins/planning_metrics_plugin.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
