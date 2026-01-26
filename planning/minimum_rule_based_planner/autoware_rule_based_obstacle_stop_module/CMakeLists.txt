cmake_minimum_required(VERSION 3.14)
project(autoware_rule_based_obstacle_stop_module)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Export plugin description file
pluginlib_export_plugin_description_file(autoware_minimum_rule_based_planner plugins.xml)

# Create shared library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/obstacle_stop.cpp
)

ament_auto_package(INSTALL_TO_SHARE config)
