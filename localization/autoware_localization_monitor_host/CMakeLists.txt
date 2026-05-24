cmake_minimum_required(VERSION 3.14)
project(autoware_localization_monitor_host)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_localization_monitor_host plugins.xml)

# Plugin library (shared, loaded via pluginlib at runtime)
ament_auto_add_library(${PROJECT_NAME}_plugins SHARED
  src/localization_error_plugin.cpp
  src/pose_instability_plugin.cpp
)

# Host node library (shared, contains rclcpp::Node subclass + main entry)
ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/localization_monitor_host.cpp
)

target_link_libraries(${PROJECT_NAME}_component ${PROJECT_NAME}_plugins)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "autoware::localization_monitor_host::LocalizationMonitorHost"
  EXECUTABLE ${PROJECT_NAME}
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
