cmake_minimum_required(VERSION 3.14)
project(autoware_system_monitor_host)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_system_monitor_host plugins.xml)

# Host component library (the single node)
ament_auto_add_library(autoware_system_monitor_host_component SHARED
  src/system_monitor_host.cpp
)

rclcpp_components_register_node(autoware_system_monitor_host_component
  PLUGIN "autoware::system_monitor_host::SystemMonitorHost"
  EXECUTABLE autoware_system_monitor_host_node
)

# Plugin library (all monitor plugins)
ament_auto_add_library(autoware_system_monitor_host_plugins SHARED
  src/plugins/cpu_monitor_plugin.cpp
  src/plugins/gpu_monitor_plugin.cpp
  src/plugins/hdd_monitor_plugin.cpp
  src/plugins/mem_monitor_plugin.cpp
  src/plugins/net_monitor_plugin.cpp
  src/plugins/ntp_monitor_plugin.cpp
  src/plugins/process_monitor_plugin.cpp
  src/plugins/voltage_monitor_plugin.cpp
  src/plugins/duplicated_node_plugin.cpp
  src/plugins/processing_time_plugin.cpp
  src/plugins/pipeline_latency_plugin.cpp
  src/plugins/service_log_plugin.cpp
  src/plugins/topic_state_plugin.cpp
  src/plugins/component_state_aggregator_plugin.cpp
)

target_link_libraries(autoware_system_monitor_host_plugins
  autoware_system_monitor_host_component
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
