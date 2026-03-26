cmake_minimum_required(VERSION 3.14)
project(autoware_pipeline_latency_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pipeline_latency_monitor_node.cpp
)

ament_target_dependencies(${PROJECT_NAME} agnocastlib)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::system::pipeline_latency_monitor::PipelineLatencyMonitorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::system::pipeline_latency_monitor::PipelineLatencyMonitorNode"
  EXECUTABLE ${PROJECT_NAME}_agnocast_node
  EXECUTOR SingleThreadedAgnocastExecutor
  RESOURCE_INDEX agnocast_components
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
