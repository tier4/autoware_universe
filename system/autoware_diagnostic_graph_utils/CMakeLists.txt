cmake_minimum_required(VERSION 3.14)
project(autoware_diagnostic_graph_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lib/graph.cpp
  src/lib/subscription.cpp
)

ament_auto_add_library(${PROJECT_NAME}_tools SHARED
  src/node/dump.cpp
  src/node/converter.cpp
  src/node/logging.cpp
)

target_link_libraries(${PROJECT_NAME}_tools
  ${agnocastlib_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}_tools SYSTEM PUBLIC
  ${agnocastlib_INCLUDE_DIRS}
)

rclcpp_components_register_node(${PROJECT_NAME}_tools
  PLUGIN "autoware::diagnostic_graph_utils::DumpNode"
  EXECUTABLE dump_node
)

rclcpp_components_register_node(${PROJECT_NAME}_tools
  PLUGIN "autoware::diagnostic_graph_utils::ConverterNode"
  EXECUTABLE converter_node
)

rclcpp_components_register_node(${PROJECT_NAME}_tools
  PLUGIN "autoware::diagnostic_graph_utils::LoggingNode"
  EXECUTABLE logging_node
)

agnocast_components_register_node(${PROJECT_NAME}_tools
  PLUGIN "autoware::diagnostic_graph_utils::LoggingNode"
  EXECUTABLE logging_agnocast_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

ament_auto_package(INSTALL_TO_SHARE launch)
