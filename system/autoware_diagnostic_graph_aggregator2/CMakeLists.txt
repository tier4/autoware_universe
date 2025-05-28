cmake_minimum_required(VERSION 3.14)
project(autoware_diagnostic_graph_aggregator2)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/common/config/context.cpp"
  "src/common/config/entity.cpp"
  "src/common/config/loader.cpp"
  "src/common/config/substitutions.cpp"
  "src/common/config/yaml.cpp"
  "src/common/graph/graph.cpp"
  "src/common/graph/units.cpp"
  "src/common/graph/links.cpp"
  "src/common/graph/logic.cpp"
  "src/common/graph/levels.cpp"
  "src/common/logics/and.cpp"
  "src/common/utils/logger.cpp"
)
target_include_directories(${PROJECT_NAME} PRIVATE src/common)

ament_auto_add_library(aggregator SHARED
  "src/node/aggregator.cpp"
  "src/node/command_mode_mapping.cpp"
)
target_include_directories(aggregator PRIVATE src/common)

ament_auto_add_executable(check_graph
  "src/tools/check_graph.cpp"
)
target_include_directories(check_graph PRIVATE src/common)

rclcpp_components_register_node(aggregator
  PLUGIN "autoware::diagnostic_graph_aggregator::AggregatorNode"
  EXECUTABLE aggregator_node
)

ament_auto_package(INSTALL_TO_SHARE launch test)
