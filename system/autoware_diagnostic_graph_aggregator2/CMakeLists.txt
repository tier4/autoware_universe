cmake_minimum_required(VERSION 3.14)
project(autoware_diagnostic_graph_aggregator2)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/common/config/loader.cpp"
  "src/common/config/parser.cpp"
  "src/common/graph/graph.cpp"
)
target_include_directories(${PROJECT_NAME} PRIVATE src/common)

ament_auto_add_executable(check_graph
  "src/tools/check_graph.cpp"
)
target_include_directories(check_graph PRIVATE src/common)

ament_auto_package(INSTALL_TO_SHARE launch)
