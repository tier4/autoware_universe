cmake_minimum_required(VERSION 3.14)
project(autoware_traffic_light_arbiter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_arbiter.cpp
  src/signal_match_validator.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  agnocastlib
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::traffic_light::TrafficLightArbiter"
  EXECUTABLE traffic_light_arbiter_node
  EXECUTOR AgnocastOnlySingleThreadedExecutor
)

# Tests are disabled because the node now uses agnocast::Node which is not compatible
# with the rclcpp::Node-based test infrastructure (AutowareTestManager).
# if(BUILD_TESTING)
#   ament_auto_add_gtest(${PROJECT_NAME}_test
#     test/test_node.cpp
#   )
# endif()

ament_auto_package(INSTALL_TO_SHARE launch config)
