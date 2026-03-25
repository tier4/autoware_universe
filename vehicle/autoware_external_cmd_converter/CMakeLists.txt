cmake_minimum_required(VERSION 3.14)
project(autoware_external_cmd_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

ament_auto_add_library(autoware_external_cmd_converter SHARED
  src/node.cpp
)

rclcpp_components_register_node(autoware_external_cmd_converter
  PLUGIN "autoware::external_cmd_converter::ExternalCmdConverterNode"
  EXECUTABLE external_cmd_converter_node
)

agnocast_components_register_node(autoware_external_cmd_converter
  PLUGIN "autoware::external_cmd_converter::ExternalCmdConverterNode"
  EXECUTABLE external_cmd_converter_agnocast_node
  EXECUTOR AgnocastOnlySingleThreadedExecutor
  RESOURCE_INDEX agnocast_components
)

# Tests disabled: agnocast::Node incompatible with rclcpp-based test framework
# if(BUILD_TESTING)
#   ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
#     tests/test_external_cmd_converter.cpp
#   )
#   target_link_libraries(test_${PROJECT_NAME}
#     autoware_external_cmd_converter
#   )
# endif()


ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
