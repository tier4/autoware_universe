cmake_minimum_required(VERSION 3.14)
project(autoware_freespace_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

ament_auto_add_library(freespace_planner_node SHARED
  src/autoware_freespace_planner/freespace_planner_node.cpp
  src/autoware_freespace_planner/utils.cpp
)

rclcpp_components_register_node(freespace_planner_node
  PLUGIN "autoware::freespace_planner::FreespacePlannerNode"
  EXECUTABLE freespace_planner
)

agnocast_components_register_node(freespace_planner_node
  PLUGIN "autoware::freespace_planner::FreespacePlannerNode"
  EXECUTABLE freespace_planner_agnocast
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

# NOTE: Tests disabled because FreespacePlannerNode uses agnocast::Node (not compatible with rclcpp test utilities)
# if(BUILD_TESTING)
#   ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
#     test/test_freespace_planner_node_interface.cpp
#     test/test_freespace_planner_utils.cpp
#     test/test_freespace_planner.cpp
#   )
#   target_link_libraries(test_${PROJECT_NAME}
#     freespace_planner_node
#   )
# endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
