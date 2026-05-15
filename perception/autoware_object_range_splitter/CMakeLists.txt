cmake_minimum_required(VERSION 3.14)
project(autoware_object_range_splitter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(autoware_agnocast_wrapper REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/object_range_splitter_node.cpp
)
ament_target_dependencies(${PROJECT_NAME} autoware_agnocast_wrapper)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::object_range_splitter::ObjectRangeSplitterNode"
  EXECUTABLE object_range_splitter_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlySingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_launch.py)
  add_launch_test(test/test_launch_composable.py)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
