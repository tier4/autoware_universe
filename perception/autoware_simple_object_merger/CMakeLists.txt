cmake_minimum_required(VERSION 3.5)
project(autoware_simple_object_merger)

# Dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(autoware_agnocast_wrapper REQUIRED)

include_directories(
  include
)

# Targets
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/simple_object_merger_base.cpp
  src/simple_detected_object_merger_node.cpp
  src/simple_tracked_object_merger_node.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
  ${autoware_agnocast_wrapper_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  agnocastlib
  autoware_agnocast_wrapper
)

autoware_agnocast_wrapper_setup(${PROJECT_NAME})

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::simple_object_merger::SimpleDetectedObjectMergerNode"
  EXECUTABLE simple_object_merger_node
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::simple_object_merger::SimpleTrackedObjectMergerNode"
  EXECUTABLE simple_tracked_object_merger_node
)

# TODO: Re-enable tests after updating to work with agnocast_wrapper::Node
# Tests
# if(BUILD_TESTING)
#   list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#
#   ament_auto_add_gtest(simple_object_merger_node_tests
#     test/test_node.cpp
#   )
# endif()

# Package
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
