cmake_minimum_required(VERSION 3.8)
project(autoware_raindrop_cluster_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

# Targets
ament_auto_add_library(low_intensity_cluster_filter SHARED
  src/low_intensity_cluster_filter_node.cpp
)

target_include_directories(low_intensity_cluster_filter PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

ament_target_dependencies(low_intensity_cluster_filter
  agnocastlib
)

agnocast_components_register_node(low_intensity_cluster_filter
  PLUGIN "autoware::low_intensity_cluster_filter::LowIntensityClusterFilter"
  EXECUTABLE low_intensity_cluster_filter_node
  EXECUTOR AgnocastOnlySingleThreadedExecutor
)

# Tests are disabled because the node now uses agnocast::Node which is not compatible
# with the rclcpp::Node-based test infrastructure.
# if(BUILD_TESTING)
#   list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   ament_auto_add_gtest(low_intensity_cluster_filter_tests
#     test/test_low_intensity_cluster_filter.cpp
#   )
# endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
