cmake_minimum_required(VERSION 3.8)
project(autoware_traffic_light_id_matcher)

# find dependencies

find_package(autoware_cmake REQUIRED)
autoware_package()

# targets
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/traffic_light_id_matcher_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::traffic_light::AutowareTrafficLightIdMatcherNode"
  EXECUTABLE traffic_light_id_matcher_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)
