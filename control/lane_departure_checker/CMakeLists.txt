cmake_minimum_required(VERSION 3.5)
project(lane_departure_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_auto_add_library(lane_departure_checker SHARED
  src/lane_departure_checker_node/lane_departure_checker.cpp
  src/lane_departure_checker_node/lane_departure_checker_node.cpp
)

rclcpp_components_register_node(lane_departure_checker
  PLUGIN "lane_departure_checker::LaneDepartureCheckerNode"
  EXECUTABLE lane_departure_checker_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(test_${PROJECT_NAME}
    test/test_lane_departure_checker.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    lane_departure_checker
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
