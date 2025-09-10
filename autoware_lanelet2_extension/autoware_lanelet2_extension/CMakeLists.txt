cmake_minimum_required(VERSION 3.14)
project(autoware_lanelet2_extension)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  lib/autoware_osm_parser.cpp
  lib/autoware_traffic_light.cpp
  lib/crosswalk.cpp
  lib/detection_area.cpp
  lib/landmark.cpp
  lib/no_parking_area.cpp
  lib/no_stopping_area.cpp
  lib/bus_stop_area.cpp
  lib/message_conversion.cpp
  lib/mgrs_projector.cpp
  lib/query.cpp
  lib/road_marking.cpp
  lib/speed_bump.cpp
  lib/transverse_mercator_projector.cpp
  lib/utilities.cpp
  lib/virtual_traffic_light.cpp
  lib/visualization.cpp
  lib/route_checker.cpp
)

ament_auto_add_executable(${PROJECT_NAME}_sample src/sample_code.cpp)
target_link_libraries(${PROJECT_NAME}_sample
  ${PROJECT_NAME}_lib
)

ament_auto_add_executable(autoware_lanelet2_validation src/validation.cpp)
target_link_libraries(autoware_lanelet2_validation
  ${PROJECT_NAME}_lib
)

ament_auto_add_executable(check_right_of_way src/check_right_of_way.cpp)
target_link_libraries(check_right_of_way
  ${PROJECT_NAME}_lib
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(message_conversion-test test/src/test_message_conversion.cpp)
  target_link_libraries(message_conversion-test ${PROJECT_NAME}_lib)
  ament_add_ros_isolated_gtest(projector-test test/src/test_projector.cpp)
  target_link_libraries(projector-test ${PROJECT_NAME}_lib)
  ament_add_ros_isolated_gtest(query-test test/src/test_query.cpp)
  target_link_libraries(query-test ${PROJECT_NAME}_lib)
  ament_add_ros_isolated_gtest(regulatory_elements-test test/src/test_regulatory_elements.cpp)
  target_link_libraries(regulatory_elements-test ${PROJECT_NAME}_lib)
  ament_add_ros_isolated_gtest(utilities-test test/src/test_utilities.cpp)
  target_link_libraries(utilities-test ${PROJECT_NAME}_lib)
  ament_add_ros_isolated_gtest(route-test test/src/test_route_checker.cpp)
  target_link_libraries(route-test ${PROJECT_NAME}_lib)
endif()

ament_auto_package()
