cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_safety_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_trajectory_safety_filter plugins.xml)

generate_parameter_library(${PROJECT_NAME}_param
  param/parameter_struct.yaml
)

include_directories(
  SYSTEM
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/trajectory_safety_filter_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_param
)

target_compile_options(${PROJECT_NAME} PUBLIC
  -Wno-error=deprecated-declarations
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_safety_filter::TrajectorySafetyFilter"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_add_library(autoware_trajectory_safety_filter_plugins SHARED
  src/filters/out_of_lane_filter.cpp
  src/filters/collision_filter.cpp
  src/filters/collision_check_filter.cpp
)

target_link_libraries(autoware_trajectory_safety_filter_plugins
  ${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_constant_curvature
    test/collision_check_filter/test_constant_curvature_predictor.cpp
  )
  target_link_libraries(test_constant_curvature gtest_main ${PROJECT_NAME})
  target_include_directories(test_constant_curvature PRIVATE src)

  ament_add_ros_isolated_gtest(test_motion
    test/collision_check_filter/test_motion.cpp
  )
  target_link_libraries(test_motion gtest_main ${PROJECT_NAME})
  target_include_directories(test_motion PRIVATE src)

  ament_add_ros_isolated_gtest(test_polygon
    test/collision_check_filter/test_polygon.cpp
  )
  target_link_libraries(test_polygon gtest_main ${PROJECT_NAME})
  target_include_directories(test_polygon PRIVATE src)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
