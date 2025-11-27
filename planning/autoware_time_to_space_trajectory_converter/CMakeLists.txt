cmake_minimum_required(VERSION 3.14)
project(autoware_time_to_space_trajectory_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::time_to_space_trajectory_converter::TimeToSpaceConverterNode"
  EXECUTABLE time_to_space_trajectory_converter
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(autoware_pyplot REQUIRED)
  option(EXPORT_TEST_PLOT_FIGURE "Enable plotting in tests" ON)

  file(GLOB_RECURSE TEST_SOURCES test/*.cpp)

  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    ${TEST_SOURCES}
    TIMEOUT 120
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${${PROJECT_NAME}_LIBRARIES}
    fmt::fmt
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  )

  ament_target_dependencies(test_${PROJECT_NAME} autoware_pyplot)

  if(EXPORT_TEST_PLOT_FIGURE)
    target_compile_definitions(test_${PROJECT_NAME} PRIVATE EXPORT_TEST_PLOT_FIGURE)

    add_custom_target(create_test_directories ALL
      COMMAND ${CMAKE_COMMAND} -E make_directory
      ${CMAKE_CURRENT_SOURCE_DIR}/test_results/test_spatial_preprocessor
      COMMAND ${CMAKE_COMMAND} -E make_directory
      ${CMAKE_CURRENT_SOURCE_DIR}/test_results/test_hermite_spline
      COMMAND ${CMAKE_COMMAND} -E make_directory
      ${CMAKE_CURRENT_SOURCE_DIR}/test_results/test_stop_state_machine
      COMMAND ${CMAKE_COMMAND} -E make_directory
      ${CMAKE_CURRENT_SOURCE_DIR}/test_results/test_time_to_space_converter_node
    )

    add_dependencies(test_${PROJECT_NAME} create_test_directories)
  endif()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
