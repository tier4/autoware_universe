cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_time_overlap)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/footprints.cpp
  src/overlap.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_overlap.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_auto_package()
