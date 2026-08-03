cmake_minimum_required(VERSION 3.14)
project(autoware_avoidance_target_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/object_filtering.cpp
  src/boundary.cpp
  src/parameter.cpp
)

if(BUILD_TESTING)
  ament_auto_add_gtest(test_update_objects_performance
    test/test_update_objects_performance.cpp
  )
  target_link_libraries(test_update_objects_performance
    ${PROJECT_NAME}
  )
endif()

ament_auto_package()
