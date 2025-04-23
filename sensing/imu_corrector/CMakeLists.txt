cmake_minimum_required(VERSION 3.5)
project(imu_corrector)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Boost REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(gyro_bias_estimation_module SHARED
  src/gyro_bias_estimation_module.cpp
)

ament_auto_add_library(imu_corrector_node SHARED
  src/imu_corrector_core.cpp
)

ament_auto_add_library(gyro_bias_estimator_node SHARED
  src/gyro_bias_estimator.cpp
)



target_link_libraries(gyro_bias_estimator_node gyro_bias_estimation_module)
target_link_libraries(gyro_bias_estimation_module Boost::boost)

rclcpp_components_register_node(imu_corrector_node
  PLUGIN "imu_corrector::ImuCorrector"
  EXECUTABLE imu_corrector
)

rclcpp_components_register_node(gyro_bias_estimator_node
  PLUGIN "imu_corrector::GyroBiasEstimator"
  EXECUTABLE gyro_bias_estimator
)

function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})
  ament_add_gmock(${test_name} ${filepath})
  target_link_libraries("${test_name}" gyro_bias_estimation_module imu_corrector_node gyro_bias_estimator_node)
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_testcase(test/test_gyro_bias_estimator.cpp)
  add_testcase(test/test_imu_corrector.cpp)
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
