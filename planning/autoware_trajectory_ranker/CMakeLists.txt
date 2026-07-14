cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_ranker)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_trajectory_ranker plugins.xml)

generate_parameter_library(${PROJECT_NAME}_param
  param/trajectory_ranker_parameter_struct.yaml
)

include_directories(
  SYSTEM
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/trajectory_ranker_node.cpp
  src/evaluation.cpp
  src/utils.cpp
)

ament_auto_add_library(simple_trajectory_ranker_node SHARED
  src/simple_trajectory_ranker_node.cpp
)

set(trajectory_ranker_lib_target ${PROJECT_NAME}_lib)

ament_auto_add_library(${trajectory_ranker_lib_target} SHARED
  src/trajectory_ranker_wrapper.cpp
)

target_link_libraries(${trajectory_ranker_lib_target}
  ${PROJECT_NAME}_param
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_param
)

target_compile_options(${PROJECT_NAME} PUBLIC
  -Wno-error=deprecated-declarations
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::trajectory_ranker::TrajectoryRanker"
  EXECUTABLE ${PROJECT_NAME}_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor
)

autoware_agnocast_wrapper_register_node(simple_trajectory_ranker_node
  PLUGIN "autoware::trajectory_ranker::SimpleTrajectoryRanker"
  EXECUTABLE simple_trajectory_ranker
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor
)

ament_auto_add_library(${PROJECT_NAME}_plugins SHARED
  src/metrics/lateral_acceleration_metric.cpp
  src/metrics/longitudinal_jerk_metric.cpp
  src/metrics/distance_metric.cpp
  src/metrics/lateral_deviation_metric.cpp
  src/metrics/steering_consistency_metric.cpp
  src/metrics/trajectory_consistency_metric.cpp
  src/metrics/metrics_utils.cpp
)

target_link_libraries(${PROJECT_NAME}_plugins
  ${PROJECT_NAME}
  ${trajectory_ranker_lib_target}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_utils
    test/test_utils.cpp
  )
  target_link_libraries(test_utils
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_metrics_utils
    test/test_metrics_utils.cpp
  )
  target_link_libraries(test_metrics_utils
    ${PROJECT_NAME}_plugins
  )

  ament_add_gtest(test_evaluator
    test/test_evaluator.cpp
  )
  target_link_libraries(test_evaluator
    ${PROJECT_NAME}
    ${PROJECT_NAME}_plugins
  )

  ament_add_gtest(test_metrics
    test/test_metrics.cpp
  )
  target_link_libraries(test_metrics
    ${PROJECT_NAME}
    ${PROJECT_NAME}_plugins
  )

  ament_add_gtest(test_simple_trajectory_ranker_node
    test/simple_trajectory_ranker_node_test.cpp
  )
  target_link_libraries(test_simple_trajectory_ranker_node
    simple_trajectory_ranker_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
