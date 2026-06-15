cmake_minimum_required(VERSION 3.14)
project(autoware_target_lanelet_estimator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/node.cpp
  src/impl.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "autoware::target_lanelet_estimator::TargetLaneletEstimatorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package()
