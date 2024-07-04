cmake_minimum_required(VERSION 3.14)
project(mrm_stop_operator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(mrm_stop_operator SHARED
  src/mrm_stop_operator.cpp
)
ament_target_dependencies(mrm_stop_operator)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "mrm_stop_operator::MrmStopOperator"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
