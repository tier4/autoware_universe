cmake_minimum_required(VERSION 3.14)
project(autoware_in_lane_mrm_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)

generate_parameter_library(in_lane_mrm_planner_parameters
  param/in_lane_mrm_planner_parameters.yaml
)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/in_lane_mrm_planner_node.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  in_lane_mrm_planner_parameters
)

target_compile_options(${PROJECT_NAME}_lib PRIVATE -Wno-deprecated-declarations)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "autoware::in_lane_mrm_planner::InLaneMrmPlannerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
    schema
)
