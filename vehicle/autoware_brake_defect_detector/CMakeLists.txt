cmake_minimum_required(VERSION 3.14)
project(autoware_brake_defect_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/brake_defect_detector.cpp
  src/brake_defect_detector_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::brake_defect_detector::BrakeDefectDetectorNode"
  EXECUTABLE brake_defect_detector_node
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
  USE_SCOPED_HEADER_INSTALL_DIR
)
