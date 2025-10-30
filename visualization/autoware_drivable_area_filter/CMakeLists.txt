cmake_minimum_required(VERSION 3.14)
project(drivable_area_visual_filter)

find_package(autoware_cmake REQUIRED)

autoware_package()


add_executable(drivable_area_visual_filter_node src/drivable_area_visual_filter_node.cpp)

ament_target_dependencies(drivable_area_visual_filter_node
  rclcpp
  rclcpp_components
  geometry_msgs
  autoware_perception_msgs
  autoware_map_msgs
  lanelet2_core
  autoware_lanelet2_extension
  autoware_adapi_v1_msgs
)

install(TARGETS
  drivable_area_visual_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
