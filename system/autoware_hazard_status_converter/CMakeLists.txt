cmake_minimum_required(VERSION 3.14)
project(autoware_hazard_status_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/converter.cpp
)

ament_target_dependencies(${PROJECT_NAME} agnocastlib)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::hazard_status_converter::Converter"
  EXECUTABLE ${PROJECT_NAME}_node
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::hazard_status_converter::Converter"
  EXECUTABLE ${PROJECT_NAME}_agnocast_node
  EXECUTOR SingleThreadedAgnocastExecutor
  RESOURCE_INDEX agnocast_components
)

ament_auto_package(INSTALL_TO_SHARE launch)
