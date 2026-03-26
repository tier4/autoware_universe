cmake_minimum_required(VERSION 3.14)
project(autoware_component_state_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME} agnocastlib)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::component_state_monitor::StateMonitor"
  EXECUTABLE ${PROJECT_NAME}_node
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::component_state_monitor::StateMonitor"
  EXECUTABLE ${PROJECT_NAME}_agnocast_node
  EXECUTOR AgnocastOnlySingleThreadedExecutor
  RESOURCE_INDEX agnocast_components
)

ament_auto_package(INSTALL_TO_SHARE config launch)
