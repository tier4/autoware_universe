cmake_minimum_required(VERSION 3.14)
project(autoware_mrm_handler)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/mrm_handler/mrm_handler_core.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::mrm_handler::MrmHandler"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::mrm_handler::MrmHandler"
  EXECUTABLE ${PROJECT_NAME}_agnocast_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
