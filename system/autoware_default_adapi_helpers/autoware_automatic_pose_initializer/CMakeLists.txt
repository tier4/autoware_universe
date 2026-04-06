cmake_minimum_required(VERSION 3.14)
project(autoware_automatic_pose_initializer)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/automatic_pose_initializer.cpp
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::automatic_pose_initializer::AutomaticPoseInitializer"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

ament_auto_package(INSTALL_TO_SHARE launch)
