cmake_minimum_required(VERSION 3.14)
project(autoware_topic_relay_controller)

find_package(autoware_cmake REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/topic_relay_controller_node.cpp
)

ament_target_dependencies(${PROJECT_NAME} agnocastlib)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::topic_relay_controller::TopicRelayController"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR CallbackIsolatedAgnocastExecutor
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
