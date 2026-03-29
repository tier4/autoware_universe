cmake_minimum_required(VERSION 3.14)
project(autoware_topic_state_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/topic_state_monitor.cpp
  src/topic_state_monitor_core.cpp
  src/transform_state_monitor_core.cpp
  src/typed_topic_state_monitor_core.cpp
)

ament_target_dependencies(${PROJECT_NAME} agnocastlib)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::topic_state_monitor::TopicStateMonitorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::topic_state_monitor::TopicStateMonitorNode"
  EXECUTABLE ${PROJECT_NAME}_agnocast_node
  EXECUTOR SingleThreadedAgnocastExecutor
  RESOURCE_INDEX agnocast_components
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::topic_state_monitor::TransformStateMonitorNode"
  EXECUTABLE transform_state_monitor_agnocast_node
  EXECUTOR AgnocastOnlySingleThreadedExecutor
  RESOURCE_INDEX agnocast_components
)

# Typed variant: replaces GenericSubscription with a compile-time type registry.
# See typed_topic_state_monitor_core.hpp for design rationale.
agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::topic_state_monitor::TypedTopicStateMonitorNode"
  EXECUTABLE typed_topic_state_monitor_agnocast_node
  EXECUTOR AgnocastOnlySingleThreadedExecutor
  RESOURCE_INDEX agnocast_components
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
