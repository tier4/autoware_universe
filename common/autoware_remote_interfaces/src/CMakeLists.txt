### Target GenericMultiplexerNode ###
set(TARGETS ${TARGETS} GenericMultiplexerNode)
add_executable(GenericMultiplexerNode
    generic_multiplexer.cpp
    generic_multiplexer_node.cpp
)

ament_target_dependencies(GenericMultiplexerNode
    rclcpp
    autoware_adapi_v1_msgs
)

### Target ObjectListModifierNode ###
set(TARGETS ${TARGETS} ObjectListModifierNode)
add_executable(ObjectListModifierNode
    object_list_modifier.cpp
    object_list_modifier_node.cpp
)

ament_target_dependencies(ObjectListModifierNode
    rclcpp
    autoware_adapi_v1_msgs
    autoware_perception_msgs
)

### Target TrafficSignalsModifierNode ###
set(TARGETS ${TARGETS} TrafficSignalsModifierNode)
add_executable(TrafficSignalsModifierNode
    traffic_signals_modifier.cpp
    traffic_signals_modifier_node.cpp
)

ament_target_dependencies(TrafficSignalsModifierNode
    rclcpp
    autoware_adapi_v1_msgs
    autoware_perception_msgs
)

### Install all Targets ###
install(
    TARGETS ${TARGETS}
    DESTINATION lib/${PROJECT_NAME}
)