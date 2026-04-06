cmake_minimum_required(VERSION 3.14)
project(autoware_default_adapi_universe)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/diagnostics.cpp
  src/fail_safe.cpp
  src/heartbeat.cpp
  src/manual_control.cpp
  src/motion.cpp
  src/mrm_request.cpp
  src/operation_mode.cpp
  src/perception.cpp
  src/planning.cpp
  src/vehicle_status.cpp
  src/vehicle_command.cpp
  src/vehicle_metrics.cpp
  src/vehicle_info.cpp
  src/vehicle_door.cpp
  src/utils/command_conversion.cpp
  src/compatibility/autoware_state.cpp
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::AutowareStateNode"
  EXECUTABLE autoware_state_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::DiagnosticsNode"
  EXECUTABLE diagnostics_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::FailSafeNode"
  EXECUTABLE fail_safe_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::HeartbeatNode"
  EXECUTABLE heartbeat_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::ManualControlNode"
  EXECUTABLE manual_control_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::MotionNode"
  EXECUTABLE motion_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::MrmRequestNode"
  EXECUTABLE mrm_request_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::OperationModeNode"
  EXECUTABLE operation_mode_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::PerceptionNode"
  EXECUTABLE perception_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::PlanningNode"
  EXECUTABLE planning_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::VehicleStatusNode"
  EXECUTABLE vehicle_status_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::VehicleCommandNode"
  EXECUTABLE vehicle_command_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::VehicleMetricsNode"
  EXECUTABLE vehicle_metrics_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::VehicleInfoNode"
  EXECUTABLE vehicle_info_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::VehicleDoorNode"
  EXECUTABLE vehicle_door_node
  EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  RESOURCE_INDEX agnocast_components
)

if(BUILD_TESTING)
endif()

install(
  PROGRAMS script/web_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE config launch)
