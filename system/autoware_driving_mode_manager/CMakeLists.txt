cmake_minimum_required(VERSION 3.8)
project(autoware_driving_mode_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/DebugModeRequest.msg"
  "msg/DebugModeFlags.msg"
  "msg/DebugModeFlagsItem.msg"
  DEPENDENCIES builtin_interfaces
)
rosidl_get_typesupport_target(
  rosidl_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

set(dmm_target_dependencies
  autoware_adapi_v1_msgs
  autoware_planning_msgs
  autoware_system_msgs
  autoware_utils_diagnostics
  autoware_vehicle_msgs
  diagnostic_updater
  pluginlib
  rclcpp
  rclcpp_components
  tier4_system_msgs
)

# NOTE:
# Keep explicit library/export wiring here instead of relying only on
# ament_auto_add_library/ament_auto_package defaults.
# In this package (rosidl + pluginlib), the auto-only setup can publish
# incomplete include propagation for downstream plugins in clean builds.
# We explicitly export our own library targets and include interfaces so
# consumers can resolve <autoware_driving_mode_manager/...> reliably.
#
# Failure mode seen without this wiring:
# - Downstream plugin includes: <autoware_driving_mode_manager/plugin.hpp>
# - Propagated include base: .../install/autoware_driving_mode_manager/include/autoware_driving_mode_manager
# - Compiler then probes:
#   .../include/autoware_driving_mode_manager/autoware_driving_mode_manager/plugin.hpp
#   (extra nested directory)
# - Actual header path is:
#   .../include/autoware_driving_mode_manager/plugin.hpp
# This mismatch causes "No such file or directory" in downstream plugin builds.

add_library(${PROJECT_NAME}_common SHARED
  "src/core/config.cpp"
  "src/core/status.cpp"
  "src/core/task.cpp"
  "src/core/init.cpp"
  "src/core/main.cpp"
  "src/core/debug.cpp"
  "src/ros_interface.cpp"
  "src/driving_mode_manager.cpp"
)
target_include_directories(${PROJECT_NAME}_common
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  PRIVATE
    "src"
)
target_link_libraries(${PROJECT_NAME}_common
  "${rosidl_typesupport_target}"
)
ament_target_dependencies(${PROJECT_NAME}_common SYSTEM ${dmm_target_dependencies})

add_library(${PROJECT_NAME}_plugin SHARED
  "src/plugin/default.cpp"
)
target_include_directories(${PROJECT_NAME}_plugin
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  PRIVATE
    "src"
)
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME}_common)
ament_target_dependencies(${PROJECT_NAME}_plugin SYSTEM ${dmm_target_dependencies})

rclcpp_components_register_node(${PROJECT_NAME}_common
  PLUGIN "autoware::driving_mode_manager::DrivingModeManager"
  EXECUTABLE "driving_mode_manager_node"
)

if(BUILD_TESTING)
  ament_auto_add_gtest(test_${PROJECT_NAME}
    "test/util/mock.cpp"
    "test/util/util.cpp"
    "test/test.cpp"
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}_common
    ${PROJECT_NAME}_plugin
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    "src"
  )
endif()

pluginlib_export_plugin_description_file(autoware_driving_mode_manager plugins.xml)

install(
  TARGETS ${PROJECT_NAME}_common ${PROJECT_NAME}_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# Keep both exports: modern consumers use targets, legacy consumers may use *_LIBRARIES.
ament_export_libraries(${PROJECT_NAME}_common ${PROJECT_NAME}_plugin)

ament_export_include_directories(include)
ament_auto_package(INSTALL_TO_SHARE config launch)
