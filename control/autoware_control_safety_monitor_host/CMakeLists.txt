cmake_minimum_required(VERSION 3.14)
project(autoware_control_safety_monitor_host)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

ament_auto_add_library(${PROJECT_NAME}_plugins SHARED
  src/surround_collision_plugin.cpp
  src/trajectory_obstacle_collision_plugin.cpp
  src/trajectory_object_collision_plugin.cpp
)

target_include_directories(${PROJECT_NAME}_plugins SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_plugins
  ${PCL_LIBRARIES}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/safety_monitor_host.cpp
)

target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_plugins)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::control_safety_monitor_host::SafetyMonitorHost"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
