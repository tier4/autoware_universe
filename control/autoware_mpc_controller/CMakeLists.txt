cmake_minimum_required(VERSION 3.14)
project(autoware_mpc_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(acados REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_motion_utils REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_utils_geometry REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(autoware_trajectory_follower_base REQUIRED)

# acados_mpc (code generation + acados_longitudinal_interface)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/acados_mpc)

# Longitudinal-only MPC controller library (for trajectory_follower_node)
add_library(longitudinal_mpc_controller SHARED src/longitudinal_mpc_controller.cpp)
target_include_directories(longitudinal_mpc_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(longitudinal_mpc_controller PUBLIC acados_longitudinal_interface)
ament_target_dependencies(longitudinal_mpc_controller PUBLIC
  rclcpp
  autoware_control_msgs
  autoware_motion_utils
  autoware_planning_msgs
  autoware_trajectory_follower_base
  geometry_msgs
)

# Export libs to lib/ so ament_export_libraries find_library() can find them
install(TARGETS
  acados_longitudinal_interface
  longitudinal_mpc_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/visualize_mpc_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install model generator scripts for use with acados or symbolic tooling
install(
  DIRECTORY generators
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_export_include_directories(include)
ament_export_libraries(longitudinal_mpc_controller acados_longitudinal_interface)

ament_package()
