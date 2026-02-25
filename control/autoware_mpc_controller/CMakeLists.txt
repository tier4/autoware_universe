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

# acados_mpc (code generation + acados_interface library); requires ACADOS_SOURCE_DIR and acados installed
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/acados_mpc)

# Node that uses the MPC interface (replacement for longitudinal + lateral MPC)
add_executable(mpc_controller_node src/node.cpp)
target_link_libraries(mpc_controller_node acados_interface)
ament_target_dependencies(mpc_controller_node
  rclcpp
  autoware_control_msgs
  autoware_motion_utils
  autoware_planning_msgs
  autoware_utils_geometry
  geometry_msgs
  nav_msgs
  tf2
  autoware_vehicle_msgs
)
# Codegen is triggered when acados_interface is built (generated files are its sources)
install(TARGETS mpc_controller_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
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

ament_package()
