cmake_minimum_required(VERSION 3.14)
project(autoware_mpc_controller)

find_package(ament_cmake REQUIRED)

# Install model generator scripts for use with acados or symbolic tooling
install(
  DIRECTORY generators
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
