cmake_minimum_required(VERSION 3.5)
project(autoware_frenetix_core_vendor)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(FRENETIX_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../../external/tum_frenetix_core)

set(FRENETIX_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages)

add_custom_target(install_frenetix ALL
  COMMAND ${Python3_EXECUTABLE} -m pip install
          --no-cache-dir --force-reinstall
          --target=${FRENETIX_INSTALL_DIR}
          ${FRENETIX_SRC}
  COMMENT "Installing Frenetix into ROS2 workspace overlay"
)

ament_package()
