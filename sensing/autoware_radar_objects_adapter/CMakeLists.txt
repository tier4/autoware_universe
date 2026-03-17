cmake_minimum_required(VERSION 3.14)
project(autoware_radar_objects_adapter)

find_package(autoware_cmake REQUIRED)
find_package(agnocast_components REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/radar_objects_adapter.cpp
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::RadarObjectsAdapter"
  EXECUTABLE radar_objects_adapter_node
  EXECUTOR CallbackIsolatedAgnocastExecutor
)

ament_auto_package(INSTALL_TO_SHARE launch config)
