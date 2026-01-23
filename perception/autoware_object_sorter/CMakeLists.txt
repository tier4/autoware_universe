cmake_minimum_required(VERSION 3.14)
project(autoware_object_sorter)

# Dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(agnocastlib REQUIRED)
find_package(agnocast_components REQUIRED)

# Targets
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/object_sorter_base.cpp
  src/detected_object_sorter_node.cpp
  src/tracked_object_sorter_node.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${agnocastlib_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  agnocastlib
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::object_sorter::DetectedObjectSorterNode"
  EXECUTABLE detected_object_sorter_node
  EXECUTOR AgnocastOnlySingleThreadedExecutor
)

agnocast_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::object_sorter::TrackedObjectSorterNode"
  EXECUTABLE tracked_object_sorter_node
  EXECUTOR AgnocastOnlySingleThreadedExecutor
)

# Tests
if(BUILD_TESTING)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
