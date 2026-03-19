cmake_minimum_required(VERSION 3.14)
project(autoware_selection_json_export_rviz_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME} SHARED
  src/selection_json_export_panel.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
ament_target_dependencies(${PROJECT_NAME}
  rviz_common
  pluginlib
)
target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
)

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY plugins/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
