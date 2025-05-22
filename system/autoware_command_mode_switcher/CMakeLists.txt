cmake_minimum_required(VERSION 3.8)
project(autoware_command_mode_switcher)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_common SHARED
  "src/common/command_container.cpp"
  "src/common/command_plugin.cpp"
  "src/common/command_status.cpp"
  "src/common/selector_interface.cpp"
  "src/switcher/manual.cpp"
  "src/command_mode_switcher.cpp"
)
target_include_directories(${PROJECT_NAME}_common PRIVATE "src")

rclcpp_components_register_node(${PROJECT_NAME}_common
  PLUGIN "autoware::command_mode_switcher::CommandModeSwitcher"
  EXECUTABLE "command_mode_switcher_node"
  EXECUTOR MultiThreadedExecutor
)

ament_auto_add_library(${PROJECT_NAME}_tools SHARED
  "src/availability_converter.cpp"
)
rclcpp_components_register_node(${PROJECT_NAME}_tools
  PLUGIN "autoware::command_mode_switcher::AvailabilityConverter"
  EXECUTABLE "availability_converter_node"
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
