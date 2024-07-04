cmake_minimum_required(VERSION 3.14)
project(control_cmd_switcher)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/control_cmd_switcher/control_cmd_switcher.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ControlCmdSwitcher"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(PROGRAMS
  tool/relay_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
