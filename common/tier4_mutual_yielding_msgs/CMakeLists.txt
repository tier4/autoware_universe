cmake_minimum_required(VERSION 3.14)
project(tier4_mutual_yielding_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(msg_dependencies
  builtin_interfaces)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CreepTrigger.msg"
  DEPENDENCIES ${msg_dependencies}
  ADD_LINTER_TESTS
)

ament_auto_package()
