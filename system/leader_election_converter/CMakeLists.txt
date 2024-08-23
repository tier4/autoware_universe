cmake_minimum_required(VERSION 3.14)
project(leader_election_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(common_converter SHARED
  src/common/converter/availability_converter.cpp
  src/common/converter/mrm_converter.cpp
  src/common/converter/log_converter.cpp
)

target_include_directories(common_converter PRIVATE
  src/common/converter
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node/leader_election_converter.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE src/common/converter)

target_link_libraries(${PROJECT_NAME} common_converter)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "leader_election_converter::LeaderElectionConverter"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

install(PROGRAMS
  script/relay_to_sub.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

ament_auto_package(INSTALL_TO_SHARE config launch)
