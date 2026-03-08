cmake_minimum_required(VERSION 3.14)
project(autoware_scenario_selector)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(autoware_agnocast_wrapper REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${autoware_agnocast_wrapper_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  autoware_agnocast_wrapper
)

autoware_agnocast_wrapper_setup(${PROJECT_NAME})

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::scenario_selector::ScenarioSelectorNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# TODO: Re-enable tests after updating to work with autoware::agnocast_wrapper::Node
# if(BUILD_TESTING)
#   ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
#     test/test_${PROJECT_NAME}_node_interface.cpp
#   )
#   target_link_libraries(test_${PROJECT_NAME}
#     ${PROJECT_NAME}
#   )
# endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
