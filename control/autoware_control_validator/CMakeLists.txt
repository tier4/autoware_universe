cmake_minimum_required(VERSION 3.22)
project(autoware_control_validator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common filters segmentation search)

generate_parameter_library(control_validator_parameters
  param/control_validator_parameters.yaml
)

ament_auto_add_library(autoware_control_validator_helpers SHARED
  src/utils.cpp
  src/debug_marker.cpp
)

# control validator
ament_auto_add_library(autoware_control_validator_component SHARED
  include/autoware/control_validator/control_validator.hpp
  src/control_validator.cpp
  src/collision_validator/collision_validator.cpp
  src/collision_validator/collision_evaluator.cpp
  src/collision_validator/pointcloud_preprocessor.cpp
  src/hazard_warning/hazard_warning.cpp
)
target_include_directories(autoware_control_validator_component
  SYSTEM PUBLIC ${PCL_INCLUDE_DIRS}
)
target_link_libraries(autoware_control_validator_component
  autoware_control_validator_helpers
  control_validator_parameters
  ${PCL_LIBRARIES}
)
# generate_parameter_library pulls in parameter_traits, which uses deprecated symbols.
target_compile_options(autoware_control_validator_component PUBLIC
  -Wno-error=deprecated-declarations
)
rclcpp_components_register_node(autoware_control_validator_component
  PLUGIN "autoware::control_validator::ControlValidator"
  EXECUTABLE autoware_control_validator_node
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/ControlValidatorStatus.msg"
  "msg/HazardAlert.msg"
  "msg/HazardSector.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# to use a message defined in the same package
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
  rosidl_target_interfaces(autoware_control_validator_component
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(
    cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(autoware_control_validator_component "${cpp_typesupport_target}")
endif()

if(BUILD_TESTING)
  file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
  ament_add_gtest(test_autoware_control_validator
    ${TEST_SOURCES}
  )
  ament_target_dependencies(test_autoware_control_validator
    ament_index_cpp
  )
  target_link_libraries(test_autoware_control_validator
    autoware_control_validator_component
    control_validator_parameters
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
