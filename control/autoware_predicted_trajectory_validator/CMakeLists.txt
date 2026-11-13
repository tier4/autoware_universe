cmake_minimum_required(VERSION 3.22)
project(autoware_predicted_trajectory_validator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common filters segmentation search)

generate_parameter_library(predicted_trajectory_validator_parameters
  param/predicted_trajectory_validator_parameters.yaml
)

ament_auto_add_library(autoware_predicted_trajectory_validator_helpers SHARED
  src/utils.cpp
  src/debug_marker.cpp
)

ament_auto_add_library(autoware_predicted_trajectory_validator_component SHARED
  include/autoware/predicted_trajectory_validator/predicted_trajectory_validator.hpp
  src/predicted_trajectory_validator.cpp
  src/uncrossable_bound_departure_validator.cpp
  src/collision_validator/collision_validator.cpp
  src/collision_validator/collision_evaluator.cpp
  src/collision_validator/pointcloud_preprocessor.cpp
  src/hazard_warning/hazard_warning.cpp
)
target_include_directories(autoware_predicted_trajectory_validator_component
  SYSTEM PUBLIC ${PCL_INCLUDE_DIRS}
)
target_link_libraries(autoware_predicted_trajectory_validator_component
  autoware_predicted_trajectory_validator_helpers
  predicted_trajectory_validator_parameters
  ${PCL_LIBRARIES}
)
# generate_parameter_library pulls in parameter_traits, which uses deprecated symbols.
target_compile_options(autoware_predicted_trajectory_validator_component PUBLIC
  -Wno-error=deprecated-declarations
)
rclcpp_components_register_node(autoware_predicted_trajectory_validator_component
  PLUGIN "autoware::predicted_trajectory_validator::PredictedTrajectoryValidator"
  EXECUTABLE autoware_predicted_trajectory_validator_node
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/ValidationResult.msg"
  "msg/PredictedTrajectoryValidatorStatus.msg"
  DEPENDENCIES builtin_interfaces
)

# to use a message defined in the same package
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
  rosidl_target_interfaces(autoware_predicted_trajectory_validator_component
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(
    cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(autoware_predicted_trajectory_validator_component "${cpp_typesupport_target}")
endif()

if(BUILD_TESTING)
  file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
  ament_add_gtest(test_autoware_predicted_trajectory_validator
    ${TEST_SOURCES}
  )
  ament_target_dependencies(test_autoware_predicted_trajectory_validator
    ament_index_cpp
  )
  target_link_libraries(test_autoware_predicted_trajectory_validator
    autoware_predicted_trajectory_validator_component
    predicted_trajectory_validator_parameters
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
