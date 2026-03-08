# Copyright 2025 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Register an autoware node component with runtime switching support
# between rclcpp::Node and agnocast::Node.
#
# This macro creates:
# 1. A standard rclcpp_components registration for the component
# 2. A standalone executable that can switch between rclcpp::Node and agnocast::Node
#    at runtime based on the ENABLE_AGNOCAST environment variable
#
# usage: autoware_agnocast_wrapper_register_node(
#        <target> PLUGIN <component> EXECUTABLE <node>)
#
# :param target: the shared library target
# :type target: string
# :param PLUGIN: the plugin name (fully qualified class name)
# :type PLUGIN: string
# :param EXECUTABLE: the node's executable name
# :type EXECUTABLE: string
#
# Example:
#   autoware_agnocast_wrapper_register_node(my_node_component
#     PLUGIN "my_package::MyNode"
#     EXECUTABLE my_node
#   )
#
macro(autoware_agnocast_wrapper_register_node target)
  cmake_parse_arguments(ARGS "" "PLUGIN;EXECUTABLE" "" ${ARGN})

  if(ARGS_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "autoware_agnocast_wrapper_register_node() called with unused "
      "arguments: ${ARGS_UNPARSED_ARGUMENTS}")
  endif()

  if("${ARGS_PLUGIN}" STREQUAL "")
    message(FATAL_ERROR
      "autoware_agnocast_wrapper_register_node macro requires a PLUGIN argument for target ${target}")
  endif()

  if("${ARGS_EXECUTABLE}" STREQUAL "")
    message(FATAL_ERROR
      "autoware_agnocast_wrapper_register_node macro requires an EXECUTABLE argument for target ${target}")
  endif()

  # Save the values with unique prefix to avoid collision with rclcpp_components_register_node
  set(_AGNOCAST_WRAPPER_COMPONENT ${ARGS_PLUGIN})
  set(_AGNOCAST_WRAPPER_NODE ${ARGS_EXECUTABLE})

  # Register with rclcpp_components for standard component container support
  # Note: This call will overwrite 'node', 'component', 'library_name' variables
  rclcpp_components_register_node(${target}
    PLUGIN ${_AGNOCAST_WRAPPER_COMPONENT}
    EXECUTABLE ${_AGNOCAST_WRAPPER_NODE}_component)

  # Set template variables after rclcpp_components_register_node call
  # These are used by configure_file for @VAR@ substitutions
  set(node ${_AGNOCAST_WRAPPER_NODE})
  set(component ${_AGNOCAST_WRAPPER_COMPONENT})
  set(library_name "$<TARGET_FILE_NAME:${target}>")

  # Generate runtime-switchable node main from template
  configure_file(
    ${autoware_agnocast_wrapper_NODE_TEMPLATE}
    ${PROJECT_BINARY_DIR}/autoware_agnocast_wrapper/node_main_configured_${_AGNOCAST_WRAPPER_NODE}.cpp.in)
  file(GENERATE
    OUTPUT ${PROJECT_BINARY_DIR}/autoware_agnocast_wrapper/node_main_${_AGNOCAST_WRAPPER_NODE}.cpp
    INPUT ${PROJECT_BINARY_DIR}/autoware_agnocast_wrapper/node_main_configured_${_AGNOCAST_WRAPPER_NODE}.cpp.in)

  # Create runtime-switchable executable
  add_executable(${_AGNOCAST_WRAPPER_NODE}
    ${PROJECT_BINARY_DIR}/autoware_agnocast_wrapper/node_main_${_AGNOCAST_WRAPPER_NODE}.cpp)

  target_link_libraries(${_AGNOCAST_WRAPPER_NODE}
    ${target})

  ament_target_dependencies(${_AGNOCAST_WRAPPER_NODE}
    rclcpp
    rclcpp_components
    class_loader
    agnocastlib
    autoware_agnocast_wrapper)

  # Apply agnocast wrapper setup (adds USE_AGNOCAST_ENABLED if ENABLE_AGNOCAST=1)
  autoware_agnocast_wrapper_setup(${_AGNOCAST_WRAPPER_NODE})

  # Install executable
  install(TARGETS ${_AGNOCAST_WRAPPER_NODE}
    DESTINATION lib/${PROJECT_NAME})
endmacro()
