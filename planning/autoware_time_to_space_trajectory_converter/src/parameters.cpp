// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "parameters.hpp"

#include <autoware_utils_rclcpp/parameter.hpp>

#include <fmt/format.h>

#include <vector>

namespace autoware::time_to_space_trajectory_converter
{
NodeParam init_node_param(rclcpp::Node & node)
{
  NodeParam node_param;

  node_param.update_rate_hz =
    autoware_utils_rclcpp::get_or_declare_parameter<double>(node, "update_rate_hz");
  node_param.resampling_resolution_m =
    autoware_utils_rclcpp::get_or_declare_parameter<double>(node, "resampling_resolution_m");
  node_param.recompute_acceleration =
    autoware_utils_rclcpp::get_or_declare_parameter<bool>(node, "recompute_acceleration");

  node_param.spatial.max_knot_yaw_diff_rad =
    autoware_utils_rclcpp::get_or_declare_parameter<double>(node, "spatial.max_knot_yaw_diff_rad");
  node_param.spatial.min_knot_dist_m =
    autoware_utils_rclcpp::get_or_declare_parameter<double>(node, "spatial.min_knot_dist_m");
  node_param.spatial.th_stop_velocity_mps =
    autoware_utils_rclcpp::get_or_declare_parameter<double>(node, "spatial.th_stop_velocity_mps");

  return node_param;
}

rcl_interfaces::msg::SetParametersResult update_node_param(
  const std::vector<rclcpp::Parameter> & params, NodeParam & node_param)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // Note: The following parameter in not re-configurable.
  // 1. update_rate_hz

  try {
    autoware_utils_rclcpp::update_param<double>(
      params, "resampling_resolution_m", node_param.resampling_resolution_m);
    autoware_utils_rclcpp::update_param<bool>(
      params, "recompute_acceleration", node_param.recompute_acceleration);

    autoware_utils_rclcpp::update_param<double>(
      params, "spatial.max_knot_yaw_diff_rad", node_param.spatial.max_knot_yaw_diff_rad);
    autoware_utils_rclcpp::update_param<double>(
      params, "spatial.min_knot_dist_m", node_param.spatial.min_knot_dist_m);
    autoware_utils_rclcpp::update_param<double>(
      params, "spatial.th_stop_velocity_mps", node_param.spatial.th_stop_velocity_mps);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}
}  // namespace autoware::time_to_space_trajectory_converter
