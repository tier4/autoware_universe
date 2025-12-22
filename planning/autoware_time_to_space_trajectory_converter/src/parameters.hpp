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

#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

#include <rclcpp/node.hpp>

#include <vector>

namespace autoware::time_to_space_trajectory_converter
{
/**
 * @brief Configuration parameters for spatial resampling logic.
 */
struct SpatialPreprocessorConfig
{
  double min_knot_dist_m = 0.1;       ///< Minimum distance required to create a new spline knot.
  double th_stop_velocity_mps = 0.1;  ///< Velocity threshold below which ego is considered stopped.
  double max_knot_yaw_diff_rad = 0.05;
  ///< If the car turns more than this(0.1 rad ~ = 5.7 degrees), force a knot
};

struct NodeParam
{
  double update_rate_hz{50.0};
  double resampling_resolution_m = 1.0;
  bool recompute_acceleration = true;
  SpatialPreprocessorConfig spatial;
};

NodeParam init_node_param(rclcpp::Node & node);
rcl_interfaces::msg::SetParametersResult update_node_param(
  const std::vector<rclcpp::Parameter> & params, NodeParam & node_param);

}  // namespace autoware::time_to_space_trajectory_converter

#endif  // PARAMETERS_HPP_
