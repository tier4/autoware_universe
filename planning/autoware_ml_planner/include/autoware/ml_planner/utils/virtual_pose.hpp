// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__ML_PLANNER__UTILS__VIRTUAL_POSE_HPP_
#define AUTOWARE__ML_PLANNER__UTILS__VIRTUAL_POSE_HPP_

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace autoware::ml_planner::utils
{

/**
 * @brief Parameters of the virtual ego pose.
 *
 * When enabled, the model is given the point of its own previous trajectory closest to the
 * vehicle instead of the measured pose, so it plans as if its last trajectory had been followed
 * exactly. When the vehicle is farther than the limits from that point, the virtual pose is reset
 * onto the vehicle for that cycle.
 */
struct VirtualPoseParams
{
  bool enable;
  double max_position_error_m;
  double max_yaw_error_deg;
  // Leading segments of the previous trajectory searched for the closest point.
  int64_t max_search_segment_count;
  // Arc length on each side of the closest point over which the spline tangent is averaged.
  double yaw_fit_half_window_m;
  // Below this window length the measured heading is used.
  double yaw_fit_min_length_m;
  // Virtual poses of earlier cycles prepended to the previous trajectory so the spline extends
  // behind the vehicle.
  int64_t history_prefix_count;
};

struct VirtualPoseResult
{
  // Pose handed to the model: the virtual pose, or the measured pose when not snapped or reset.
  geometry_msgs::msg::Pose pose;
  // A closest point on the previous trajectory was found.
  bool snapped;
  // The closest point was beyond a limit, so the measured pose is used.
  bool reset;
  // Distance and heading difference between the measured pose and the closest point.
  double position_error_m;
  double yaw_error_deg;
};

/**
 * @brief Computes the pose handed to the model.
 *
 * @param measured_pose Measured ego pose (map frame).
 * @param polyline Previous trajectory in the map frame: prefix_count earlier virtual poses (oldest
 *        first), then the previous planning start pose, then the previous ego prediction.
 * @param prefix_count Number of leading earlier poses in polyline; they extend the spline backwards
 *        but are never selected as the closest point.
 * @param params See VirtualPoseParams.
 */
VirtualPoseResult compute_virtual_pose(
  const geometry_msgs::msg::Pose & measured_pose, const std::vector<Eigen::Matrix4d> & polyline,
  int64_t prefix_count, const VirtualPoseParams & params);

}  // namespace autoware::ml_planner::utils

#endif  // AUTOWARE__ML_PLANNER__UTILS__VIRTUAL_POSE_HPP_
