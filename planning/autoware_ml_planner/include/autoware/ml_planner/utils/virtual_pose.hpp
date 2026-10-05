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
#include <string>
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
  // Reset when the vehicle is farther than these from the virtual pose, measured along / across the
  // virtual heading.
  double max_longitudinal_error_m{0.5};
  double max_lateral_error_m{0.3};
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
  // Trajectory the virtual pose is taken from: "raw" (the model's prediction) or "optimized"
  // (the node's output after border avoidance and trajectory optimization).
  std::string reference;
  // Low-speed mode: while engaged and slow, the virtual pose is the previous trajectory at the
  // elapsed time instead of its closest point, so a vehicle pulling away is planned from where the
  // trajectory says it is by now. Hysteresis: entered below enter speed, left above exit speed.
  bool time_based_enable{false};
  double time_based_enter_speed_mps{0.5};
  double time_based_exit_speed_mps{0.7};
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
  // The pose was taken at the elapsed time on the previous trajectory (low-speed mode).
  bool time_based{false};
};

struct ElapsedTimePoint
{
  Eigen::Vector2d position;
  // Speed of the previous trajectory at the elapsed time (segment length over its duration).
  double speed_mps;
};

/**
 * @brief Point of the previous trajectory at the elapsed time, linear between its vertices.
 *
 * @param frame_pose Planning start pose of the previous trajectory (time 0).
 * @param prediction Following poses of the previous trajectory.
 * @param times Time of each prediction pose from the planning start [s], increasing.
 * @param elapsed_sec Time since the previous planning start; clamped to the trajectory.
 */
std::optional<ElapsedTimePoint> point_at_elapsed_time(
  const Eigen::Matrix4d & frame_pose, const std::vector<Eigen::Matrix4d> & prediction,
  const std::vector<double> & times, double elapsed_sec);

/**
 * @brief Low-speed mode state for the next cycle, with hysteresis against chattering.
 *
 * Off whenever the mode is disabled or the vehicle is not engaged. Otherwise switched on below the
 * enter speed and off above the exit speed; in between the current state is kept.
 */
bool update_time_based_mode(
  bool active, bool engaged, double speed_mps, const VirtualPoseParams & params);

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

/**
 * @brief Same, but the virtual pose is the point of the spline closest to query instead of the
 *        measured position; the reset limits are still checked against the measured pose.
 */
VirtualPoseResult compute_virtual_pose(
  const geometry_msgs::msg::Pose & measured_pose, const geometry_msgs::msg::Point & query,
  const std::vector<Eigen::Matrix4d> & polyline, int64_t prefix_count,
  const VirtualPoseParams & params);

}  // namespace autoware::ml_planner::utils

#endif  // AUTOWARE__ML_PLANNER__UTILS__VIRTUAL_POSE_HPP_
