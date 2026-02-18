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

#include "path_shift_to_ego.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

double lookup_table(
  const std::vector<double> & breakpoints, const std::vector<double> & values, const double query)
{
  if (breakpoints.empty()) {
    return 0.0;
  }
  // Clamp at boundaries (no extrapolation)
  if (query <= breakpoints.front()) {
    return values.front();
  }
  if (query >= breakpoints.back()) {
    return values.back();
  }
  // Find the upper bracket
  const auto it = std::lower_bound(breakpoints.begin(), breakpoints.end(), query);
  const size_t idx = static_cast<size_t>(it - breakpoints.begin());
  const double v0 = breakpoints[idx - 1];
  const double v1 = breakpoints[idx];
  const double ratio = (query - v0) / (v1 - v0);
  return values[idx - 1] + ratio * (values[idx] - values[idx - 1]);
}

Trajectory shift_trajectory_to_ego(
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & ego_pose,
  const double ego_velocity, const TrajectoryShiftParams & params, const double delta_arc_length)
{
  if (trajectory.points.size() < 2) {
    return trajectory;
  }

  // Calculate lateral offset of ego from the trajectory
  const double lateral_offset =
    autoware::motion_utils::calcLateralOffset(trajectory.points, ego_pose.position);

  // Calculate yaw deviation between ego and the nearest trajectory point
  const size_t nearest_idx =
    autoware::motion_utils::findNearestIndex(trajectory.points, ego_pose.position);
  const double ego_yaw_raw = tf2::getYaw(ego_pose.orientation);
  const double traj_yaw = tf2::getYaw(trajectory.points.at(nearest_idx).pose.orientation);
  const double yaw_deviation = std::abs(autoware_utils::normalize_radian(ego_yaw_raw - traj_yaw));

  // If both lateral offset and yaw deviation are small enough, no shift needed
  if (
    std::abs(lateral_offset) < params.minimum_shift_length &&
    yaw_deviation < params.minimum_shift_yaw) {
    return trajectory;
  }

  // Look up ratio and guide_distance from velocity-dependent LUT
  const double ratio = lookup_table(
    params.velocity_breakpoints, params.shift_length_to_distance_ratio_table, ego_velocity);
  const double shift_distance =
    std::max(params.minimum_shift_distance, /*std::abs(lateral_offset) **/ ratio);
  const double guide_distance =
    lookup_table(params.velocity_breakpoints, params.guide_distance_table, ego_velocity);

  RCLCPP_INFO(rclcpp::get_logger(""), "shift_distance: %lf", shift_distance);

  // Find the nearest index on the trajectory for ego position
  const size_t ego_nearest_idx =
    autoware::motion_utils::findNearestIndex(trajectory.points, ego_pose.position);

  // Walk forward along the trajectory to find the skip end index
  double accumulated_length = 0.0;
  size_t skip_idx = ego_nearest_idx;
  for (size_t i = ego_nearest_idx; i + 1 < trajectory.points.size(); ++i) {
    accumulated_length += autoware_utils::calc_distance2d(
      trajectory.points.at(i).pose.position, trajectory.points.at(i + 1).pose.position);
    skip_idx = i + 1;
    if (accumulated_length >= shift_distance) {
      break;
    }
  }

  // If not enough room to shift (skip end is at trajectory end), skip
  if (skip_idx >= trajectory.points.size() - 1) {
    return trajectory;
  }

  // Build new waypoints: ego + guide point + remaining trajectory from skip_idx onward
  const double ref_velocity = trajectory.points.at(ego_nearest_idx).longitudinal_velocity_mps;
  const double ego_yaw = tf2::getYaw(ego_pose.orientation);

  TrajectoryPoint ego_point;
  ego_point.pose = ego_pose;
  ego_point.longitudinal_velocity_mps = ref_velocity;

  // Guide point along ego's heading to constrain the spline's initial tangent
  TrajectoryPoint guide_point;
  guide_point.pose.position.x = ego_pose.position.x + guide_distance * std::cos(ego_yaw);
  guide_point.pose.position.y = ego_pose.position.y + guide_distance * std::sin(ego_yaw);
  guide_point.pose.position.z = ego_pose.position.z;
  guide_point.pose.orientation = ego_pose.orientation;
  guide_point.longitudinal_velocity_mps = ref_velocity;

  std::vector<TrajectoryPoint> new_points;
  new_points.reserve(trajectory.points.size() - skip_idx + 2);
  new_points.push_back(ego_point);
  new_points.push_back(guide_point);
  new_points.insert(
    new_points.end(), trajectory.points.begin() + skip_idx, trajectory.points.end());

  // (void)delta_arc_length;
  // Build smooth trajectory using pretty_build (CubicSpline interpolation)
  const auto smooth_traj =
    autoware::experimental::trajectory::pretty_build<TrajectoryPoint>(new_points);
  if (!smooth_traj) {
    // fallback: return the raw stitched points
    Trajectory output;
    output.header = trajectory.header;
    output.points = new_points;
    return output;
  }

  // Resample at uniform intervals
  const double total_length = smooth_traj->length();
  std::vector<TrajectoryPoint> resampled_points;
  for (double s = 0.0; s < total_length; s += delta_arc_length) {
    resampled_points.push_back(smooth_traj->compute(s));
  }
  // Ensure the last point is included
  const double last_sampled =
    resampled_points.empty() ? -1.0 : (resampled_points.size() - 1) * delta_arc_length;
  if (total_length - last_sampled > 1e-6) {
    resampled_points.push_back(smooth_traj->compute(total_length));
  }

  Trajectory output;
  output.header = trajectory.header;
  output.points = resampled_points;
  return output;
}

Trajectory shift_trajectory_to_ego_quintic(
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & ego_pose,
  const double ego_velocity, const double ego_yaw_rate, const TrajectoryShiftParams & params,
  const double delta_arc_length)
{
  if (trajectory.points.size() < 2) {
    return trajectory;
  }

  // Calculate lateral offset and yaw deviation
  const double lateral_offset =
    autoware::motion_utils::calcLateralOffset(trajectory.points, ego_pose.position);
  const size_t nearest_idx =
    autoware::motion_utils::findNearestIndex(trajectory.points, ego_pose.position);
  const double ego_yaw = tf2::getYaw(ego_pose.orientation);
  const double traj_yaw = tf2::getYaw(trajectory.points.at(nearest_idx).pose.orientation);
  const double signed_yaw_dev = autoware_utils::normalize_radian(ego_yaw - traj_yaw);
  const double abs_yaw_dev = std::abs(signed_yaw_dev);

  // If both lateral offset and yaw deviation are small enough, no shift needed
  if (
    std::abs(lateral_offset) < params.minimum_shift_length &&
    abs_yaw_dev < params.minimum_shift_yaw) {
    return trajectory;
  }

  // Compute initial curvature from yaw rate and velocity
  // κ₀ = yaw_rate / velocity, with velocity clamped to avoid division by near-zero
  constexpr double min_velocity_for_curvature = 2.77;  // [m/s]
  const double clamped_velocity = std::max(std::abs(ego_velocity), min_velocity_for_curvature);

  // Calculate merge distance L so that peak lateral acceleration stays below a_limit.
  // Peak curvature of the quintic polynomial: \kappa_max ≈ 5.77 * |d| / L²
  // Lateral acceleration: a_lat = v² * κ_max ≤ a_limit
  // → L ≥ v * √(5.77 * |d| / a_limit)
  constexpr double a_limit = 0.5;       // [m/s²] maximum lateral acceleration
  constexpr double kappa_coeff = 5.77;  // peak curvature coefficient of quintic polynomial
  const double abs_d = std::abs(lateral_offset);
  double L = std::max(
    params.minimum_shift_distance,
    std::abs(clamped_velocity) * std::sqrt(kappa_coeff * abs_d / a_limit));

  // Walk forward to find merge index, clamping L to available trajectory length
  double accumulated_length = 0.0;
  size_t merge_idx = nearest_idx;
  for (size_t i = nearest_idx; i + 1 < trajectory.points.size(); ++i) {
    accumulated_length += autoware_utils::calc_distance2d(
      trajectory.points.at(i).pose.position, trajectory.points.at(i + 1).pose.position);
    merge_idx = i + 1;
    if (accumulated_length >= L) {
      break;
    }
  }
  if (merge_idx >= trajectory.points.size() - 1) {
    // Clamp L to the remaining trajectory length and set merge_idx to the second-to-last point
    L = accumulated_length;
    merge_idx = trajectory.points.size() - 2;
  }

  const double kappa0 = ego_yaw_rate / clamped_velocity;

  // Quintic polynomial coefficients (analytical solution)
  // y(s) = a0 + a1*s + a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5
  // Boundary conditions:
  //   s=0: y=d, y'=tan(Δθ), y''=κ₀
  //   s=L: y=0, y'=0, y''=0
  const double d = lateral_offset;
  const double q = std::tan(std::clamp(signed_yaw_dev, -M_PI / 4.0, M_PI / 4.0));
  const double L2 = L * L;
  const double L3 = L2 * L;
  const double a0 = d;
  const double a1 = q;
  const double a2 = kappa0 / 2.0;
  const double a3 = -(20.0 * d + 12.0 * q * L + 3.0 * kappa0 * L2) / (2.0 * L3);
  const double a4 = (30.0 * d + 16.0 * q * L + 3.0 * kappa0 * L2) / (2.0 * L3 * L);
  const double a5 = -(12.0 * d + 6.0 * q * L + kappa0 * L2) / (2.0 * L3 * L2);

  // Sample points along the reference trajectory with lateral offset from quintic polynomial
  const double ref_velocity = trajectory.points.at(nearest_idx).longitudinal_velocity_mps;
  std::vector<TrajectoryPoint> shifted_points;

  // First point is the ego position directly
  TrajectoryPoint ego_pt;
  ego_pt.pose = ego_pose;
  ego_pt.longitudinal_velocity_mps = ref_velocity;
  shifted_points.push_back(ego_pt);

  for (double s = delta_arc_length; s < L; s += delta_arc_length) {
    // Evaluate quintic polynomial
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double y_s = a0 + a1 * s + a2 * s2 + a3 * s3 + a4 * s3 * s + a5 * s3 * s2;
    const double yp_s = a1 + 2.0 * a2 * s + 3.0 * a3 * s2 + 4.0 * a4 * s3 + 5.0 * a5 * s3 * s;

    // Get corresponding pose on the original trajectory
    const auto base_pose =
      autoware::motion_utils::calcLongitudinalOffsetPose(trajectory.points, ego_pose.position, s);
    if (!base_pose) {
      break;
    }

    // Offset laterally: normal vector is (-sin(yaw), cos(yaw)), left-positive
    const double base_yaw = tf2::getYaw(base_pose->orientation);
    TrajectoryPoint pt;
    pt.pose.position.x = base_pose->position.x + y_s * (-std::sin(base_yaw));
    pt.pose.position.y = base_pose->position.y + y_s * std::cos(base_yaw);
    pt.pose.position.z = base_pose->position.z;
    pt.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(base_yaw + std::atan2(yp_s, 1.0));
    pt.longitudinal_velocity_mps = ref_velocity;
    shifted_points.push_back(pt);
  }

  // Append remaining trajectory points after merge
  for (size_t i = merge_idx; i < trajectory.points.size(); ++i) {
    shifted_points.push_back(trajectory.points.at(i));
  }

  Trajectory output;
  output.header = trajectory.header;
  output.points = shifted_points;
  return output;
}

}  // namespace autoware::minimum_rule_based_planner
