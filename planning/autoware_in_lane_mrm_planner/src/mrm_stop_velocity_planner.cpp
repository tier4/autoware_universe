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

#include "mrm_stop_velocity_planner.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

namespace autoware::in_lane_mrm_planner
{
namespace
{
constexpr double kStopVelocityThreshold = 0.01;
constexpr double kMinVelocityForTimeCalc = 0.1;
constexpr double kIntegrationDt = 0.02;

double extract_longitudinal_accel(const AccelWithCovarianceStamped & accel)
{
  return accel.accel.accel.linear.x;
}

std::vector<double> calc_arc_lengths(const TrajectoryPoints & points)
{
  std::vector<double> arc_lengths(points.size(), 0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    arc_lengths.at(i) = autoware::motion_utils::calcSignedArcLength(points, 0, i);
  }
  return arc_lengths;
}

TrajectoryPoint interpolate_point(
  const TrajectoryPoint & from, const TrajectoryPoint & to, const double ratio)
{
  TrajectoryPoint out;
  out.pose = autoware_utils_geometry::calc_interpolated_pose(from.pose, to.pose, ratio);
  out.longitudinal_velocity_mps = 0.0F;
  out.lateral_velocity_mps = 0.0F;
  out.acceleration_mps2 = 0.0F;
  out.heading_rate_rps = 0.0F;
  return out;
}

TrajectoryPoint sample_point_at_arc_length(
  const TrajectoryPoints & points, const std::vector<double> & arc_lengths, const double s)
{
  if (points.empty()) {
    return TrajectoryPoint{};
  }
  if (s <= 0.0) {
    return points.front();
  }
  if (s >= arc_lengths.back()) {
    return points.back();
  }

  const auto it = std::upper_bound(arc_lengths.begin(), arc_lengths.end(), s);
  const size_t idx = static_cast<size_t>(std::distance(arc_lengths.begin(), it));
  const size_t i0 = std::max<size_t>(1, idx) - 1;
  const size_t i1 = i0 + 1;
  const double ds = arc_lengths.at(i1) - arc_lengths.at(i0);
  const double ratio = (ds < 1e-9) ? 0.0 : (s - arc_lengths.at(i0)) / ds;
  return interpolate_point(points.at(i0), points.at(i1), ratio);
}

void advance_decel_state(
  double & v, double & a, const double ds, const double jerk, const double a_target)
{
  if (ds <= 0.0 || v <= 0.0) {
    v = 0.0;
    a = a_target;
    return;
  }

  double remaining = ds;
  while (remaining > 1e-9 && v > 0.0) {
    const double dt = std::min(kIntegrationDt, remaining / std::max(v, kStopVelocityThreshold));
    if (a > a_target) {
      a += jerk * dt;
      a = std::max(a, a_target);
    } else {
      a = a_target;
    }
    v += a * dt;
    if (v <= 0.0) {
      v = 0.0;
      a = a_target;
      break;
    }
    remaining -= v * dt;
  }
}

double calc_required_stop_distance(const double v0, const double a0, const double jerk, const double a_target)
{
  double v = std::max(0.0, v0);
  double a = a0;
  double distance = 0.0;
  constexpr double max_distance = 500.0;

  while (v > kStopVelocityThreshold && distance < max_distance) {
    const double dt = kIntegrationDt;
    const double ds = std::max(v * dt, kStopVelocityThreshold * dt);
    advance_decel_state(v, a, ds, jerk, a_target);
    distance += ds;
  }
  return distance;
}

bool can_relax_jerk(const double jerk, const double max_jerk)
{
  return jerk > max_jerk;
}

bool can_relax_decel(const double decel, const double max_decel)
{
  return decel > max_decel;
}

}  // namespace

MrmStopVelocityPlanner::MrmStopVelocityPlanner(const Params & params) : params_(params.mrm_velocity)
{
}

void MrmStopVelocityPlanner::update_params(const Params & params)
{
  params_ = params.mrm_velocity;
}

std::optional<size_t> MrmStopVelocityPlanner::find_constraint_stop_index(
  const TrajectoryPoints & points)
{
  if (points.size() < 2) {
    return std::nullopt;
  }

  for (size_t i = 0; i + 1 < points.size(); ++i) {
    if (points.at(i).longitudinal_velocity_mps <= kStopVelocityThreshold) {
      return i;
    }
  }
  return points.size() - 1;
}

bool MrmStopVelocityPlanner::is_feasible(
  const TrajectoryPoints & points, const size_t ego_idx, const size_t constraint_idx,
  const double v0, const double a0, const double jerk, const double decel) const
{
  if (points.empty() || ego_idx >= points.size() || constraint_idx >= points.size()) {
    return false;
  }

  const double available_distance = autoware::motion_utils::calcSignedArcLength(
    points, ego_idx, constraint_idx);
  const double required_distance = calc_required_stop_distance(v0, a0, jerk, decel);
  return available_distance + 1e-3 >= required_distance;
}

double MrmStopVelocityPlanner::required_stop_distance(
  const double v0, const double a0, const double jerk, const double decel) const
{
  return calc_required_stop_distance(v0, a0, jerk, decel);
}

MrmStopVelocityPlanner::DecelLimits MrmStopVelocityPlanner::select_profile_limits(
  const TrajectoryPoints & points, const size_t ego_idx, const size_t constraint_idx,
  const double v0, const double a0) const
{
  DecelLimits limits{params_.target_jerk, params_.target_deceleration};

  if (is_feasible(points, ego_idx, constraint_idx, v0, a0, limits.jerk, limits.decel)) {
    return limits;
  }

  while (!is_feasible(points, ego_idx, constraint_idx, v0, a0, limits.jerk, limits.decel)) {
    if (can_relax_jerk(limits.jerk, params_.max_jerk_relaxation)) {
      limits.jerk += params_.step_jerk_relaxation;
      continue;
    }
    if (can_relax_decel(limits.decel, params_.max_deceleration_relaxation)) {
      limits.decel += params_.step_deceleration_relaxation;
      continue;
    }

    RCLCPP_ERROR(
      rclcpp::get_logger("mrm_stop_velocity_planner"),
      "Cannot stop before constraint index %zu; applying max relaxation limits", constraint_idx);
    limits.jerk = params_.max_jerk_relaxation;
    limits.decel = params_.max_deceleration_relaxation;
    break;
  }

  return limits;
}

void MrmStopVelocityPlanner::densify_near_arc_length(
  TrajectoryPoints & points, const double center_arc_length) const
{
  if (points.size() < 2 || params_.decel_resample_interval <= 0.0) {
    return;
  }

  const auto arc_lengths = calc_arc_lengths(points);
  const double s_begin =
    std::max(0.0, center_arc_length - params_.decel_resample_range);
  const double s_end =
    std::min(arc_lengths.back(), center_arc_length + params_.decel_resample_range);

  std::set<double> sample_s;
  sample_s.insert(arc_lengths.front());
  sample_s.insert(arc_lengths.back());

  for (const double s : arc_lengths) {
    if (s < s_begin || s > s_end) {
      sample_s.insert(s);
    }
  }

  for (double s = s_begin; s <= s_end + 1e-6; s += params_.decel_resample_interval) {
    sample_s.insert(s);
  }

  TrajectoryPoints resampled;
  resampled.reserve(sample_s.size());
  for (const double s : sample_s) {
    resampled.push_back(sample_point_at_arc_length(points, arc_lengths, s));
  }
  points = std::move(resampled);
}

void MrmStopVelocityPlanner::fill_forward(
  TrajectoryPoints & points, const size_t ego_idx, const double v0, const double a0,
  const double jerk, const double decel) const
{
  if (points.empty() || ego_idx >= points.size()) {
    return;
  }

  double v = std::max(0.0, v0);
  double a = a0;
  bool stopped = false;

  points.at(ego_idx).longitudinal_velocity_mps = static_cast<float>(v);
  points.at(ego_idx).acceleration_mps2 = static_cast<float>(a);

  for (size_t i = ego_idx + 1; i < points.size(); ++i) {
    if (stopped) {
      points.at(i).longitudinal_velocity_mps = 0.0F;
      points.at(i).acceleration_mps2 = static_cast<float>(decel);
      continue;
    }

    const double ds =
      autoware::motion_utils::calcSignedArcLength(points, i - 1, i);
    advance_decel_state(v, a, ds, jerk, decel);

    if (v <= kStopVelocityThreshold) {
      stopped = true;
      v = 0.0;
      a = decel;
    }

    points.at(i).longitudinal_velocity_mps = static_cast<float>(v);
    points.at(i).acceleration_mps2 = static_cast<float>(a);
  }
}

void MrmStopVelocityPlanner::apply(
  TrajectoryPoints & points, const Odometry & odom,
  const AccelWithCovarianceStamped & accel) const
{
  if (points.empty()) {
    return;
  }

  const double v0 = std::max(0.0, odom.twist.twist.linear.x);
  const double a0 = extract_longitudinal_accel(accel);

  const size_t ego_idx = autoware::motion_utils::findNearestSegmentIndex(
    points, odom.pose.pose.position);
  const size_t constraint_idx =
    find_constraint_stop_index(points).value_or(points.size() - 1);

  const auto limits = select_profile_limits(points, ego_idx, constraint_idx, v0, a0);

  const double ego_arc_length = autoware::motion_utils::calcSignedArcLength(points, 0, ego_idx);
  const double predicted_stop_arc_length =
    ego_arc_length + calc_required_stop_distance(v0, a0, limits.jerk, limits.decel);
  densify_near_arc_length(points, predicted_stop_arc_length);

  const size_t ego_idx_after =
    autoware::motion_utils::findNearestSegmentIndex(points, odom.pose.pose.position);

  fill_forward(points, ego_idx_after, v0, a0, limits.jerk, limits.decel);

  autoware::motion_utils::calculate_time_from_start(
    points, odom.pose.pose.position, static_cast<float>(kMinVelocityForTimeCalc));
}

}  // namespace autoware::in_lane_mrm_planner
