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

#include "in_lane_mrm_trajectory_validator.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <cmath>
#include <limits>

namespace autoware::in_lane_mrm_planner
{
namespace
{
constexpr double kHazardousTrailingStoppedRatio = 0.8;
}  // namespace

InLaneMrmTrajectoryValidator::InLaneMrmTrajectoryValidator(const Params & params)
: params_(params.trajectory_validator)
{
}

void InLaneMrmTrajectoryValidator::update_params(const Params & params)
{
  params_ = params.trajectory_validator;
}

bool InLaneMrmTrajectoryValidator::has_finite_values(const TrajectoryPoints & points) const
{
  for (const auto & point : points) {
    if (
      !std::isfinite(point.pose.position.x) || !std::isfinite(point.pose.position.y) ||
      !std::isfinite(point.pose.position.z) || !std::isfinite(point.pose.orientation.w) ||
      !std::isfinite(point.pose.orientation.x) || !std::isfinite(point.pose.orientation.y) ||
      !std::isfinite(point.pose.orientation.z) ||
      !std::isfinite(point.longitudinal_velocity_mps) ||
      !std::isfinite(point.lateral_velocity_mps) || !std::isfinite(point.acceleration_mps2)) {
      return false;
    }
  }
  return true;
}

bool InLaneMrmTrajectoryValidator::has_sufficient_geometry(const TrajectoryPoints & points) const
{
  if (points.size() < static_cast<size_t>(params_.min_point_count)) {
    return false;
  }

  double min_spacing = std::numeric_limits<double>::max();
  for (size_t i = 1; i < points.size(); ++i) {
    const auto & p0 = points.at(i - 1).pose.position;
    const auto & p1 = points.at(i).pose.position;
    min_spacing = std::min(
      min_spacing, autoware_utils::calc_distance2d(p0, p1));
  }

  if (min_spacing < params_.min_point_spacing_m) {
    return false;
  }

  const double total_length =
    autoware::motion_utils::calcSignedArcLength(points, 0, points.size() - 1);
  return total_length + 1e-3 >= params_.min_trajectory_length_m;
}

bool InLaneMrmTrajectoryValidator::has_hazardous_velocity_step(
  const TrajectoryPoints & points) const
{
  if (points.size() < 2) {
    return false;
  }

  if (points.front().longitudinal_velocity_mps <= params_.hazardous_leading_velocity_mps) {
    return false;
  }

  size_t stopped_count = 0;
  for (size_t i = 1; i < points.size(); ++i) {
    if (points.at(i).longitudinal_velocity_mps <= params_.standstill_velocity_threshold_mps) {
      ++stopped_count;
    }
  }

  const double stopped_ratio =
    static_cast<double>(stopped_count) / static_cast<double>(points.size() - 1);
  return stopped_ratio >= kHazardousTrailingStoppedRatio;
}

bool InLaneMrmTrajectoryValidator::has_velocity_mismatch_at_standstill(
  const TrajectoryPoints & points, const double v0) const
{
  if (v0 > params_.standstill_velocity_threshold_mps) {
    return false;
  }

  for (const auto & point : points) {
    if (point.longitudinal_velocity_mps > params_.standstill_velocity_threshold_mps) {
      return true;
    }
  }
  return false;
}

InLaneMrmTrajectoryValidator::ValidationResult InLaneMrmTrajectoryValidator::validate(
  const TrajectoryPoints & points, const Odometry & odom) const
{
  ValidationResult result;
  result.ok = true;

  if (!params_.enable) {
    return result;
  }

  if (points.empty()) {
    result.ok = false;
    result.reason = "trajectory is empty";
    return result;
  }

  if (!has_finite_values(points)) {
    result.ok = false;
    result.reason = "trajectory contains non-finite values";
    return result;
  }

  if (!has_sufficient_geometry(points)) {
    result.ok = false;
    result.reason = "trajectory geometry is insufficient for planning output";
    return result;
  }

  const double v0 = std::max(0.0, odom.twist.twist.linear.x);

  if (has_hazardous_velocity_step(points)) {
    result.ok = false;
    result.reason = "hazardous velocity step profile detected";
    return result;
  }

  if (has_velocity_mismatch_at_standstill(points, v0)) {
    result.ok = false;
    result.reason = "non-zero velocity profile while ego is near standstill";
    return result;
  }

  return result;
}

}  // namespace autoware::in_lane_mrm_planner
