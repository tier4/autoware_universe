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

#include "autoware/trajectory_processor/trajectory_modifier_plugins/map_velocity_limits.hpp"

#include "autoware/trajectory_processor/trajectory_processor_plugin_base.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/interpolation/spherical_linear_interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/trajectory_point.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace autoware::trajectory_processor::plugin
{

namespace
{
autoware::avoidance_target_detector::ExtendedRouteHandler::VelocityLimitOverrides
make_velocity_limit_overrides(const TrajectoryProcessorParams & params)
{
  autoware::avoidance_target_detector::ExtendedRouteHandler::VelocityLimitOverrides overrides;
  const auto & ids = params.map_velocity_limits.limit_velocity_from_map_debug_lanelet_ids;
  const auto & velocities = params.map_velocity_limits.limit_velocity_from_map_debug_max_velocities;
  if (ids.size() != velocities.size()) {
    throw std::invalid_argument(
      "limit_velocity_from_map_debug_lanelet_ids and "
      "limit_velocity_from_map_debug_max_velocities must have equal lengths");
  }
  for (std::size_t index = 0; index < ids.size(); ++index) {
    if (!std::isfinite(velocities[index]) || velocities[index] < 0.0) {
      throw std::invalid_argument(
        "limit_velocity_from_map_debug_max_velocities must contain finite non-negative values");
    }
    if (!overrides.emplace(ids[index], velocities[index]).second) {
      throw std::invalid_argument(
        "limit_velocity_from_map_debug_lanelet_ids must not contain duplicates");
    }
  }
  return overrides;
}
}  // namespace

void MapVelocityLimits::on_initialize(const TrajectoryProcessorParams & params)
{
  update_params(params);
}

void MapVelocityLimits::update_params(const TrajectoryProcessorParams & params)
{
  enabled_ = params.use_map_velocity_limits;
  limit_overrides_ = make_velocity_limit_overrides(params);
  constant_deceleration_ = params.stopping_constraints.nominal_deceleration;
  enable_smoothing_ = params.map_velocity_limits.enable_smoothing;
}

bool MapVelocityLimits::is_trajectory_modification_required(
  [[maybe_unused]] const TrajectoryPoints & traj_points, const TrajectoryProcessorData & input)
{
  if (!input.lanelet_map_bin || !input.route) {
    return false;
  }
  if (!extended_route_handler_ || previous_route_uuid_ != input.route->uuid) {
    auto handler = std::make_shared<autoware::avoidance_target_detector::ExtendedRouteHandler>(
      *input.lanelet_map_bin, *input.route);
    handler->create_map();
    extended_route_handler_ = handler;
    previous_route_uuid_ = input.route->uuid;
  }
  return true;
}
ProcessingResult MapVelocityLimits::process(
  TrajectoryPoints & traj_points, TrajectoryProcessorData & input)
{
  if (!enabled_ || !is_trajectory_modification_required(traj_points, input)) {
    return ProcessingResult::Unchanged;
  }

  bool modified = false;
  const double dt = 0.1;  // Fixed time step

  // 1. Cap the map limits
  for (auto & point : traj_points) {
    const auto map_velocity_limit =
      extended_route_handler_->get_velocity_limit(point.pose.position, limit_overrides_);
    if (
      map_velocity_limit && std::isfinite(*map_velocity_limit) &&
      point.longitudinal_velocity_mps > *map_velocity_limit) {
      point.longitudinal_velocity_mps = static_cast<float>(*map_velocity_limit);
      modified = true;
    }
  }

  if (!enable_smoothing_ || traj_points.size() < 2) {
    return modified ? ProcessingResult::Modified : ProcessingResult::Unchanged;
  }

  const double max_decel = std::abs(constant_deceleration_);
  const double max_accel = max_decel;

  // 2. Backward pass: Enforce deceleration limits (v_i <= v_{i+1} + decel * dt)
  for (int i = static_cast<int>(traj_points.size()) - 2; i >= 0; --i) {
    const double limit = traj_points[i + 1].longitudinal_velocity_mps + (max_decel * dt);
    if (traj_points[i].longitudinal_velocity_mps > limit) {
      traj_points[i].longitudinal_velocity_mps = static_cast<float>(limit);
      modified = true;
    }
  }

  // 3. Forward pass: Enforce acceleration limits (v_i <= v_{i-1} + accel * dt)
  const auto current_velocity = std::max(0.0, input.current_odometry->twist.twist.linear.x);

  const double first_point_limit = current_velocity + (max_accel * dt);
  if (traj_points[0].longitudinal_velocity_mps > first_point_limit) {
    traj_points[0].longitudinal_velocity_mps = static_cast<float>(first_point_limit);
    modified = true;
  }

  for (size_t i = 1; i < traj_points.size(); ++i) {
    const double limit = traj_points[i - 1].longitudinal_velocity_mps + (max_accel * dt);
    if (traj_points[i].longitudinal_velocity_mps > limit) {
      traj_points[i].longitudinal_velocity_mps = static_cast<float>(limit);
      modified = true;
    }
  }

  if (!modified) {
    return ProcessingResult::Unchanged;
  }

  // 4. Re-calculate spatial positions by interpolating the original shape
  const auto original_points = traj_points;

  // 4a. Calculate cumulative arc length of the original trajectory
  std::vector<double> orig_s(original_points.size(), 0.0);
  for (size_t i = 1; i < original_points.size(); ++i) {
    const double dx = original_points[i].pose.position.x - original_points[i - 1].pose.position.x;
    const double dy = original_points[i].pose.position.y - original_points[i - 1].pose.position.y;
    const double dz = original_points[i].pose.position.z - original_points[i - 1].pose.position.z;
    orig_s[i] = orig_s[i - 1] + std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  // 4b. Calculate the new desired arc lengths based on the capped velocity profile
  std::vector<double> new_s(traj_points.size(), 0.0);
  for (size_t i = 1; i < traj_points.size(); ++i) {
    // s = v * dt
    new_s[i] = new_s[i - 1] + traj_points[i - 1].longitudinal_velocity_mps * dt;
  }

  // 4c. Sample the original trajectory at the new arc lengths
  size_t orig_idx = 0;
  for (size_t i = 1; i < traj_points.size(); ++i) {
    const double target_s = new_s[i];

    // Advance the index to find the original segment bracketing target_s
    while (orig_idx + 1 < orig_s.size() && orig_s[orig_idx + 1] <= target_s) {
      orig_idx++;
    }

    if (orig_idx + 1 >= orig_s.size()) {
      // If the new velocity profile pushes us past the end of the original trajectory,
      // safely clamp to the final pose.
      traj_points[i].pose = original_points.back().pose;
    } else {
      // Interpolate exactly between the original points
      const double s_start = orig_s[orig_idx];
      const double s_end = orig_s[orig_idx + 1];
      const double ratio =
        std::clamp((target_s - s_start) / std::max(s_end - s_start, 1e-6), 0.0, 1.0);

      const auto & p0 = original_points[orig_idx].pose;
      const auto & p1 = original_points[orig_idx + 1].pose;

      traj_points[i].pose.position.x =
        autoware::interpolation::lerp(p0.position.x, p1.position.x, ratio);
      traj_points[i].pose.position.y =
        autoware::interpolation::lerp(p0.position.y, p1.position.y, ratio);
      traj_points[i].pose.position.z =
        autoware::interpolation::lerp(p0.position.z, p1.position.z, ratio);

      // Slerp the quaternion orientation to prevent normalization errors
      traj_points[i].pose.orientation =
        autoware::interpolation::lerpOrientation(p0.orientation, p1.orientation, ratio);
    }
  }

  // 5. Calculate the acceleration via the time derivative (dv / dt)
  for (size_t i = 0; i + 1 < traj_points.size(); ++i) {
    const double dv =
      traj_points[i + 1].longitudinal_velocity_mps - traj_points[i].longitudinal_velocity_mps;
    traj_points[i].acceleration_mps2 = static_cast<float>(dv / dt);
  }

  // Pad the last point's acceleration to match the previous point
  traj_points.back().acceleration_mps2 = traj_points[traj_points.size() - 2].acceleration_mps2;

  return ProcessingResult::Modified;
}

}  // namespace autoware::trajectory_processor::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_processor::plugin::MapVelocityLimits,
  autoware::trajectory_processor::plugin::TrajectoryProcessorPluginBase)
