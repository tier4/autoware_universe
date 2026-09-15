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

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
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

namespace detail
{
MapVelocityLimitResult apply_map_velocity_limits(
  TrajectoryPoints & points, const double deceleration,
  const std::function<std::optional<double>(const geometry_msgs::msg::Point &)> & velocity_limit)
{
  if (points.empty()) {
    return {};
  }

  if (!std::isfinite(deceleration) || deceleration < 0.0) {
    return MapVelocityLimitResult{
      ProcessingResult::Unchanged, "Map velocity limiting requires non-negative deceleration"};
  }

  constexpr double dt = 0.1;
  const auto original = points;
  const auto count = points.size();

  // 1. Limit each velocity with the map velocity limit
  std::optional<std::size_t> first_modified_idx = std::nullopt;

  for (std::size_t i = 0; i < count; ++i) {
    const auto limit = velocity_limit(points[i].pose.position);
    if (limit && std::isfinite(*limit) && *limit >= 0.0) {
      if (points[i].longitudinal_velocity_mps > *limit) {
        points[i].longitudinal_velocity_mps = static_cast<float>(*limit);
        points[i].acceleration_mps2 = std::min(points[i].acceleration_mps2, 0.0F);
        // Remember the 1st modified point
        if (!first_modified_idx) {
          first_modified_idx = i;
        }
      }
    }
  }

  // If no point was above the velocity limit -> return Unchanged
  if (!first_modified_idx) {
    return {ProcessingResult::Unchanged, {}};
  }

  // 2. Update velocities and accelerations between the 1st trajectory point and the 1st modified
  // point We apply the desired deceleration backward, ignoring current ego velocity feasibility.
  for (int i = static_cast<int>(*first_modified_idx); i >= 0; --i) {
    // Only calculate the backward deceleration curve for points strictly before the first modified
    // index
    if (i < static_cast<int>(*first_modified_idx)) {
      const float target_velocity =
        points[i + 1].longitudinal_velocity_mps + static_cast<float>(deceleration * dt);

      if (points[i].longitudinal_velocity_mps > target_velocity) {
        points[i].longitudinal_velocity_mps = target_velocity;
        points[i].acceleration_mps2 = -static_cast<float>(deceleration);
      } else {
        points[i].acceleration_mps2 = static_cast<float>(
          (points[i + 1].longitudinal_velocity_mps - points[i].longitudinal_velocity_mps) / dt);
      }
    }
  }

  // 3. Starting from the 2nd point, update the point arc lengths along the trajectory
  // based on the updated velocities to preserve the original shape.
  std::vector<double> original_s(count, 0.0);
  for (std::size_t i = 1; i < count; ++i) {
    const auto & p0 = original[i - 1].pose.position;
    const auto & p1 = original[i].pose.position;
    original_s[i] = original_s[i - 1] + std::hypot(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
  }

  std::vector<double> new_s(count, 0.0);
  for (std::size_t i = 1; i < count; ++i) {
    // Integrate new distance using the trapezoidal rule
    new_s[i] =
      new_s[i - 1] +
      0.5 * (points[i - 1].longitudinal_velocity_mps + points[i].longitudinal_velocity_mps) * dt;
  }

  std::size_t segment = 0;
  for (std::size_t i = 1; i < count; ++i) {
    // Advance the segment to match the new integrated arc length
    while (segment + 1 < count && original_s[segment + 1] <= new_s[i]) {
      ++segment;
    }

    if (segment + 1 == count) {
      // If the new velocity pushes us past the spatial end of the original path, clamp to the last
      // pose
      points[i].pose = original.back().pose;
    } else {
      const double segment_length = original_s[segment + 1] - original_s[segment];
      const double ratio = (new_s[i] - original_s[segment]) / std::max(segment_length, 1e-6);

      const auto & p0 = original[segment].pose;
      const auto & p1 = original[segment + 1].pose;
      auto & pose = points[i].pose;

      // Interpolate the exact shape
      pose.position.x = autoware::interpolation::lerp(p0.position.x, p1.position.x, ratio);
      pose.position.y = autoware::interpolation::lerp(p0.position.y, p1.position.y, ratio);
      pose.position.z = autoware::interpolation::lerp(p0.position.z, p1.position.z, ratio);
      pose.orientation =
        autoware::interpolation::lerpOrientation(p0.orientation, p1.orientation, ratio);
    }
  }

  return {ProcessingResult::Modified, {}};
}
}  // namespace detail

ProcessingResult MapVelocityLimits::process(
  TrajectoryPoints & traj_points, TrajectoryProcessorData & input)
{
  if (
    !enabled_ || traj_points.empty() || !input.current_odometry ||
    !is_trajectory_modification_required(traj_points, input)) {
    return ProcessingResult::Unchanged;
  }
  const auto result = detail::apply_map_velocity_limits(
    traj_points, std::abs(constant_deceleration_),
    [this](const geometry_msgs::msg::Point & position) {
      return extended_route_handler_->get_velocity_limit(position, limit_overrides_);
    });
  return result.status;
}

}  // namespace autoware::trajectory_processor::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_processor::plugin::MapVelocityLimits,
  autoware::trajectory_processor::plugin::TrajectoryProcessorPluginBase)
