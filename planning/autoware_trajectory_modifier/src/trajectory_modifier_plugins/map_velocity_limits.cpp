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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/map_velocity_limits.hpp"

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/velocity_limits.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_plugin_base.hpp"

#include <cmath>
#include <memory>
#include <utility>

namespace autoware::trajectory_modifier::plugin
{

namespace
{
autoware::avoidance_target_detector::ExtendedRouteHandler::VelocityLimitOverrides
make_velocity_limit_overrides(const TrajectoryModifierParams & params)
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

void MapVelocityLimits::on_initialize(const TrajectoryModifierParams & params)
{
  update_params(params);
}

void MapVelocityLimits::update_params(const TrajectoryModifierParams & params)
{
  enabled_ = params.use_map_velocity_limits;
  limit_overrides_ = make_velocity_limit_overrides(params);
  constant_deceleration_ = params.stopping_constraints.nominal_deceleration;
}

bool MapVelocityLimits::is_trajectory_modification_required(
  [[maybe_unused]] const TrajectoryPoints & traj_points, const TrajectoryModifierData & input)
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
  TrajectoryPoints & traj_points, TrajectoryModifierData & input)
{
  if (
    !enabled_ || traj_points.empty() || !input.current_odometry ||
    !is_trajectory_modification_required(traj_points, input)) {
    return ProcessingResult::Unchanged;
  }
  const auto result = detail::apply_velocity_limits(
    traj_points, std::abs(constant_deceleration_),
    [this](const geometry_msgs::msg::Point & position) {
      return extended_route_handler_->get_velocity_limit(position, limit_overrides_);
    });
  return result.status;
}

}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::MapVelocityLimits,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
