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

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/trajectory_point.hpp>

#include <cmath>
#include <memory>

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
  ProcessingResult result = ProcessingResult::Unchanged;
  if (!enabled_ || !is_trajectory_modification_required(traj_points, input)) {
    return result;
  }
  for (auto & point : traj_points) {
    const auto map_velocity_limit =
      extended_route_handler_->get_velocity_limit(point.pose.position, limit_overrides_);
    if (
      map_velocity_limit && std::isfinite(*map_velocity_limit) &&
      point.longitudinal_velocity_mps > *map_velocity_limit) {
      point.longitudinal_velocity_mps = static_cast<float>(*map_velocity_limit);
      result = ProcessingResult::Modified;
    }
  }
  if (!enable_smoothing_) {
    return result;
  }
  const auto current_velocity = input.current_odometry->twist.twist.linear.x;
  if (traj_points.size() < 2 || current_velocity <= 0.0) {
    return result;
  }

  auto distance = motion_utils::calcSignedArcLength(
    traj_points, input.current_odometry->pose.pose.position, size_t{0});
  const auto velocity_squared = current_velocity * current_velocity;
  const auto twice_deceleration = 2.0 * std::abs(constant_deceleration_);
  for (size_t i = 0; i < traj_points.size(); ++i) {
    auto & point = traj_points[i];
    if (i > 0) {
      const auto & previous_position = traj_points[i - 1].pose.position;
      distance += std::hypot(
        point.pose.position.x - previous_position.x, point.pose.position.y - previous_position.y);
    }
    if (distance < 0.0) {
      continue;
    }
    const auto reachable_velocity_squared = velocity_squared - twice_deceleration * distance;
    if (reachable_velocity_squared <= 0.0) {
      break;
    }
    const auto velocity = point.longitudinal_velocity_mps;
    if (velocity >= 0.0 && velocity * velocity < reachable_velocity_squared) {
      point.longitudinal_velocity_mps = static_cast<float>(std::sqrt(reachable_velocity_squared));
      result = ProcessingResult::Modified;
    }
  }
  return result;
}
}  // namespace autoware::trajectory_processor::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_processor::plugin::MapVelocityLimits,
  autoware::trajectory_processor::plugin::TrajectoryProcessorPluginBase)
