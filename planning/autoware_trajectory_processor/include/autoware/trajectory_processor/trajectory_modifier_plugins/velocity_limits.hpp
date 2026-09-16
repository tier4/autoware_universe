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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__VELOCITY_LIMITS_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__VELOCITY_LIMITS_HPP_

#include "autoware/trajectory_processor/trajectory_processor_plugin_base.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <functional>
#include <optional>
#include <string>

namespace autoware::trajectory_processor::plugin::detail
{

struct VelocityLimitResult
{
  ProcessingResult status{ProcessingResult::Unchanged};
  std::string error;
};

struct VelocityLimitOptions
{
  bool make_profile_feasible{false};
  std::optional<double> current_ego_velocity{};
};

// Retimes along the input polyline, keeping its first pose and every timestamp. The callback
// permits resolving either a spatially varying map limit or one external limit for every point.
// Feasible mode anchors the velocity profile to the current ego velocity at t=0 and never raises
// a point above its original velocity.
VelocityLimitResult apply_velocity_limits(
  TrajectoryPoints & points, double deceleration,
  const std::function<std::optional<double>(const geometry_msgs::msg::Point &)> & velocity_limit,
  const VelocityLimitOptions & options = {});

}  // namespace autoware::trajectory_processor::plugin::detail

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__VELOCITY_LIMITS_HPP_
