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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__VELOCITY_LIMITS_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__VELOCITY_LIMITS_HPP_

#include "autoware/trajectory_modifier/trajectory_modifier_plugin_base.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <functional>
#include <optional>
#include <string>

namespace autoware::trajectory_modifier::plugin::detail
{

struct VelocityLimitResult
{
  ProcessingResult status{ProcessingResult::Unchanged};
  std::string error;
};

/// @brief Bounds that the limited velocity profile never violates (all positive magnitudes).
struct VelocityLimitConstraints
{
  double max_acceleration{1.0};  ///< [m/s^2]
  double max_deceleration{1.0};  ///< [m/s^2]
  double max_jerk{1.0};          ///< [m/s^3]
};

struct VelocityLimitOptions
{
  std::optional<double> current_ego_velocity;
  std::optional<double> current_ego_acceleration;
};

/// @brief Ego velocity and acceleration of the plugin input, when available.
VelocityLimitOptions make_velocity_limit_options(const TrajectoryModifierData & data);

/// @brief Distance needed to reach target_velocity from (velocity, acceleration) and end with zero
/// acceleration, braking with at most max_deceleration and max_jerk. Zero when releasing the
/// current acceleration with max_jerk already ends at or below target_velocity.
double jerk_limited_braking_distance(
  double velocity, double acceleration, double target_velocity, double max_deceleration,
  double max_jerk);

/// @brief Limit the velocity of a time-sampled trajectory.
///
/// The callback resolves the limit at a position: one external limit everywhere, or spatially
/// varying map limits. When any point exceeds its limit, the profile is regenerated from the ego
/// state (the first point when no ego state is given) by a forward simulation that never violates
/// the constraints. At every arc length it stays at or below the input velocity and the limit,
/// except where reaching them in time would violate the constraints; such a limit is exceeded
/// temporarily. Braking for a lower limit ahead starts when 95% of the deceleration limit is needed
/// to reach it where it begins, and the profile does not accelerate above the lowest limit ahead.
/// Limit changes between input points are located by querying the callback. A velocity
/// drop at the last input point is ignored, since planners commonly end their horizon with a zero
/// velocity that is not a stop. A measured acceleration outside the constraints is clamped into
/// them. The first pose, the point count and every timestamp are kept, and the other poses are
/// re-sampled along the input polyline to match the new profile.
VelocityLimitResult apply_velocity_limits(
  TrajectoryPoints & points, const VelocityLimitConstraints & constraints,
  const std::function<std::optional<double>(const geometry_msgs::msg::Point &)> & velocity_limit,
  const VelocityLimitOptions & options = {});

}  // namespace autoware::trajectory_modifier::plugin::detail

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__VELOCITY_LIMITS_HPP_
