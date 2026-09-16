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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/velocity_limits.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/interpolation/spherical_linear_interpolation.hpp>
#include <rclcpp/duration.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace autoware::trajectory_modifier::plugin::detail
{

VelocityLimitResult apply_velocity_limits(
  TrajectoryPoints & points, const double deceleration,
  const std::function<std::optional<double>(const geometry_msgs::msg::Point &)> & velocity_limit,
  const VelocityLimitOptions & options)
{
  if (points.empty()) {
    return {};
  }

  if (!std::isfinite(deceleration) || deceleration < 0.0) {
    return VelocityLimitResult{
      ProcessingResult::Unchanged, "Velocity limiting requires non-negative deceleration"};
  }

  if (
    options.make_profile_feasible &&
    (!options.current_ego_velocity || !std::isfinite(*options.current_ego_velocity) ||
     *options.current_ego_velocity < 0.0)) {
    return VelocityLimitResult{
      ProcessingResult::Unchanged,
      "Feasible velocity limiting requires a finite non-negative current ego velocity"};
  }

  const auto original = points;
  const auto count = points.size();
  std::vector<double> times(count, 0.0);
  for (std::size_t i = 0; i < count; ++i) {
    times[i] = rclcpp::Duration(points[i].time_from_start).seconds();
    if (!std::isfinite(times[i]) || times[i] < 0.0 || (i > 0 && times[i] <= times[i - 1])) {
      return VelocityLimitResult{
        ProcessingResult::Unchanged,
        "Velocity limiting requires finite, non-negative, strictly increasing timestamps"};
    }
  }

  std::optional<std::size_t> first_modified_idx = std::nullopt;
  for (std::size_t i = 0; i < count; ++i) {
    const auto limit = velocity_limit(points[i].pose.position);
    if (limit && std::isfinite(*limit) && *limit >= 0.0) {
      if (points[i].longitudinal_velocity_mps > *limit) {
        points[i].longitudinal_velocity_mps = static_cast<float>(*limit);
        points[i].acceleration_mps2 = std::min(points[i].acceleration_mps2, 0.0F);
        if (!first_modified_idx) {
          first_modified_idx = i;
        }
      }
    }
  }

  if (!first_modified_idx) {
    return {ProcessingResult::Unchanged, {}};
  }

  if (options.make_profile_feasible) {
    const double current_ego_velocity = *options.current_ego_velocity;
    for (std::size_t i = 0; i < count; ++i) {
      const double target_velocity = points[i].longitudinal_velocity_mps;
      const double deceleration_profile = current_ego_velocity - deceleration * times[i];
      points[i].longitudinal_velocity_mps = static_cast<float>(std::min(
        static_cast<double>(original[i].longitudinal_velocity_mps),
        std::max(target_velocity, deceleration_profile)));
    }

    for (std::size_t i = 0; i + 1 < count; ++i) {
      const double dt = times[i + 1] - times[i];
      points[i].acceleration_mps2 = static_cast<float>(
        (points[i + 1].longitudinal_velocity_mps - points[i].longitudinal_velocity_mps) / dt);
    }
    points.back().acceleration_mps2 = 0.0F;
  } else {
    // Apply the desired deceleration backward, ignoring current ego velocity feasibility.
    for (int i = static_cast<int>(*first_modified_idx); i >= 0; --i) {
      if (i < static_cast<int>(*first_modified_idx)) {
        const double dt = times[i + 1] - times[i];
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
  }

  std::vector<double> original_s(count, 0.0);
  for (std::size_t i = 1; i < count; ++i) {
    const auto & p0 = original[i - 1].pose.position;
    const auto & p1 = original[i].pose.position;
    original_s[i] = original_s[i - 1] + std::hypot(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
  }

  std::vector<double> new_s(count, 0.0);
  for (std::size_t i = 1; i < count; ++i) {
    const double dt = times[i] - times[i - 1];
    new_s[i] =
      new_s[i - 1] +
      0.5 * (points[i - 1].longitudinal_velocity_mps + points[i].longitudinal_velocity_mps) * dt;
  }

  std::size_t segment = 0;
  for (std::size_t i = 1; i < count; ++i) {
    while (segment + 1 < count && original_s[segment + 1] <= new_s[i]) {
      ++segment;
    }

    if (segment + 1 == count) {
      points[i].pose = original.back().pose;
    } else {
      const double segment_length = original_s[segment + 1] - original_s[segment];
      const double ratio = (new_s[i] - original_s[segment]) / std::max(segment_length, 1e-6);

      const auto & p0 = original[segment].pose;
      const auto & p1 = original[segment + 1].pose;
      auto & pose = points[i].pose;

      pose.position.x = autoware::interpolation::lerp(p0.position.x, p1.position.x, ratio);
      pose.position.y = autoware::interpolation::lerp(p0.position.y, p1.position.y, ratio);
      pose.position.z = autoware::interpolation::lerp(p0.position.z, p1.position.z, ratio);
      pose.orientation =
        autoware::interpolation::lerpOrientation(p0.orientation, p1.orientation, ratio);
    }
  }

  return {ProcessingResult::Modified, {}};
}

}  // namespace autoware::trajectory_modifier::plugin::detail
