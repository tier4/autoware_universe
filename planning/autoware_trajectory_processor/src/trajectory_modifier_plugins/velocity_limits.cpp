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

#include "autoware/trajectory_processor/trajectory_modifier_plugins/velocity_limits.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/interpolation/spherical_linear_interpolation.hpp>
#include <rclcpp/duration.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace autoware::trajectory_processor::plugin::detail
{

VelocityLimitResult apply_velocity_limits(
  TrajectoryPoints & points, const double deceleration, const double max_jerk,
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

  // 0. LIMIT ASSIGNMENT
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
  // Skip smoothing when we already decelerated bellow the limit
  if (options.current_ego_velocity) {
    constexpr auto deadband_ratio = 0.15;
    const auto deadband_threshold = (1.0 - deadband_ratio) * *options.current_ego_velocity;
    if (points[*first_modified_idx].longitudinal_velocity_mps > deadband_threshold) {
      return {ProcessingResult::Modified, {}};
    }
  }

  // Use original.front() in case the limit assignment above overwrote the ego starting point
  const double ego_v =
    options.current_ego_velocity.value_or(original.front().longitudinal_velocity_mps);
  const double ego_a =
    options.current_ego_acceleration.value_or(original.front().acceleration_mps2);

  // 1. FORWARD REACHABILITY PASS (Ego-Awareness)
  // Embeds the ego vehicle's current state and kinematic limits into the trajectory.
  // This physically forces the trajectory to encode any inevitable undershoot/velocity loss
  // caused by existing negative acceleration and jerk limits.
  double reach_v = ego_v;
  double reach_a = ego_a;
  double last_time_reach = 0.0;

  for (std::size_t i = 0; i < count; ++i) {
    const double dt = times[i] - last_time_reach;
    if (dt < 1e-6) {
      points[i].longitudinal_velocity_mps = static_cast<float>(std::clamp(
        static_cast<double>(points[i].longitudinal_velocity_mps), 0.0, std::max(0.0, reach_v)));
      continue;
    }

    // Determine the absolute kinematic limits from the current state
    const double a_min_kin = reach_a - max_jerk * dt;
    const double a_max_kin = reach_a + max_jerk * dt;

    const double min_possible_a = std::max(-deceleration, a_min_kin);
    const double max_possible_a = a_max_kin;

    // Bounds for what velocity the vehicle can actually reach at this step
    const double min_reach_v = std::max(0.0, reach_v + min_possible_a * dt);
    const double max_reach_v = std::max(0.0, reach_v + max_possible_a * dt);

    const double clamp_min = std::min(min_reach_v, max_reach_v);
    const double clamp_max = std::max(min_reach_v, max_reach_v);

    // Apply the reachability envelope to the trajectory point
    points[i].longitudinal_velocity_mps = static_cast<float>(
      std::clamp(static_cast<double>(points[i].longitudinal_velocity_mps), clamp_min, clamp_max));

    // Update the simulation state based on the clamped velocity
    const double required_a = (points[i].longitudinal_velocity_mps - reach_v) / dt;
    reach_a = std::clamp(required_a, min_possible_a, max_possible_a);
    reach_v = points[i].longitudinal_velocity_mps;
    last_time_reach = times[i];
  }

  // 2. BACKWARD PASS (Safety & Smoothing)
  // Ensures decelerations to meet future limits are smooth.
  // Crucially, it now smoothly connects backward into the exact ego-state envelope
  // generated by the Reachability Pass, naturally creating a jerk-limited ramp OUT of deceleration.
  for (int i = static_cast<int>(count) - 2; i >= 0; --i) {
    const double dt = times[i + 1] - times[i];
    const double a_lower_bound = points[i + 1].acceleration_mps2 - max_jerk * dt;
    const double target_a = std::max(-deceleration, a_lower_bound);
    const double max_safe_v = points[i + 1].longitudinal_velocity_mps - target_a * dt;

    if (points[i].longitudinal_velocity_mps > max_safe_v) {
      points[i].longitudinal_velocity_mps = static_cast<float>(max_safe_v);
      points[i].acceleration_mps2 = static_cast<float>(target_a);
    } else {
      points[i].acceleration_mps2 = static_cast<float>(
        (points[i + 1].longitudinal_velocity_mps - points[i].longitudinal_velocity_mps) / dt);
    }
  }

  // 3. GENERALIZED FORWARD PASS (Tracking)
  // Simply tracks the mathematically feasible profile cleanly.
  double current_v = ego_v;
  double current_a = ego_a;
  double last_time = 0.0;

  for (std::size_t i = 0; i < count; ++i) {
    const double dt = times[i] - last_time;
    if (dt < 1e-6) {
      points[i].longitudinal_velocity_mps = static_cast<float>(current_v);
      if (i > 0) {
        points[i - 1].acceleration_mps2 = static_cast<float>(current_a);
      }
      continue;
    }

    const double a_min_kin = current_a - max_jerk * dt;
    const double a_max_kin = current_a + max_jerk * dt;
    const double required_a = (points[i].longitudinal_velocity_mps - current_v) / dt;

    double clamped_a = std::clamp(required_a, a_min_kin, a_max_kin);
    clamped_a = std::max(-deceleration, clamped_a);

    current_v = current_v + clamped_a * dt;
    // prevent negative velocities
    points[i].longitudinal_velocity_mps = std::max(0.0f, static_cast<float>(current_v));

    if (i > 0) {
      points[i - 1].acceleration_mps2 = static_cast<float>(clamped_a);
    }

    current_a = clamped_a;
    last_time = times[i];
  }

  points.back().acceleration_mps2 = 0.0F;

  // 4. SPATIAL INTERPOLATION
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

}  // namespace autoware::trajectory_processor::plugin::detail
