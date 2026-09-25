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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/external_velocity_limit.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_plugins/velocity_limits.hpp"

#include <rclcpp/duration.hpp>

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <vector>

namespace
{
using autoware::trajectory_modifier::plugin::ProcessingResult;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware::trajectory_modifier::plugin::detail::apply_velocity_limits;
using autoware::trajectory_modifier::plugin::detail::get_external_velocity_limit_deceleration;
using autoware::trajectory_modifier::plugin::detail::VelocityLimitOptions;
using autoware_internal_planning_msgs::msg::VelocityLimit;

TEST(ExternalVelocityLimit, UsesMessageMinimumAccelerationWhenProvided)
{
  VelocityLimit limit;
  limit.use_constraints = true;
  limit.constraints.min_acceleration = -2.5F;

  EXPECT_DOUBLE_EQ(get_external_velocity_limit_deceleration(limit, 1.0), 2.5);
}

TEST(ExternalVelocityLimit, UsesNominalDecelerationWithoutConstraints)
{
  VelocityLimit limit;
  limit.use_constraints = false;

  EXPECT_DOUBLE_EQ(get_external_velocity_limit_deceleration(limit, -1.5), 1.5);
}

TEST(ExternalVelocityLimit, AppliesOneLimitToEveryTrajectoryPoint)
{
  TrajectoryPoints points(20);
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto & point = points[i];
    point.pose.position.x = static_cast<double>(i);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 10.0F;
    point.time_from_start = rclcpp::Duration::from_seconds(0.1 * static_cast<double>(i));
  }
  const auto original = points;
  constexpr double max_velocity = 4.0;

  const auto result =
    apply_velocity_limits(points, 1.0, [max_velocity](const geometry_msgs::msg::Point &) {
      return std::optional<double>{max_velocity};
    });

  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  ASSERT_EQ(points.size(), original.size());
  EXPECT_EQ(points.front().pose, original.front().pose);
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_FLOAT_EQ(points[i].longitudinal_velocity_mps, static_cast<float>(max_velocity));
    EXPECT_EQ(points[i].time_from_start, original[i].time_from_start);
  }
}

TEST(ExternalVelocityLimit, MakesProfileFeasibleFromEgoVelocityUsingTimestamps)
{
  const std::vector<double> times{0.1, 0.25, 1.0, 2.5, 3.0, 3.5};
  TrajectoryPoints points(times.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto & point = points[i];
    point.pose.position.x = 10.0 * times[i];
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 10.0F;
    point.time_from_start = rclcpp::Duration::from_seconds(times[i]);
  }

  VelocityLimitOptions options;
  options.make_profile_feasible = true;
  options.current_ego_velocity = 10.0;
  constexpr double target_velocity = 4.0;
  constexpr double deceleration = 2.0;
  const auto result = apply_velocity_limits(
    points, deceleration,
    [target_velocity](const geometry_msgs::msg::Point &) {
      return std::optional<double>{target_velocity};
    },
    options);

  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double expected_velocity =
      std::max(target_velocity, *options.current_ego_velocity - deceleration * times[i]);
    EXPECT_FLOAT_EQ(points[i].longitudinal_velocity_mps, static_cast<float>(expected_velocity));
  }
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double expected_acceleration =
      (points[i + 1].longitudinal_velocity_mps - points[i].longitudinal_velocity_mps) /
      (times[i + 1] - times[i]);
    EXPECT_FLOAT_EQ(points[i].acceleration_mps2, static_cast<float>(expected_acceleration));
  }
  EXPECT_FLOAT_EQ(points.back().acceleration_mps2, 0.0F);
}

TEST(ExternalVelocityLimit, FeasibleProfileNeverRaisesOriginalVelocity)
{
  const std::vector<double> times{0.1, 0.2, 0.3, 0.4};
  const std::vector<float> original_velocities{10.0F, 7.0F, 3.0F, 2.0F};
  TrajectoryPoints points(times.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto & point = points[i];
    point.pose.position.x = static_cast<double>(i);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = original_velocities[i];
    point.time_from_start = rclcpp::Duration::from_seconds(times[i]);
  }

  VelocityLimitOptions options;
  options.make_profile_feasible = true;
  options.current_ego_velocity = 10.0;
  const auto result = apply_velocity_limits(
    points, 1.0, [](const geometry_msgs::msg::Point &) { return std::optional<double>{4.0}; },
    options);

  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_LE(points[i].longitudinal_velocity_mps, original_velocities[i]);
  }
}

}  // namespace
