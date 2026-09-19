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

#include "autoware/trajectory_processor/trajectory_modifier_plugins/external_velocity_limit.hpp"
#include "autoware/trajectory_processor/trajectory_modifier_plugins/velocity_limits.hpp"

#include <rclcpp/duration.hpp>

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <vector>

namespace
{
using autoware::trajectory_processor::plugin::ProcessingResult;
using autoware::trajectory_processor::plugin::TrajectoryPoints;
using autoware::trajectory_processor::plugin::detail::apply_velocity_limits;
using autoware::trajectory_processor::plugin::detail::get_external_velocity_limit_deceleration;
using autoware::trajectory_processor::plugin::detail::get_external_velocity_limit_min_jerk;
using autoware::trajectory_processor::plugin::detail::VelocityLimitOptions;
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

TEST(ExternalVelocityLimit, UsesMessageMinimumJerkWhenProvided)
{
  VelocityLimit limit;
  limit.use_constraints = true;
  limit.constraints.min_jerk = -0.75F;

  EXPECT_DOUBLE_EQ(get_external_velocity_limit_min_jerk(limit, 1.5), 0.75);
}

TEST(ExternalVelocityLimit, UsesNominalJerkWithoutConstraints)
{
  VelocityLimit limit;
  limit.use_constraints = false;

  EXPECT_DOUBLE_EQ(get_external_velocity_limit_min_jerk(limit, -1.5), 1.5);
}

TEST(ExternalVelocityLimit, AppliesOneLimitToEveryTrajectoryPoint)
{
  TrajectoryPoints points(20);
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto & point = points[i];
    point.pose.position.x = static_cast<double>(i);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 10.0F;
    point.time_from_start = rclcpp::Duration::from_seconds(0.1 * static_cast<double>(i + 1));
  }
  const auto original = points;
  constexpr double max_velocity = 4.0;

  const auto result =
    apply_velocity_limits(points, 1.0, 0.5, [max_velocity](const geometry_msgs::msg::Point &) {
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

TEST(ExternalVelocityLimit, AppliesJerkLimitedProfileFromEgoStateUsingTimestamps)
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
  options.current_ego_velocity = 10.0;
  options.current_ego_acceleration = 0.0;
  constexpr double target_velocity = 4.0;
  constexpr double deceleration = 2.0;
  constexpr double max_jerk = 1.0;
  const auto result = apply_velocity_limits(
    points, deceleration, max_jerk,
    [target_velocity](const geometry_msgs::msg::Point &) {
      return std::optional<double>{target_velocity};
    },
    options);

  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  const std::vector<double> expected_velocities{9.99, 9.9525, 9.2025, 6.2025, 5.2025, 4.2025};
  const std::vector<double> expected_accelerations{-0.25, -1.0, -2.0, -2.0, -2.0, 0.0};
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_NEAR(points[i].longitudinal_velocity_mps, expected_velocities[i], 1e-5) << i;
    EXPECT_NEAR(points[i].acceleration_mps2, expected_accelerations[i], 1e-5) << i;
  }
}

TEST(ExternalVelocityLimit, RespectsMaximumJerkFromCurrentEgoAcceleration)
{
  const std::vector<double> times{0.1, 0.2, 0.3, 0.4};
  TrajectoryPoints points(times.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto & point = points[i];
    point.pose.position.x = static_cast<double>(i);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 10.0F;
    point.time_from_start = rclcpp::Duration::from_seconds(times[i]);
  }

  VelocityLimitOptions options;
  options.current_ego_velocity = 10.0;
  options.current_ego_acceleration = 1.0;
  constexpr double max_jerk = 0.5;
  const auto result = apply_velocity_limits(
    points, 2.0, max_jerk,
    [](const geometry_msgs::msg::Point &) { return std::optional<double>{4.0}; }, options);

  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  const std::vector<double> expected_velocities{10.095, 10.185, 10.27, 10.35};
  const std::vector<double> expected_accelerations{0.9, 0.85, 0.8, 0.0};
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_NEAR(points[i].longitudinal_velocity_mps, expected_velocities[i], 1e-5) << i;
    EXPECT_NEAR(points[i].acceleration_mps2, expected_accelerations[i], 1e-5) << i;
  }
}

}  // namespace
