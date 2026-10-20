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

#include "autoware/trajectory_modifier/trajectory_modifier_context.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_plugins/external_velocity_limit.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_plugins/velocity_limits.hpp"

#include <rclcpp/duration.hpp>

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit_clear_command.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace
{
using autoware::trajectory_modifier::ExternalVelocityLimitState;
using autoware::trajectory_modifier::plugin::ProcessingResult;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware::trajectory_modifier::plugin::detail::apply_velocity_limits;
using autoware::trajectory_modifier::plugin::detail::get_external_velocity_limit_deceleration;
using autoware::trajectory_modifier::plugin::detail::get_external_velocity_limit_min_jerk;
using autoware::trajectory_modifier::plugin::detail::VelocityLimitConstraints;
using autoware::trajectory_modifier::plugin::detail::VelocityLimitOptions;
using autoware_internal_planning_msgs::msg::VelocityLimit;
using autoware_internal_planning_msgs::msg::VelocityLimitClearCommand;

VelocityLimit::ConstSharedPtr make_limit(const float speed, const std::string & sender)
{
  auto limit = std::make_shared<VelocityLimit>();
  limit->max_velocity = speed;
  limit->sender = sender;
  return limit;
}

VelocityLimitClearCommand::ConstSharedPtr make_clear(
  const std::string & sender, const bool command = true)
{
  auto clear = std::make_shared<VelocityLimitClearCommand>();
  clear->command = command;
  clear->sender = sender;
  return clear;
}

TEST(ExternalVelocityLimitState, ClearOnlyRemovesMatchingActiveSender)
{
  ExternalVelocityLimitState state;
  state.update(make_limit(5.0F, "external"), nullptr);
  ASSERT_TRUE(state.active_limit());

  state.update(nullptr, make_clear("other"));
  state.update(nullptr, make_clear("external", false));
  ASSERT_TRUE(state.active_limit());
  EXPECT_EQ(state.clear_revision(), 0U);

  state.update(nullptr, make_clear("external"));
  EXPECT_FALSE(state.active_limit());
  EXPECT_EQ(state.clear_revision(), 1U);

  state.update(nullptr, make_clear("external"));
  EXPECT_EQ(state.clear_revision(), 1U);
  state.update(make_limit(4.0F, "external"), nullptr);
  ASSERT_TRUE(state.active_limit());
  EXPECT_FLOAT_EQ(state.active_limit()->max_velocity, 4.0F);
}

TEST(ExternalVelocityLimitState, ClearWinsUnknownOrOlderLimitEvents)
{
  ExternalVelocityLimitState state;
  state.update(make_limit(5.0F, "external"), make_clear("external"));
  EXPECT_FALSE(state.active_limit());

  const rclcpp::Time limit_time{10, 0, RCL_ROS_TIME};
  const rclcpp::Time clear_time{11, 0, RCL_ROS_TIME};
  state.update(make_limit(5.0F, "external"), make_clear("external"), limit_time, clear_time);
  EXPECT_FALSE(state.active_limit());
  state.update(make_limit(5.0F, "external"), nullptr, limit_time);
  EXPECT_FALSE(state.active_limit());

  state.update(make_limit(4.0F, "external"), nullptr, rclcpp::Time{12, 0, RCL_ROS_TIME});
  ASSERT_TRUE(state.active_limit());
  EXPECT_FLOAT_EQ(state.active_limit()->max_velocity, 4.0F);
  state.update(nullptr, make_clear("external"), std::nullopt, clear_time);
  EXPECT_TRUE(state.active_limit());
}

TEST(ExternalVelocityLimitState, NewerLimitAfterQueuedClearReactivates)
{
  ExternalVelocityLimitState state;
  state.update(
    make_limit(4.0F, "external"), make_clear("external"), rclcpp::Time{12, 0, RCL_ROS_TIME},
    rclcpp::Time{11, 0, RCL_ROS_TIME});
  ASSERT_TRUE(state.active_limit());
  EXPECT_FLOAT_EQ(state.active_limit()->max_velocity, 4.0F);
}

TEST(ExternalVelocityLimitState, OtherSendersClearDoesNotForgetEarlierClear)
{
  ExternalVelocityLimitState state;
  state.update(make_limit(5.0F, "external"), nullptr, rclcpp::Time{9, 0, RCL_ROS_TIME});
  state.update(nullptr, make_clear("external"), std::nullopt, rclcpp::Time{10, 0, RCL_ROS_TIME});
  state.update(nullptr, make_clear("other"), std::nullopt, rclcpp::Time{11, 0, RCL_ROS_TIME});
  state.update(make_limit(5.0F, "external"), nullptr, rclcpp::Time{9, 0, RCL_ROS_TIME});
  EXPECT_FALSE(state.active_limit());
}

auto constant_limit(const double velocity)
{
  return [velocity](const geometry_msgs::msg::Point &) { return std::optional<double>{velocity}; };
}

TrajectoryPoints make_constant_speed_trajectory(
  const std::vector<double> & times, const double velocity)
{
  TrajectoryPoints points(times.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto & point = points[i];
    point.pose.position.x = velocity * times[i];
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = static_cast<float>(velocity);
    point.time_from_start = rclcpp::Duration::from_seconds(times[i]);
  }
  return points;
}

VelocityLimitOptions make_options(const double velocity, const double acceleration)
{
  VelocityLimitOptions options;
  options.current_ego_velocity = velocity;
  options.current_ego_acceleration = acceleration;
  return options;
}

/// @brief Point accelerations within the deceleration bound and changing no faster than the jerk.
void expect_within_constraints(
  const TrajectoryPoints & points, const std::vector<double> & times,
  const VelocityLimitConstraints & constraints, const double initial_acceleration)
{
  constexpr double tolerance = 1e-4;
  double previous_acceleration = initial_acceleration;
  double previous_time = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double acceleration = points[i].acceleration_mps2;
    EXPECT_GE(acceleration, -constraints.max_deceleration - tolerance) << i;
    EXPECT_LE(
      std::abs(acceleration - previous_acceleration),
      constraints.max_jerk * (times[i] - previous_time) + tolerance)
      << i;
    previous_acceleration = acceleration;
    previous_time = times[i];
  }
}

TEST(ExternalVelocityLimit, UsesMessageMinimumAccelerationWhenProvided)
{
  VelocityLimit limit;
  limit.use_constraints = true;
  limit.constraints.min_acceleration = -2.5F;

  EXPECT_DOUBLE_EQ(get_external_velocity_limit_deceleration(limit, 1.0), 2.5);
}

TEST(ExternalVelocityLimit, UsesDefaultDecelerationWithoutConstraints)
{
  VelocityLimit limit;
  limit.use_constraints = false;

  EXPECT_DOUBLE_EQ(get_external_velocity_limit_deceleration(limit, -1.5), 1.5);
}

TEST(ExternalVelocityLimit, UsesDefaultDecelerationForZeroConstraint)
{
  VelocityLimit limit;
  limit.use_constraints = true;
  limit.constraints.min_acceleration = 0.0F;

  EXPECT_DOUBLE_EQ(get_external_velocity_limit_deceleration(limit, 1.5), 1.5);
}

TEST(ExternalVelocityLimit, UsesMessageMinimumJerkWhenProvided)
{
  VelocityLimit limit;
  limit.use_constraints = true;
  limit.constraints.min_jerk = -0.75F;

  EXPECT_DOUBLE_EQ(get_external_velocity_limit_min_jerk(limit, 1.5), 0.75);
}

TEST(ExternalVelocityLimit, UsesDefaultJerkWithoutConstraints)
{
  VelocityLimit limit;
  limit.use_constraints = false;

  EXPECT_DOUBLE_EQ(get_external_velocity_limit_min_jerk(limit, -1.5), 1.5);
}

TEST(ExternalVelocityLimit, UsesDefaultJerkForZeroConstraint)
{
  VelocityLimit limit;
  limit.use_constraints = true;
  limit.constraints.min_jerk = 0.0F;

  EXPECT_DOUBLE_EQ(get_external_velocity_limit_min_jerk(limit, 1.5), 1.5);
}

TEST(ExternalVelocityLimit, AppliesOneLimitToEveryTrajectoryPoint)
{
  std::vector<double> times;
  for (std::size_t i = 0; i < 20; ++i) {
    times.push_back(0.1 * static_cast<double>(i + 1));
  }
  auto points = make_constant_speed_trajectory(times, 10.0);
  const auto original = points;
  constexpr double max_velocity = 4.0;

  const auto result = apply_velocity_limits(
    points, VelocityLimitConstraints{1.0, 1.0, 0.5}, constant_limit(max_velocity),
    make_options(max_velocity, 0.0));

  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  ASSERT_EQ(points.size(), original.size());
  EXPECT_EQ(points.front().pose, original.front().pose);
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_NEAR(points[i].longitudinal_velocity_mps, max_velocity, 1e-5) << i;
    EXPECT_EQ(points[i].time_from_start, original[i].time_from_start);
  }
}

TEST(ExternalVelocityLimit, AppliesJerkLimitedProfileFromEgoStateUsingTimestamps)
{
  const std::vector<double> times{0.1, 0.25, 1.0, 2.5, 3.0, 3.5, 5.0, 8.0};
  auto points = make_constant_speed_trajectory(times, 10.0);
  constexpr double target_velocity = 4.0;
  const VelocityLimitConstraints constraints{1.0, 2.0, 1.0};

  const auto result = apply_velocity_limits(
    points, constraints, constant_limit(target_velocity), make_options(10.0, 0.0));

  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  expect_within_constraints(points, times, constraints, 0.0);
  double previous_velocity = 10.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double velocity = points[i].longitudinal_velocity_mps;
    EXPECT_LE(velocity, previous_velocity + 1e-6) << i;
    EXPECT_GE(velocity, target_velocity - 1e-2) << i;
    previous_velocity = velocity;
  }
  EXPECT_NEAR(points.back().longitudinal_velocity_mps, target_velocity, 1e-2);
  EXPECT_NEAR(points.back().acceleration_mps2, 0.0, 1e-2);
}

TEST(ExternalVelocityLimit, RespectsMaximumJerkFromCurrentEgoAcceleration)
{
  const std::vector<double> times{0.1, 0.2, 0.3, 0.4};
  auto points = make_constant_speed_trajectory(times, 10.0);
  const VelocityLimitConstraints constraints{1.0, 2.0, 0.5};

  const auto result =
    apply_velocity_limits(points, constraints, constant_limit(4.0), make_options(10.0, 1.0));

  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  expect_within_constraints(points, times, constraints, 1.0);
  // The positive acceleration cannot vanish at once, so the velocity keeps rising at first.
  double previous_velocity = 10.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_GT(points[i].longitudinal_velocity_mps, previous_velocity) << i;
    previous_velocity = points[i].longitudinal_velocity_mps;
  }
}

}  // namespace
