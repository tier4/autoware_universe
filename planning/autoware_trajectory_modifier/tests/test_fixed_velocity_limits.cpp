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

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/duration.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

namespace
{
using autoware::trajectory_modifier::TrajectoryModifierData;
using autoware::trajectory_modifier::plugin::ProcessingResult;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware::trajectory_modifier::plugin::detail::FixedVelocityLimitCache;
using autoware::trajectory_modifier::plugin::detail::FixedVelocityLimitKey;
using autoware::trajectory_modifier::plugin::detail::FixedVelocityLimitParameters;

TrajectoryPoints make_points(const double start_x)
{
  TrajectoryPoints points(60);
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto & point = points[i];
    point.time_from_start = rclcpp::Duration::from_seconds(0.1 * (i + 1));
    point.pose.position.x = start_x + static_cast<double>(i);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 10.0F;
  }
  return points;
}

TrajectoryModifierData make_data(
  const std::int32_t seconds, const std::uint32_t nanoseconds, const double ego_x,
  const double ego_velocity, const double ego_acceleration, const std::uint64_t batch,
  const double ego_y = 0.0, const double ego_yaw = 0.0)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = seconds;
  stamp.nanosec = nanoseconds;
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->header.frame_id = "map";
  odometry->header.stamp = stamp;
  odometry->pose.pose.position.x = ego_x;
  odometry->pose.pose.position.y = ego_y;
  odometry->pose.pose.orientation.z = std::sin(ego_yaw / 2.0);
  odometry->pose.pose.orientation.w = std::cos(ego_yaw / 2.0);
  odometry->twist.twist.linear.x = ego_velocity;
  auto acceleration = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
  acceleration->header.stamp = stamp;
  acceleration->accel.accel.linear.x = ego_acceleration;

  TrajectoryModifierData data;
  data.current_odometry = odometry;
  data.current_acceleration = acceleration;
  data.candidate_header = odometry->header;
  data.candidate_generator_id.uuid[0] = 1;
  data.candidate_count = 1;
  data.candidate_batch_sequence = batch;
  return data;
}

TEST(FixedVelocityLimits, ReusesAnchoredExternalProfileAtLaterTimestamps)
{
  FixedVelocityLimitCache cache;
  const FixedVelocityLimitKey key{std::optional<double>{4.0}, 1.0, 1.0, 0U, false};
  const auto limit = [](const geometry_msgs::msg::Point &) { return std::optional<double>{4.0}; };
  auto first = make_points(0.0);
  auto first_data = make_data(10, 0U, 0.0, 10.0, 0.0, 1U);
  ASSERT_EQ(cache.process(first, first_data, key, limit).status, ProcessingResult::Modified);
  ASSERT_EQ(cache.profile_build_count(), 1U);
  ASSERT_EQ(cache.calculation_count(), 1U);

  auto second = make_points(1.0);
  auto second_data = make_data(
    10, 100000000U, 1.0, first.front().longitudinal_velocity_mps, first.front().acceleration_mps2,
    2U);
  ASSERT_EQ(cache.process(second, second_data, key, limit).status, ProcessingResult::Modified);
  EXPECT_EQ(cache.profile_build_count(), 1U);
  EXPECT_EQ(cache.calculation_count(), 1U);
  EXPECT_NEAR(second.front().longitudinal_velocity_mps, first[1].longitudinal_velocity_mps, 1e-4);
  EXPECT_DOUBLE_EQ(second.front().pose.position.x, 1.0);
}

TEST(FixedVelocityLimits, ConfiguredSpeedToleranceControlsReuseAndClearsOnUpdate)
{
  FixedVelocityLimitCache cache;
  FixedVelocityLimitParameters parameters;
  parameters.speed_deviation_threshold = 0.5;
  cache.set_parameters(parameters);
  const FixedVelocityLimitKey key{std::optional<double>{4.0}, 1.0, 1.0, 0U, false};
  const auto limit = [](const geometry_msgs::msg::Point &) { return std::optional<double>{4.0}; };
  auto first = make_points(0.0);
  const auto first_data = make_data(10, 0U, 0.0, 10.0, 0.0, 1U);
  ASSERT_EQ(cache.process(first, first_data, key, limit).status, ProcessingResult::Modified);
  ASSERT_EQ(cache.calculation_count(), 1U);
  cache.set_parameters(parameters);

  auto second = make_points(1.0);
  const auto second_data = make_data(
    10, 100000000U, 1.0, first.front().longitudinal_velocity_mps + 0.4,
    first.front().acceleration_mps2, 2U);
  ASSERT_EQ(cache.process(second, second_data, key, limit).status, ProcessingResult::Modified);
  EXPECT_EQ(cache.calculation_count(), 1U);

  parameters.speed_deviation_threshold = 0.3;
  cache.set_parameters(parameters);
  auto third = make_points(2.0);
  const auto third_data = make_data(
    10, 200000000U, 2.0, first[1].longitudinal_velocity_mps, first[1].acceleration_mps2, 3U);
  EXPECT_EQ(cache.process(third, third_data, key, limit).status, ProcessingResult::Modified);
  EXPECT_EQ(cache.calculation_count(), 2U);
}

TEST(FixedVelocityLimits, AcceptsFirstTrajectoryPointAheadOfEgo)
{
  FixedVelocityLimitCache cache;
  const FixedVelocityLimitKey key{std::optional<double>{4.0}, 1.0, 1.0, 0U, false};
  const auto limit = [](const geometry_msgs::msg::Point &) { return std::optional<double>{4.0}; };
  auto first = make_points(1.0);
  auto first_data = make_data(10, 0U, 0.0, 10.0, 0.0, 1U);
  ASSERT_EQ(cache.process(first, first_data, key, limit).status, ProcessingResult::Modified);
  ASSERT_EQ(cache.profile_build_count(), 1U);

  auto second = make_points(2.0);
  auto second_data = make_data(
    10, 100000000U, 1.0, first.front().longitudinal_velocity_mps, first.front().acceleration_mps2,
    2U);
  EXPECT_EQ(cache.process(second, second_data, key, limit).status, ProcessingResult::Modified);
  EXPECT_EQ(cache.profile_build_count(), 1U);
  EXPECT_EQ(cache.calculation_count(), 1U);
}

TEST(FixedVelocityLimits, MapZoneMovingBetweenPointIndicesDoesNotRebuild)
{
  FixedVelocityLimitCache cache;
  const FixedVelocityLimitKey key{std::nullopt, 1.0, 1.0, 1U, true};
  const auto limit = [](const geometry_msgs::msg::Point & point) {
    return std::optional<double>{point.x >= 30.0 ? 4.0 : 10.0};
  };
  auto first = make_points(0.0);
  auto first_data = make_data(10, 0U, 0.0, 10.0, 0.0, 1U);
  ASSERT_EQ(cache.process(first, first_data, key, limit).status, ProcessingResult::Modified);
  ASSERT_EQ(cache.profile_build_count(), 1U);

  auto second = make_points(1.0);
  auto second_data = make_data(
    10, 100000000U, 1.0, first.front().longitudinal_velocity_mps, first.front().acceleration_mps2,
    2U);
  ASSERT_EQ(cache.process(second, second_data, key, limit).status, ProcessingResult::Modified);
  EXPECT_EQ(cache.profile_build_count(), 1U);
  EXPECT_EQ(cache.calculation_count(), 1U);
}

TEST(FixedVelocityLimits, LimitChangeAndEgoDeviationRecalculate)
{
  FixedVelocityLimitCache cache;
  const auto limit = [](const geometry_msgs::msg::Point &) { return std::optional<double>{4.0}; };
  auto first = make_points(0.0);
  auto first_data = make_data(10, 0U, 0.0, 10.0, 0.0, 1U);
  const FixedVelocityLimitKey key{std::optional<double>{4.0}, 1.0, 1.0, 0U, false};
  ASSERT_EQ(cache.process(first, first_data, key, limit).status, ProcessingResult::Modified);

  auto changed = make_points(1.0);
  auto second_data = make_data(
    10, 100000000U, 1.0, first.front().longitudinal_velocity_mps, first.front().acceleration_mps2,
    2U);
  const FixedVelocityLimitKey changed_key{std::optional<double>{3.0}, 1.0, 1.0, 0U, false};
  const auto lower_limit = [](const geometry_msgs::msg::Point &) {
    return std::optional<double>{3.0};
  };
  ASSERT_EQ(
    cache.process(changed, second_data, changed_key, lower_limit).status,
    ProcessingResult::Modified);
  EXPECT_EQ(cache.calculation_count(), 2U);

  auto deviated = make_points(2.0);
  auto third_data = make_data(10, 200000000U, 2.0, 8.0, 0.0, 3U);
  EXPECT_EQ(
    cache.process(deviated, third_data, changed_key, lower_limit).status,
    ProcessingResult::Modified);
  EXPECT_EQ(cache.calculation_count(), 3U);
}

TEST(FixedVelocityLimits, NewlyVisibleMapRestrictionRecalculates)
{
  FixedVelocityLimitCache cache;
  const FixedVelocityLimitKey key{std::nullopt, 1.0, 1.0, 1U, true};
  const auto limit = [](const geometry_msgs::msg::Point & point) {
    return std::optional<double>{point.x >= 60.0 ? 2.0 : point.x >= 30.0 ? 4.0 : 10.0};
  };
  auto first = make_points(0.0);
  auto first_data = make_data(10, 0U, 0.0, 10.0, 0.0, 1U);
  ASSERT_EQ(cache.process(first, first_data, key, limit).status, ProcessingResult::Modified);
  ASSERT_EQ(cache.calculation_count(), 1U);

  auto second = make_points(1.0);
  auto second_data = make_data(
    10, 100000000U, 1.0, first.front().longitudinal_velocity_mps, first.front().acceleration_mps2,
    2U);
  EXPECT_EQ(cache.process(second, second_data, key, limit).status, ProcessingResult::Modified);
  EXPECT_EQ(cache.calculation_count(), 2U);
}

TEST(FixedVelocityLimits, MatchesCandidatesWithinSharedGeneratorSeparately)
{
  FixedVelocityLimitCache cache;
  const FixedVelocityLimitKey key{std::optional<double>{4.0}, 1.0, 1.0, 0U, false};
  const auto limit = [](const geometry_msgs::msg::Point &) { return std::optional<double>{4.0}; };
  auto straight = make_points(0.0);
  auto sloped = make_points(0.0);
  for (auto & point : sloped) {
    point.pose.position.y = 0.05 * point.pose.position.x;
  }
  auto first_straight_data = make_data(10, 0U, 0.0, 10.0, 0.0, 1U);
  first_straight_data.candidate_count = 2;
  auto first_sloped_data = first_straight_data;
  first_sloped_data.candidate_index = 1;
  ASSERT_EQ(
    cache.process(straight, first_straight_data, key, limit).status, ProcessingResult::Modified);
  ASSERT_EQ(
    cache.process(sloped, first_sloped_data, key, limit).status, ProcessingResult::Modified);
  ASSERT_EQ(cache.profile_build_count(), 2U);

  auto shifted_sloped = make_points(1.0);
  for (auto & point : shifted_sloped) {
    point.pose.position.y = 0.05 * point.pose.position.x;
  }
  auto shifted_straight = make_points(1.0);
  auto second_sloped_data = make_data(
    10, 100000000U, 1.0, sloped.front().longitudinal_velocity_mps, sloped.front().acceleration_mps2,
    2U, 0.05, std::atan(0.05));
  second_sloped_data.candidate_count = 2;
  auto second_straight_data = second_sloped_data;
  second_straight_data.candidate_index = 1;
  ASSERT_EQ(
    cache.process(shifted_sloped, second_sloped_data, key, limit).status,
    ProcessingResult::Modified);
  ASSERT_EQ(
    cache.process(shifted_straight, second_straight_data, key, limit).status,
    ProcessingResult::Modified);
  EXPECT_EQ(cache.profile_build_count(), 2U);
  EXPECT_EQ(cache.calculation_count(), 2U);
}

TEST(FixedVelocityLimits, UnusableTimestampFallsBackWithoutReusingProfile)
{
  FixedVelocityLimitCache cache;
  const FixedVelocityLimitKey key{std::optional<double>{4.0}, 1.0, 1.0, 0U, false};
  const auto limit = [](const geometry_msgs::msg::Point &) { return std::optional<double>{4.0}; };
  auto first = make_points(0.0);
  auto first_data = make_data(10, 0U, 0.0, 10.0, 0.0, 1U);
  ASSERT_EQ(cache.process(first, first_data, key, limit).status, ProcessingResult::Modified);
  ASSERT_EQ(cache.profile_build_count(), 1U);

  auto second = make_points(1.0);
  auto stale_data = make_data(10, 100000000U, 1.0, 10.0, 0.0, 2U);
  stale_data.candidate_header.stamp.sec = 0;
  stale_data.candidate_header.stamp.nanosec = 0;
  EXPECT_EQ(cache.process(second, stale_data, key, limit).status, ProcessingResult::Modified);
  EXPECT_EQ(cache.calculation_count(), 2U);
  EXPECT_EQ(cache.profile_build_count(), 1U);
}

}  // namespace
