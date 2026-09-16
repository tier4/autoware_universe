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

#include <optional>

namespace
{
using autoware::trajectory_processor::plugin::ProcessingResult;
using autoware::trajectory_processor::plugin::TrajectoryPoints;
using autoware::trajectory_processor::plugin::detail::apply_velocity_limits;
using autoware::trajectory_processor::plugin::detail::get_external_velocity_limit_deceleration;
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

}  // namespace
