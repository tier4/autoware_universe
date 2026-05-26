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

#include "in_lane_mrm_trajectory_validator.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <gtest/gtest.h>

namespace autoware::in_lane_mrm_planner
{
namespace
{

Params make_default_params()
{
  Params params;
  params.trajectory_validator.enable = true;
  params.trajectory_validator.min_point_count = 3;
  params.trajectory_validator.min_point_spacing_m = 0.05;
  params.trajectory_validator.min_trajectory_length_m = 1.0;
  params.trajectory_validator.standstill_velocity_threshold_mps = 0.01;
  params.trajectory_validator.hazardous_leading_velocity_mps = 0.5;
  return params;
}

TrajectoryPoints make_straight_trajectory(
  const size_t num_points, const double spacing, const float velocity)
{
  TrajectoryPoints points;
  points.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = spacing * static_cast<double>(i);
    point.pose.position.y = 0.0;
    point.pose.position.z = 0.0;
    point.pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
    point.longitudinal_velocity_mps = velocity;
    point.acceleration_mps2 = 0.0F;
    points.push_back(point);
  }
  return points;
}

Odometry make_odometry(const double velocity)
{
  Odometry odom;
  odom.pose.pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
  odom.twist.twist.linear.x = velocity;
  return odom;
}

}  // namespace

TEST(InLaneMrmTrajectoryValidatorTest, AcceptsUniformLowSpeedTrajectory)
{
  const InLaneMrmTrajectoryValidator validator(make_default_params());
  const auto points = make_straight_trajectory(10, 1.0, 0.0F);

  const auto result = validator.validate(points, make_odometry(0.0));
  EXPECT_TRUE(result.ok);
}

TEST(InLaneMrmTrajectoryValidatorTest, RejectsHazardousVelocityStepProfile)
{
  const InLaneMrmTrajectoryValidator validator(make_default_params());
  auto points = make_straight_trajectory(10, 1.0, 0.0F);
  points.front().longitudinal_velocity_mps = 16.67F;

  const auto result = validator.validate(points, make_odometry(0.0));
  EXPECT_FALSE(result.ok);
}

TEST(InLaneMrmTrajectoryValidatorTest, RejectsNonZeroVelocityAtStandstill)
{
  const InLaneMrmTrajectoryValidator validator(make_default_params());
  auto points = make_straight_trajectory(10, 1.0, 5.0F);

  const auto result = validator.validate(points, make_odometry(0.0));
  EXPECT_FALSE(result.ok);
}

TEST(InLaneMrmTrajectoryValidatorTest, RejectsInsufficientPointCount)
{
  const InLaneMrmTrajectoryValidator validator(make_default_params());
  const auto points = make_straight_trajectory(2, 1.0, 0.0F);

  const auto result = validator.validate(points, make_odometry(0.0));
  EXPECT_FALSE(result.ok);
}

}  // namespace autoware::in_lane_mrm_planner
