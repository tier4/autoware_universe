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

#include "autoware/trajectory_processor/time_sequence_raw/trajectory_optimizer.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <random>

namespace autoware::trajectory_processor::test
{
using autoware::trajectory_processor::time_sequence_raw::opt_dt_s;
using autoware::trajectory_processor::time_sequence_raw::opt_horizon;
using autoware::trajectory_processor::time_sequence_raw::TrajectoryOptimizationParams;
using autoware::trajectory_processor::time_sequence_raw::TrajectoryOptimizer;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;

class TimeSequenceTrajectoryOptimizerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    vehicle_info_.wheel_base_m = 2.75;
    vehicle_info_.max_steer_angle_rad = 0.7;

    odometry_.header.frame_id = "map";
    odometry_.pose.pose.orientation.w = 1.0;
    odometry_.twist.twist.linear.x = 8.0;
  }

  static Trajectory make_noisy_trajectory(
    const double speed, const double noise_std, const float fake_speed = 0.0F)
  {
    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, noise_std);

    Trajectory trajectory;
    trajectory.header.frame_id = "map";
    for (size_t i = 1; i <= opt_horizon; ++i) {
      TrajectoryPoint point;
      point.pose.position.x = speed * opt_dt_s * static_cast<double>(i) + noise(rng);
      point.pose.position.y = noise(rng);
      point.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
      point.longitudinal_velocity_mps = fake_speed;
      point.acceleration_mps2 = 99.0F;
      trajectory.points.push_back(point);
    }
    return trajectory;
  }

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  Odometry odometry_;
};

TEST_F(TimeSequenceTrajectoryOptimizerTest, OptimizesNoisyTrajectory)
{
  TrajectoryOptimizationParams params;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  const auto raw = make_noisy_trajectory(8.0, 0.15);
  const auto result = optimizer.optimize(raw, odometry_, 0.0, 0);

  ASSERT_TRUE(result.optimized) << "acados status: " << result.solver_status;
  ASSERT_EQ(result.trajectory.points.size(), opt_horizon);

  const auto & first = result.trajectory.points.front();
  EXPECT_EQ(first.time_from_start.sec, 0);
  EXPECT_NEAR(first.time_from_start.nanosec * 1e-9, opt_dt_s, 1e-9);
  const double v0 = odometry_.twist.twist.linear.x;
  EXPECT_NEAR(first.pose.position.x, v0 * opt_dt_s, 0.3);
  EXPECT_NEAR(first.pose.position.y, 0.0, 0.3);

  for (const auto & point : result.trajectory.points) {
    EXPECT_GE(point.longitudinal_velocity_mps, params.min_velocity_mps - 1e-6);
    EXPECT_LE(std::abs(point.front_wheel_angle_rad), vehicle_info_.max_steer_angle_rad + 1e-6);
    EXPECT_GE(point.acceleration_mps2, params.min_acceleration_mps2 - 1e-6);
    EXPECT_LE(point.acceleration_mps2, params.max_acceleration_mps2 + 1e-6);
  }

  for (const auto & point : result.trajectory.points) {
    EXPECT_LT(std::abs(point.pose.position.y), 0.5);
  }
}

TEST_F(TimeSequenceTrajectoryOptimizerTest, IgnoresIncomingTrajectorySpeed)
{
  TrajectoryOptimizationParams params;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  const auto raw = make_noisy_trajectory(8.0, 0.05, 99.0F);
  const auto result = optimizer.optimize(raw, odometry_, 0.0, 0);
  ASSERT_TRUE(result.optimized) << "acados status: " << result.solver_status;

  for (const auto & point : result.trajectory.points) {
    EXPECT_LT(std::abs(point.longitudinal_velocity_mps), 20.0F);
    EXPECT_GT(point.longitudinal_velocity_mps, 1.0F);
  }
  EXPECT_NEAR(result.trajectory.points.front().longitudinal_velocity_mps, 8.0F, 2.0F);
}

TEST_F(TimeSequenceTrajectoryOptimizerTest, FallsBackOnShortTrajectory)
{
  TrajectoryOptimizationParams params;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  Trajectory short_trajectory;
  short_trajectory.header.frame_id = "map";
  short_trajectory.points.resize(3);

  const auto result = optimizer.optimize(short_trajectory, odometry_, std::nullopt, 0);
  EXPECT_FALSE(result.optimized);
  EXPECT_EQ(result.trajectory.points.size(), short_trajectory.points.size());
}

TEST_F(TimeSequenceTrajectoryOptimizerTest, WarmStartAcrossCycles)
{
  TrajectoryOptimizationParams params;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  const auto raw = make_noisy_trajectory(8.0, 0.15);
  const auto first = optimizer.optimize(raw, odometry_, 0.0, 0);
  ASSERT_TRUE(first.optimized);

  const auto second = optimizer.optimize(raw, odometry_, 0.0, 0);
  ASSERT_TRUE(second.optimized);
}

}  // namespace autoware::trajectory_processor::test
