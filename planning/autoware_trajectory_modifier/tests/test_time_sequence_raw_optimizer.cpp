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

#include "autoware/trajectory_modifier/time_sequence_raw/trajectory_optimizer.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <random>

namespace autoware::trajectory_modifier::test
{
using autoware::trajectory_modifier::time_sequence_raw::opt_dt_s;
using autoware::trajectory_modifier::time_sequence_raw::opt_horizon;
using autoware::trajectory_modifier::time_sequence_raw::TrajectoryOptimizationParams;
using autoware::trajectory_modifier::time_sequence_raw::TrajectoryOptimizer;
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

  static Trajectory make_straight_trajectory(const double x0, const double speed)
  {
    Trajectory trajectory;
    trajectory.header.frame_id = "map";
    for (size_t i = 1; i <= opt_horizon; ++i) {
      TrajectoryPoint point;
      point.pose.position.x = x0 + speed * opt_dt_s * static_cast<double>(i);
      point.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
      trajectory.points.push_back(point);
    }
    return trajectory;
  }

  static geometry_msgs::msg::Pose make_pose(const double x, const double y, const double yaw = 0.0)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
    return pose;
  }

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  Odometry odometry_;
};

TEST_F(TimeSequenceTrajectoryOptimizerTest, OptimizesNoisyTrajectory)
{
  TrajectoryOptimizationParams params;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  const auto raw = make_noisy_trajectory(8.0, 0.15);
  const auto result = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0);

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
  const auto result = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0);
  ASSERT_TRUE(result.optimized) << "acados status: " << result.solver_status;

  for (const auto & point : result.trajectory.points) {
    EXPECT_LT(std::abs(point.longitudinal_velocity_mps), 20.0F);
    EXPECT_GT(point.longitudinal_velocity_mps, 1.0F);
  }
  // Initial v is odometry twist (~8 m/s), not the fake 99 m/s field.
  EXPECT_NEAR(result.trajectory.points.front().longitudinal_velocity_mps, 8.0F, 2.0F);
}

TEST_F(TimeSequenceTrajectoryOptimizerTest, SeedsSpeedFromOdometryTwist)
{
  TrajectoryOptimizationParams params;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  odometry_.twist.twist.linear.x = 0.0;
  const auto raw = make_noisy_trajectory(8.0, 0.05);
  const auto result = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0);
  ASSERT_TRUE(result.optimized) << "acados status: " << result.solver_status;
  EXPECT_NEAR(result.initial_speed_mps, 0.0, 1e-9);
}

TEST_F(TimeSequenceTrajectoryOptimizerTest, InitialAccelerationContinuity)
{
  TrajectoryOptimizationParams params;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  const auto raw = make_noisy_trajectory(8.0, 0.05);
  constexpr double initial_accel_mps2 = 1.0;
  const auto result = optimizer.optimize(raw, odometry_, 0.0, initial_accel_mps2, 0);
  ASSERT_TRUE(result.optimized) << "acados status: " << result.solver_status;

  EXPECT_NEAR(result.trajectory.points.front().acceleration_mps2, initial_accel_mps2, 0.5F);
}

TEST_F(TimeSequenceTrajectoryOptimizerTest, FallsBackOnShortTrajectory)
{
  TrajectoryOptimizationParams params;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  Trajectory short_trajectory;
  short_trajectory.header.frame_id = "map";
  short_trajectory.points.resize(3);

  const auto result = optimizer.optimize(short_trajectory, odometry_, std::nullopt, 0.0, 0);
  EXPECT_FALSE(result.optimized);
  EXPECT_EQ(result.trajectory.points.size(), short_trajectory.points.size());
}

TEST_F(TimeSequenceTrajectoryOptimizerTest, WarmStartAcrossCycles)
{
  TrajectoryOptimizationParams params;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  const auto raw = make_noisy_trajectory(8.0, 0.15);
  const auto first = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0);
  ASSERT_TRUE(first.optimized);

  const auto second = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0);
  ASSERT_TRUE(second.optimized);
}

TEST_F(TimeSequenceTrajectoryOptimizerTest, ClearWarmStartResetsPreviousSolution)
{
  TrajectoryOptimizationParams params;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  const auto raw = make_noisy_trajectory(8.0, 0.15);
  const auto first = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0);
  ASSERT_TRUE(first.optimized);

  optimizer.clear_warm_start(0);
  const auto second = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0);
  ASSERT_TRUE(second.optimized);
}

TEST_F(TimeSequenceTrajectoryOptimizerTest, TemporalConsistencySucceedsAcrossCycles)
{
  TrajectoryOptimizationParams params;
  params.temporal_consistency.enable = true;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  auto raw = make_noisy_trajectory(8.0, 0.15);
  raw.header.stamp.sec = 0;
  const auto first = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0);
  ASSERT_TRUE(first.optimized);

  raw.header.stamp.nanosec = 100000000;
  const auto second = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0, std::nullopt, false);
  ASSERT_TRUE(second.optimized);
  EXPECT_TRUE(second.temporal_applied);
  EXPECT_STREQ(second.temporal_skip_reason, "none");
}

TEST_F(TimeSequenceTrajectoryOptimizerTest, TemporalConsistencyAppliesWhenEgoIsFarFromGoal)
{
  TrajectoryOptimizationParams params;
  params.temporal_consistency.enable = true;
  params.goal.snap_distance_m = 100.0;
  params.goal.unlatch_horizon_s = 8.0;
  params.goal.unlatch_min_speed_mps = 3.0;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  constexpr double speed = 8.0;
  odometry_.twist.twist.linear.x = speed;
  auto raw = make_straight_trajectory(0.0, speed);
  raw.header.stamp.sec = 0;
  const auto goal = make_pose(100.0, 0.0);

  const auto first = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0, goal);
  ASSERT_TRUE(first.optimized);
  EXPECT_FALSE(first.temporal_applied);
  EXPECT_STREQ(first.temporal_skip_reason, "no_warm_start");

  raw.header.stamp.nanosec = 100000000;
  const auto second = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0, goal);
  ASSERT_TRUE(second.optimized) << "acados status: " << second.solver_status;
  EXPECT_TRUE(second.temporal_applied) << "skip=" << second.temporal_skip_reason;
  EXPECT_STREQ(second.temporal_skip_reason, "none");
}

TEST_F(TimeSequenceTrajectoryOptimizerTest, TemporalConsistencySkippedWhenReferenceShifted)
{
  TrajectoryOptimizationParams params;
  params.temporal_consistency.enable = true;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  auto raw = make_noisy_trajectory(8.0, 0.15);
  const auto first = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0);
  ASSERT_TRUE(first.optimized);

  raw.header.stamp.nanosec = 100000000;
  const auto second = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0, std::nullopt, true);
  ASSERT_TRUE(second.optimized);
  EXPECT_FALSE(second.temporal_applied);
  EXPECT_STREQ(second.temporal_skip_reason, "border_shift");
}

TEST_F(TimeSequenceTrajectoryOptimizerTest, SnapsTerminalWhenEndpointNearGoal)
{
  TrajectoryOptimizationParams params;
  params.temporal_consistency.enable = false;
  params.goal.snap_distance_m = 1.0;
  params.goal.unlatch_horizon_s = 20.0;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  constexpr double x0 = 50.0;
  constexpr double speed = 8.0;
  odometry_.pose.pose.position.x = x0;
  odometry_.twist.twist.linear.x = speed;
  const auto raw = make_straight_trajectory(x0, speed);
  const auto & terminal = raw.points.back().pose.position;
  const auto goal = make_pose(terminal.x + 0.4, 0.3);

  const auto result = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0, goal);
  ASSERT_TRUE(result.optimized) << "acados status: " << result.solver_status;
  EXPECT_NEAR(result.trajectory.points.back().pose.position.x, goal.position.x, 0.25);
  EXPECT_NEAR(result.trajectory.points.back().pose.position.y, goal.position.y, 0.25);
}

TEST_F(TimeSequenceTrajectoryOptimizerTest, TemporalConsistencyAppliesWhenGoalSnapIsLatched)
{
  TrajectoryOptimizationParams params;
  params.temporal_consistency.enable = true;
  params.goal.snap_distance_m = 1.0;
  params.goal.unlatch_horizon_s = 20.0;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  constexpr double x0 = 50.0;
  constexpr double speed = 8.0;
  odometry_.pose.pose.position.x = x0;
  odometry_.twist.twist.linear.x = speed;
  auto raw = make_straight_trajectory(x0, speed);
  const auto & terminal = raw.points.back().pose.position;
  const auto goal = make_pose(terminal.x + 0.2, 0.0);

  const auto first = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0, goal);
  ASSERT_TRUE(first.optimized);
  EXPECT_TRUE(first.goal_snap_active);
  EXPECT_FALSE(first.temporal_applied);
  EXPECT_STREQ(first.temporal_skip_reason, "no_warm_start");

  raw.header.stamp.nanosec = 100000000;
  const auto second = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0, goal);
  ASSERT_TRUE(second.optimized);
  EXPECT_TRUE(second.goal_snap_active);
  EXPECT_TRUE(second.temporal_applied) << "skip=" << second.temporal_skip_reason;
  EXPECT_STREQ(second.temporal_skip_reason, "none");
}

TEST_F(TimeSequenceTrajectoryOptimizerTest, TimeSampledTemporalCoversTakeoff)
{
  TrajectoryOptimizationParams params;
  params.temporal_consistency.enable = true;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  odometry_.twist.twist.linear.x = 0.0;
  auto stopped = make_straight_trajectory(0.0, 0.0);
  stopped.header.stamp.sec = 0;
  const auto first = optimizer.optimize(stopped, odometry_, 0.0, 0.0, 0);
  ASSERT_TRUE(first.optimized);

  odometry_.twist.twist.linear.x = 8.0;
  auto moving = make_straight_trajectory(0.0, 8.0);
  moving.header.stamp.nanosec = 100000000;
  const auto second = optimizer.optimize(moving, odometry_, 0.0, 0.0, 0);
  ASSERT_TRUE(second.optimized);
  EXPECT_TRUE(second.temporal_applied);
  EXPECT_EQ(second.temporal_valid_stages, opt_horizon);
  EXPECT_STREQ(second.temporal_skip_reason, "none");
}

TEST_F(TimeSequenceTrajectoryOptimizerTest, DoesNotLatchGoalSnapWhenEgoIsFar)
{
  TrajectoryOptimizationParams params;
  params.temporal_consistency.enable = false;
  params.goal.snap_distance_m = 100.0;
  params.goal.unlatch_horizon_s = 8.0;
  params.goal.unlatch_min_speed_mps = 3.0;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  constexpr double speed = 8.0;
  odometry_.twist.twist.linear.x = speed;
  const auto raw = make_straight_trajectory(0.0, speed);
  const auto goal = make_pose(100.0, 0.0);

  const auto result = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0, goal);
  ASSERT_TRUE(result.optimized) << "acados status: " << result.solver_status;
  // Horizon length is 8 s * 8 m/s = 64 m. Without snap the terminal stays near 64;
  // a latched snap with snap_distance_m=100 would pull it toward the goal at 100.
  EXPECT_NEAR(result.trajectory.points.back().pose.position.x, 64.0, 3.0);
}

TEST_F(TimeSequenceTrajectoryOptimizerTest, UnlatchesGoalSnapWhenEgoDrivesAway)
{
  TrajectoryOptimizationParams params;
  params.temporal_consistency.enable = false;
  params.goal.snap_distance_m = 1.0;
  params.goal.unlatch_horizon_s = 20.0;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  constexpr double x0 = 50.0;
  constexpr double speed = 8.0;
  odometry_.pose.pose.position.x = x0;
  odometry_.twist.twist.linear.x = speed;
  auto raw = make_straight_trajectory(x0, speed);
  const auto & terminal = raw.points.back().pose.position;
  const auto goal = make_pose(terminal.x + 0.2, 0.0);

  const auto latched = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0, goal);
  ASSERT_TRUE(latched.optimized);
  EXPECT_NEAR(latched.trajectory.points.back().pose.position.x, goal.position.x, 0.25);

  constexpr double far_x0 = -200.0;
  odometry_.pose.pose.position.x = far_x0;
  raw = make_straight_trajectory(far_x0, speed);
  raw.header.stamp.nanosec = 100000000;
  const auto unlatched = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0, goal);
  ASSERT_TRUE(unlatched.optimized) << "acados status: " << unlatched.solver_status;
  EXPECT_NEAR(
    unlatched.trajectory.points.back().pose.position.x, far_x0 + speed * opt_dt_s * opt_horizon,
    2.0);
}

TEST_F(TimeSequenceTrajectoryOptimizerTest, GoalPositionChangeClearsPreviousSolutions)
{
  TrajectoryOptimizationParams params;
  params.temporal_consistency.enable = true;
  params.goal.snap_distance_m = 1.0;
  params.goal.unlatch_horizon_s = 20.0;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  constexpr double x0 = 50.0;
  constexpr double speed = 8.0;
  odometry_.pose.pose.position.x = x0;
  odometry_.twist.twist.linear.x = speed;
  auto raw = make_straight_trajectory(x0, speed);
  raw.header.stamp.sec = 0;
  const auto & terminal = raw.points.back().pose.position;
  const auto goal_a = make_pose(terminal.x + 0.2, 0.0);

  const auto first = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0, goal_a);
  ASSERT_TRUE(first.optimized);
  EXPECT_TRUE(first.goal_snap_active);

  raw.header.stamp.nanosec = 100000000;
  const auto second = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0, goal_a);
  ASSERT_TRUE(second.optimized);
  EXPECT_TRUE(second.goal_snap_active);
  EXPECT_TRUE(second.temporal_applied);

  // Nearby new goal still snap-eligible. Clearing previous_solutions_ is required so the
  // old latched plan is not reused as a temporal reference under a matching latch flag.
  const auto goal_b = make_pose(terminal.x + 0.5, 0.0);
  raw.header.stamp.nanosec = 200000000;
  const auto changed = optimizer.optimize(raw, odometry_, 0.0, 0.0, 0, goal_b);
  ASSERT_TRUE(changed.optimized) << "acados status: " << changed.solver_status;
  EXPECT_TRUE(changed.goal_snap_active);
  EXPECT_FALSE(changed.temporal_applied);
  EXPECT_STREQ(changed.temporal_skip_reason, "no_warm_start");
}

}  // namespace autoware::trajectory_modifier::test
