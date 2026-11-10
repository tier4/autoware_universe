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

#include "autoware/ml_planner/optimization/trajectory_optimizer.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <random>

namespace autoware::ml_planner::test
{
using autoware::ml_planner::optimization::opt_dt_s;
using autoware::ml_planner::optimization::opt_horizon;
using autoware::ml_planner::optimization::TrajectoryOptimizationParams;
using autoware::ml_planner::optimization::TrajectoryOptimizer;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;

class TrajectoryOptimizerTest : public ::testing::Test
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

  // Straight-line constant-speed trajectory with position noise, starting at t = 0.1 s
  // slightly ahead of base_link (mimicking the raw model output: poses only).
  static Trajectory make_noisy_trajectory(const double speed, const double noise_std)
  {
    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, noise_std);

    Trajectory trajectory;
    trajectory.header.frame_id = "map";
    for (size_t i = 1; i <= opt_horizon; ++i) {
      TrajectoryPoint point;
      point.pose.position.x = speed * opt_dt_s * static_cast<double>(i) + noise(rng);
      point.pose.position.y = noise(rng);
      point.pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
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
      point.pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
      trajectory.points.push_back(point);
    }
    return trajectory;
  }

  static geometry_msgs::msg::Pose make_pose(const double x, const double y, const double yaw = 0.0)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
    return pose;
  }

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  Odometry odometry_;
};

TEST_F(TrajectoryOptimizerTest, OptimizesNoisyTrajectory)
{
  TrajectoryOptimizationParams params;
  params.enable = true;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  const auto raw = make_noisy_trajectory(8.0, 0.15);
  const auto result = optimizer.optimize(raw, odometry_, 0.0, 0);

  ASSERT_TRUE(result.optimized) << "acados status: " << result.solver_status;
  ASSERT_EQ(result.trajectory.points.size(), opt_horizon);

  // Points are published from t = 0.1 s; the first point follows from the base_link
  // initial state (x0 at origin, v0 = 8 m/s) through the vehicle dynamics.
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

  // The solution should stay close to the (noise-free) straight line.
  for (const auto & point : result.trajectory.points) {
    EXPECT_LT(std::abs(point.pose.position.y), 0.5);
  }
}

TEST_F(TrajectoryOptimizerTest, FallsBackOnShortTrajectory)
{
  TrajectoryOptimizationParams params;
  params.enable = true;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  Trajectory short_trajectory;
  short_trajectory.header.frame_id = "map";
  short_trajectory.points.resize(3);

  const auto result = optimizer.optimize(short_trajectory, odometry_, 0.0, 0);
  EXPECT_FALSE(result.optimized);
  EXPECT_EQ(result.trajectory.points.size(), short_trajectory.points.size());
}

TEST_F(TrajectoryOptimizerTest, WarmStartAcrossCycles)
{
  TrajectoryOptimizationParams params;
  params.enable = true;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  const auto raw = make_noisy_trajectory(8.0, 0.15);
  const auto first = optimizer.optimize(raw, odometry_, 0.0, 0);
  ASSERT_TRUE(first.optimized);

  // Second solve with warm start must also succeed.
  const auto second = optimizer.optimize(raw, odometry_, 0.0, 0);
  ASSERT_TRUE(second.optimized);
}

TEST_F(TrajectoryOptimizerTest, DoesNotLatchGoalSnapWhenEgoIsFar)
{
  TrajectoryOptimizationParams params;
  params.enable = true;
  params.temporal_consistency.enable = false;
  params.goal.snap_distance_m = 100.0;
  params.goal.unlatch_horizon_s = 8.0;
  params.goal.unlatch_min_speed_mps = 3.0;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  constexpr double speed = 8.0;
  odometry_.twist.twist.linear.x = speed;
  const auto raw = make_straight_trajectory(0.0, speed);
  const auto goal = make_pose(100.0, 0.0);

  const auto result = optimizer.optimize(raw, odometry_, 0.0, 0, goal);
  ASSERT_TRUE(result.optimized) << "acados status: " << result.solver_status;
  EXPECT_FALSE(result.goal_snap_active);
  // Horizon length is 8 s * 8 m/s = 64 m. Without snap the terminal stays near 64;
  // a latched snap with snap_distance_m=100 would pull it toward the goal at 100.
  EXPECT_NEAR(result.trajectory.points.back().pose.position.x, 64.0, 3.0);
}

TEST_F(TrajectoryOptimizerTest, UnlatchesGoalSnapWhenEgoDrivesAway)
{
  TrajectoryOptimizationParams params;
  params.enable = true;
  params.temporal_consistency.enable = false;
  params.goal.snap_distance_m = 1.0;
  params.goal.unlatch_horizon_s = 20.0;
  params.goal.weight_longitudinal = 100.0;
  params.goal.weight_lateral = 100.0;
  params.goal.weight_yaw = 10.0;
  params.goal.weight_velocity = 1.0;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  constexpr double x0 = 50.0;
  constexpr double speed = 8.0;
  odometry_.pose.pose.position.x = x0;
  odometry_.twist.twist.linear.x = speed;
  auto raw = make_straight_trajectory(x0, speed);
  const auto & terminal = raw.points.back().pose.position;
  const auto goal = make_pose(terminal.x + 0.2, 0.0);

  const auto latched = optimizer.optimize(raw, odometry_, 0.0, 0, goal);
  ASSERT_TRUE(latched.optimized);
  EXPECT_TRUE(latched.goal_snap_active);
  EXPECT_NEAR(latched.trajectory.points.back().pose.position.x, goal.position.x, 0.35);

  constexpr double far_x0 = -200.0;
  odometry_.pose.pose.position.x = far_x0;
  raw = make_straight_trajectory(far_x0, speed);
  // Age the previous solution so a geographically invalid warm start is not reused after
  // this teleport; a real drive-away stays within max_warm_start_age_s.
  raw.header.stamp.sec = 1;
  const auto unlatched = optimizer.optimize(raw, odometry_, 0.0, 0, goal);
  ASSERT_TRUE(unlatched.optimized) << "acados status: " << unlatched.solver_status;
  EXPECT_FALSE(unlatched.goal_snap_active);
  EXPECT_NEAR(
    unlatched.trajectory.points.back().pose.position.x, far_x0 + speed * opt_dt_s * opt_horizon,
    2.0);
}

TEST_F(TrajectoryOptimizerTest, UnlatchSkipsTemporalOnceThenRestoresIt)
{
  TrajectoryOptimizationParams params;
  params.enable = true;
  params.temporal_consistency.enable = true;
  params.goal.snap_distance_m = 1.0;
  params.goal.unlatch_horizon_s = 8.125;
  params.goal.unlatch_min_speed_mps = 3.0;
  TrajectoryOptimizer optimizer(params, vehicle_info_, 1);

  constexpr double x0 = 50.0;
  constexpr double speed = 8.0;
  odometry_.pose.pose.position.x = x0;
  odometry_.twist.twist.linear.x = speed;
  auto raw = make_straight_trajectory(x0, speed);
  raw.header.stamp.sec = 0;
  const auto & terminal = raw.points.back().pose.position;
  const auto goal = make_pose(terminal.x + 0.2, 0.0);

  const auto latched = optimizer.optimize(raw, odometry_, 0.0, 0, goal);
  ASSERT_TRUE(latched.optimized);
  EXPECT_TRUE(latched.goal_snap_active);

  constexpr double away_x0 = 48.0;
  odometry_.pose.pose.position.x = away_x0;
  raw = make_straight_trajectory(away_x0, speed);
  raw.header.stamp.nanosec = 100000000;
  const auto unlatched = optimizer.optimize(raw, odometry_, 0.0, 0, goal);
  ASSERT_TRUE(unlatched.optimized) << "acados status: " << unlatched.solver_status;
  EXPECT_FALSE(unlatched.goal_snap_active);
  EXPECT_FALSE(unlatched.temporal_applied);

  raw.header.stamp.nanosec = 200000000;
  const auto restored = optimizer.optimize(raw, odometry_, 0.0, 0, goal);
  ASSERT_TRUE(restored.optimized) << "acados status: " << restored.solver_status;
  EXPECT_FALSE(restored.goal_snap_active);
  EXPECT_TRUE(restored.temporal_applied);
}

TEST_F(TrajectoryOptimizerTest, GoalPositionChangeClearsPreviousSolutions)
{
  TrajectoryOptimizationParams params;
  params.enable = true;
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

  const auto first = optimizer.optimize(raw, odometry_, 0.0, 0, goal_a);
  ASSERT_TRUE(first.optimized);
  EXPECT_TRUE(first.goal_snap_active);

  raw.header.stamp.nanosec = 100000000;
  const auto second = optimizer.optimize(raw, odometry_, 0.0, 0, goal_a);
  ASSERT_TRUE(second.optimized);
  EXPECT_TRUE(second.goal_snap_active);
  EXPECT_TRUE(second.temporal_applied);

  // Nearby new goal still snap-eligible. Clearing previous_solutions_ is required so the
  // old latched plan is not reused as a temporal reference under a matching latch flag.
  const auto goal_b = make_pose(terminal.x + 0.5, 0.0);
  raw.header.stamp.nanosec = 200000000;
  const auto changed = optimizer.optimize(raw, odometry_, 0.0, 0, goal_b);
  ASSERT_TRUE(changed.optimized) << "acados status: " << changed.solver_status;
  EXPECT_TRUE(changed.goal_snap_active);
  EXPECT_FALSE(changed.temporal_applied);
}

}  // namespace autoware::ml_planner::test
