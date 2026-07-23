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

#include "autoware/diffusion_planner/utils/trajectory_stitcher.hpp"

#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"

#include <Eigen/Dense>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <memory>
#include <string>

namespace autoware::diffusion_planner::test
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;
using unique_identifier_msgs::msg::UUID;

class TrajectoryStitcherTest : public ::testing::Test
{
protected:
  static constexpr double kDt = 0.1;
  static constexpr double kSpacing = 1.0;

  void SetUp() override
  {
    params_.enable = true;
    params_.rearm_cooldown_s = 1.0;
    stitcher_ = std::make_unique<TrajectoryStitcher>(params_);
  }

  // Straight line along +x at 10 m/s: point i at x = (i + 1) * 1.0 m,
  // time_from_start = 0.1 * (i + 1).
  Trajectory make_trajectory(const size_t num_points = 20, const double stamp_offset_s = 0.0) const
  {
    Trajectory trajectory;
    trajectory.header.stamp = at(stamp_offset_s);
    trajectory.header.frame_id = "map";
    for (size_t i = 0; i < num_points; ++i) {
      TrajectoryPoint point;
      point.pose.position.x = static_cast<double>(i + 1) * kSpacing;
      point.pose.orientation.w = 1.0;
      point.time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(i + 1) * kDt);
      point.longitudinal_velocity_mps = 10.0;
      trajectory.points.push_back(point);
    }
    return trajectory;
  }

  static geometry_msgs::msg::Pose make_pose(const double x, const double y = 0.0)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation.w = 1.0;
    return pose;
  }

  static Odometry make_odom(const double x, const double y = 0.0, const double vx = 10.0)
  {
    Odometry odometry;
    odometry.pose.pose = make_pose(x, y);
    odometry.twist.twist.linear.x = vx;
    return odometry;
  }

  static UUID make_uuid(const uint8_t value)
  {
    UUID uuid;
    uuid.uuid.fill(value);
    return uuid;
  }

  rclcpp::Time at(const double seconds_after_t0) const
  {
    return t0_ + rclcpp::Duration::from_seconds(seconds_after_t0);
  }

  void store_default_trajectory(const double stamp_offset_s = 0.0) const
  {
    stitcher_->set_previous_trajectory(make_trajectory(20, stamp_offset_s), uuid_a_);
  }

  StitchingStatus arm() const
  {
    const auto status = stitcher_->compute_planning_origin(at(0.1), make_odom(1.0), uuid_a_, false);
    EXPECT_TRUE(status.stitched);
    return status;
  }

  TrajectoryStitcherParams params_;
  std::unique_ptr<TrajectoryStitcher> stitcher_;
  rclcpp::Time t0_{100, 0, RCL_ROS_TIME};
  UUID uuid_a_{make_uuid(1)};
  UUID uuid_b_{make_uuid(2)};
};

TEST_F(TrajectoryStitcherTest, FirstCycleResets)
{
  const auto status = stitcher_->compute_planning_origin(at(0.1), make_odom(1.0), uuid_a_, false);
  EXPECT_FALSE(status.stitched);
  EXPECT_EQ(status.reset_reason, "no_previous_trajectory");
  EXPECT_DOUBLE_EQ(status.planning_origin.position.x, 1.0);
}

TEST_F(TrajectoryStitcherTest, StitchesAtProjection)
{
  store_default_trajectory();
  arm();

  const auto mid_segment =
    stitcher_->compute_planning_origin(at(0.1), make_odom(1.4, 0.2), uuid_a_, false);
  EXPECT_TRUE(mid_segment.stitched);
  EXPECT_NEAR(mid_segment.planning_origin.position.x, 1.4, 1e-9);
  EXPECT_NEAR(mid_segment.planning_origin.position.y, 0.0, 1e-9);
  EXPECT_NEAR(mid_segment.lateral_deviation_m, 0.2, 1e-9);
  EXPECT_NEAR(mid_segment.longitudinal_deviation_m, 0.0, 1e-9);

  const auto before_start =
    stitcher_->compute_planning_origin(at(0.1), make_odom(0.5), uuid_a_, false);
  EXPECT_TRUE(before_start.stitched);
  EXPECT_NEAR(before_start.planning_origin.position.x, 1.0, 1e-9);
}

TEST_F(TrajectoryStitcherTest, TimeOffsetLeadsAlongArc)
{
  params_.time_offset_s = 0.05;
  stitcher_->update_params(params_);
  store_default_trajectory();

  const auto status = stitcher_->compute_planning_origin(at(0.1), make_odom(1.0), uuid_a_, false);
  EXPECT_TRUE(status.stitched);
  EXPECT_NEAR(status.planning_origin.position.x, 1.5, 1e-9);
  EXPECT_NEAR(status.longitudinal_deviation_m, 0.5, 1e-9);
}

TEST_F(TrajectoryStitcherTest, LateralDeviationTriggersReset)
{
  store_default_trajectory();
  arm();

  const auto status =
    stitcher_->compute_planning_origin(at(0.1), make_odom(1.0, 0.4), uuid_a_, false);
  EXPECT_FALSE(status.stitched);
  EXPECT_EQ(status.reset_reason, "deviation_exceeded");
  EXPECT_NEAR(status.lateral_deviation_m, 0.4, 1e-9);
  EXPECT_DOUBLE_EQ(status.planning_origin.position.y, 0.4);
}

TEST_F(TrajectoryStitcherTest, RearmWaitsForCooldownAndConvergence)
{
  store_default_trajectory();
  arm();

  stitcher_->compute_planning_origin(at(0.2), make_odom(1.0, 0.4), uuid_a_, false);

  store_default_trajectory(1.0);
  const auto in_cooldown =
    stitcher_->compute_planning_origin(at(1.1), make_odom(1.0, 0.05), uuid_a_, false);
  EXPECT_FALSE(in_cooldown.stitched);
  EXPECT_EQ(in_cooldown.reset_reason, "waiting_rearm");

  const auto still_deviated =
    stitcher_->compute_planning_origin(at(1.3), make_odom(1.0, 0.2), uuid_a_, false);
  EXPECT_FALSE(still_deviated.stitched);
  EXPECT_EQ(still_deviated.reset_reason, "waiting_rearm");

  const auto rearmed =
    stitcher_->compute_planning_origin(at(1.3), make_odom(1.0, 0.05), uuid_a_, false);
  EXPECT_TRUE(rearmed.stitched);
  EXPECT_NEAR(rearmed.planning_origin.position.y, 0.0, 1e-9);
}

TEST_F(TrajectoryStitcherTest, ProjectionBeyondHorizonResets)
{
  store_default_trajectory();

  const auto past_end =
    stitcher_->compute_planning_origin(at(0.1), make_odom(25.0), uuid_a_, false);
  EXPECT_FALSE(past_end.stitched);
  EXPECT_EQ(past_end.reset_reason, "beyond_horizon");

  const auto far_behind =
    stitcher_->compute_planning_origin(at(0.1), make_odom(-3.0), uuid_a_, false);
  EXPECT_FALSE(far_behind.stitched);
  EXPECT_EQ(far_behind.reset_reason, "beyond_horizon");
}

TEST_F(TrajectoryStitcherTest, StaleTrajectoryResets)
{
  store_default_trajectory();

  const auto status = stitcher_->compute_planning_origin(at(0.5), make_odom(5.0), uuid_a_, false);
  EXPECT_FALSE(status.stitched);
  EXPECT_EQ(status.reset_reason, "stale_trajectory");

  const auto next = stitcher_->compute_planning_origin(at(0.1), make_odom(1.0), uuid_a_, false);
  EXPECT_EQ(next.reset_reason, "no_previous_trajectory");
}

TEST_F(TrajectoryStitcherTest, RouteChangeResets)
{
  store_default_trajectory();
  arm();

  const auto status = stitcher_->compute_planning_origin(at(0.1), make_odom(1.0), uuid_b_, false);
  EXPECT_FALSE(status.stitched);
  EXPECT_EQ(status.reset_reason, "route_changed");

  stitcher_->set_previous_trajectory(make_trajectory(), uuid_b_);
  const auto restitched =
    stitcher_->compute_planning_origin(at(0.1), make_odom(1.0), uuid_b_, false);
  EXPECT_TRUE(restitched.stitched);
}

TEST_F(TrajectoryStitcherTest, LowSpeedResets)
{
  store_default_trajectory();

  const auto status =
    stitcher_->compute_planning_origin(at(0.1), make_odom(1.0, 0.0, 0.1), uuid_a_, false);
  EXPECT_FALSE(status.stitched);
  EXPECT_EQ(status.reset_reason, "low_speed");

  const auto reverse =
    stitcher_->compute_planning_origin(at(0.1), make_odom(1.0, 0.0, -1.0), uuid_a_, false);
  EXPECT_EQ(reverse.reset_reason, "low_speed");
}

TEST_F(TrajectoryStitcherTest, DisabledPassesThrough)
{
  params_.enable = false;
  stitcher_->update_params(params_);
  store_default_trajectory();

  const auto status = stitcher_->compute_planning_origin(at(0.1), make_odom(1.0), uuid_a_, false);
  EXPECT_FALSE(status.stitched);
  EXPECT_EQ(status.reset_reason, "disabled");
  EXPECT_DOUBLE_EQ(status.planning_origin.position.x, 1.0);
}

TEST_F(TrajectoryStitcherTest, MppiOverwriteForcesReset)
{
  store_default_trajectory();

  const auto status = stitcher_->compute_planning_origin(at(0.1), make_odom(1.0), uuid_a_, true);
  EXPECT_FALSE(status.stitched);
  EXPECT_EQ(status.reset_reason, "mppi_active");
}

TEST_F(TrajectoryStitcherTest, ResetClearsState)
{
  store_default_trajectory();
  stitcher_->push_planning_origin_history(at(0.0), make_pose(0.0), 31);
  arm();

  stitcher_->reset();

  const auto status = stitcher_->compute_planning_origin(at(0.1), make_odom(1.0), uuid_a_, false);
  EXPECT_EQ(status.reset_reason, "no_previous_trajectory");
  EXPECT_TRUE(stitcher_->planning_origin_history().empty());
}

TEST_F(TrajectoryStitcherTest, PlanningOriginHistoryDeque)
{
  constexpr size_t max_size = 31;
  constexpr size_t num_pushes = 35;
  for (size_t i = 0; i < num_pushes; ++i) {
    stitcher_->push_planning_origin_history(
      at(static_cast<double>(i) * kDt), make_pose(static_cast<double>(i)), max_size);
  }

  const auto & history = stitcher_->planning_origin_history();
  ASSERT_EQ(history.size(), max_size);
  EXPECT_DOUBLE_EQ(history.front().pose.pose.position.x, 4.0);
  EXPECT_DOUBLE_EQ(history.back().pose.pose.position.x, 34.0);
  for (size_t i = 1; i < history.size(); ++i) {
    EXPECT_LT(
      rclcpp::Time(history[i - 1].header.stamp).seconds(),
      rclcpp::Time(history[i].header.stamp).seconds());
  }

  // The deque must be directly consumable by create_ego_agent_past (time interpolation mode)
  const size_t num_timesteps = 5;
  const auto reference_time = rclcpp::Time(history.back().header.stamp);
  const auto ego_agent_past = preprocess::create_ego_agent_past(
    history, num_timesteps, Eigen::Matrix4d::Identity(), reference_time);
  ASSERT_EQ(ego_agent_past.size(), num_timesteps * 4);
  for (size_t t = 0; t < num_timesteps; ++t) {
    const double expected_x = static_cast<double>(num_pushes - num_timesteps + t);
    EXPECT_NEAR(ego_agent_past[t * 4], expected_x, 1e-5);
  }
}

TEST_F(TrajectoryStitcherTest, NoisyStopTailIsHandled)
{
  auto trajectory = make_trajectory();
  for (size_t i = 15; i < trajectory.points.size(); ++i) {
    trajectory.points[i].pose.position.x = 15.0 + (i % 2 == 0 ? 1e-4 : -1e-4);
    trajectory.points[i].longitudinal_velocity_mps = 0.0;
  }
  stitcher_->set_previous_trajectory(trajectory, uuid_a_);

  const auto status = stitcher_->compute_planning_origin(at(0.1), make_odom(1.0), uuid_a_, false);
  EXPECT_TRUE(status.stitched);
  EXPECT_TRUE(std::isfinite(status.lateral_deviation_m));
  EXPECT_TRUE(std::isfinite(status.longitudinal_deviation_m));
}

TEST_F(TrajectoryStitcherTest, DegenerateTrajectoryResets)
{
  auto trajectory = make_trajectory();
  for (auto & point : trajectory.points) {
    point.pose.position.x = 5.0;
  }
  stitcher_->set_previous_trajectory(trajectory, uuid_a_);

  const auto status = stitcher_->compute_planning_origin(at(0.1), make_odom(5.0), uuid_a_, false);
  EXPECT_FALSE(status.stitched);
  EXPECT_EQ(status.reset_reason, "degenerate_trajectory");
}

}  // namespace autoware::diffusion_planner::test
