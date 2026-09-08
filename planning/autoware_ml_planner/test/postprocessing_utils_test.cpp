// Copyright 2025 TIER IV, Inc.
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

#include "autoware/ml_planner/postprocessing/postprocessing_utils.hpp"

#include "autoware/ml_planner/dimensions.hpp"

#include <Eigen/Dense>

#include <geometry_msgs/msg/point.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace autoware::ml_planner::test
{
using autoware_planning_msgs::msg::Trajectory;

TEST(PostprocessingUtilsTest, CreateTrajectoryAndMultipleTrajectories)
{
  constexpr auto prediction_shape = OUTPUT_SHAPE;
  auto batch_size = prediction_shape[0];
  auto agent_size = prediction_shape[1];
  auto rows = prediction_shape[2];
  auto cols = prediction_shape[3];
  std::vector<float> data(batch_size * agent_size * rows * cols, 0.0f);
  // Fill with some values for checking
  for (size_t i = 0; i < data.size(); ++i) data[i] = static_cast<float>(i);

  std::vector<int64_t> shape{batch_size, agent_size, rows, cols};
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  rclcpp::Time stamp(123, 0);

  auto expected_points = prediction_shape[2];
  const auto agent_poses = postprocess::parse_predictions(data, transform);
  geometry_msgs::msg::Point base_position;
  auto traj = postprocess::create_ego_trajectory(agent_poses, stamp, base_position, 0);
  ASSERT_EQ(traj.points.size(), expected_points);
}

TEST(PostprocessingUtilsTest, FixStopPointsAfterConfiguredDecelerationDuration)
{
  Trajectory trajectory;
  for (size_t i = 0; i < 7; ++i) {
    auto & point = trajectory.points.emplace_back();
    const double time = 0.5 * static_cast<double>(i);
    point.time_from_start.sec = static_cast<int32_t>(time);
    point.time_from_start.nanosec = static_cast<uint32_t>((time - std::floor(time)) * 1.0e9);
    point.pose.position.x = static_cast<double>(i);
    point.longitudinal_velocity_mps = i >= 5 ? 0.2F : 2.0F;
    point.acceleration_mps2 = i >= 3 ? -1.0F : 0.0F;
  }

  postprocess::StopPointFixingParams params;
  params.velocity_threshold_mps = 0.3;
  params.min_deceleration_duration_sec = 1.0;
  const auto stop_index = postprocess::fix_stop_points(trajectory, params);

  ASSERT_EQ(stop_index, 5U);
  EXPECT_DOUBLE_EQ(trajectory.points[4].pose.position.x, 4.0);
  for (size_t i = 5; i < trajectory.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(trajectory.points[i].pose.position.x, 5.0);
    EXPECT_FLOAT_EQ(trajectory.points[i].longitudinal_velocity_mps, 0.0F);
    EXPECT_FLOAT_EQ(trajectory.points[i].acceleration_mps2, 0.0F);
  }
}

TEST(PostprocessingUtilsTest, FixStopPointsResetsDecelerationDuration)
{
  Trajectory trajectory;
  for (size_t i = 0; i < 5; ++i) {
    auto & point = trajectory.points.emplace_back();
    point.time_from_start.sec = static_cast<int32_t>(i);
    point.pose.position.x = static_cast<double>(i);
    point.longitudinal_velocity_mps = 0.2F;
    point.acceleration_mps2 = -1.0F;
  }
  trajectory.points[2].acceleration_mps2 = 0.0F;

  postprocess::StopPointFixingParams params;
  params.min_deceleration_duration_sec = 2.0;
  EXPECT_FALSE(postprocess::fix_stop_points(trajectory, params).has_value());
  EXPECT_DOUBLE_EQ(trajectory.points.back().pose.position.x, 4.0);
}

namespace
{
// Ego poses on the model output grid, laid out along +x at the given positions.
std::vector<std::vector<std::vector<Eigen::Matrix4d>>> make_ego_poses_along_x(
  const std::vector<double> & positions_x)
{
  std::vector<Eigen::Matrix4d> ego_poses;
  for (const double x : positions_x) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose(0, 3) = x;
    ego_poses.push_back(pose);
  }
  return {{ego_poses}};
}
}  // namespace

TEST(PostprocessingUtilsTest, CreateEgoTrajectoryConstantSpeed)
{
  // Points 1 m apart on the 0.1 s grid, the first one 1 m ahead of the ego position: 10 m/s.
  const auto agent_poses = make_ego_poses_along_x({1.0, 2.0, 3.0, 4.0, 5.0});
  geometry_msgs::msg::Point base_position;

  const auto trajectory =
    postprocess::create_ego_trajectory(agent_poses, rclcpp::Time(0), base_position, 0);

  ASSERT_EQ(trajectory.points.size(), 5U);
  for (const auto & point : trajectory.points) {
    EXPECT_NEAR(point.longitudinal_velocity_mps, 10.0F, 1e-3F);
    EXPECT_NEAR(point.acceleration_mps2, 0.0F, 1e-3F);
    // The model predicts poses only.
    EXPECT_FLOAT_EQ(point.heading_rate_rps, 0.0F);
    EXPECT_FLOAT_EQ(point.front_wheel_angle_rad, 0.0F);
  }
}

TEST(PostprocessingUtilsTest, CreateEgoTrajectoryAcceleration)
{
  // Steps of 0.1, 0.2, 0.3 m on the 0.1 s grid: 1, 2, 3 m/s, i.e. 10 m/s^2.
  const auto agent_poses = make_ego_poses_along_x({0.1, 0.3, 0.6});
  geometry_msgs::msg::Point base_position;

  const auto trajectory =
    postprocess::create_ego_trajectory(agent_poses, rclcpp::Time(0), base_position, 0);

  ASSERT_EQ(trajectory.points.size(), 3U);
  EXPECT_NEAR(trajectory.points[0].longitudinal_velocity_mps, 1.0F, 1e-3F);
  EXPECT_NEAR(trajectory.points[1].longitudinal_velocity_mps, 2.0F, 1e-3F);
  EXPECT_NEAR(trajectory.points[2].longitudinal_velocity_mps, 3.0F, 1e-3F);
  EXPECT_NEAR(trajectory.points[0].acceleration_mps2, 10.0F, 1e-2F);
  EXPECT_NEAR(trajectory.points[1].acceleration_mps2, 10.0F, 1e-2F);
  // The last point has no successor.
  EXPECT_FLOAT_EQ(trajectory.points[2].acceleration_mps2, 0.0F);
}

TEST(PostprocessingUtilsTest, CreateEgoTrajectoryUsesEgoPositionForTheFirstPoint)
{
  // Same poses, ego already at x = 1: the first point no longer covers any distance.
  const auto agent_poses = make_ego_poses_along_x({1.0, 2.0});
  geometry_msgs::msg::Point base_position;
  base_position.x = 1.0;

  const auto trajectory =
    postprocess::create_ego_trajectory(agent_poses, rclcpp::Time(0), base_position, 0);

  ASSERT_EQ(trajectory.points.size(), 2U);
  EXPECT_NEAR(trajectory.points[0].longitudinal_velocity_mps, 0.0F, 1e-3F);
  EXPECT_NEAR(trajectory.points[1].longitudinal_velocity_mps, 10.0F, 1e-3F);
}

}  // namespace autoware::ml_planner::test
