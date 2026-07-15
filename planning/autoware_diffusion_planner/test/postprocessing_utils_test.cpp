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

#include "autoware/diffusion_planner/postprocessing/postprocessing_utils.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"

#include <Eigen/Dense>

#include <geometry_msgs/msg/point.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <vector>

namespace autoware::diffusion_planner::test
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
  const int64_t velocity_smoothing_window = 8;
  const bool enable_force_stop = false;
  const double stopping_threshold = 0.0;
  const auto agent_poses = postprocess::parse_predictions(data, transform);
  geometry_msgs::msg::Point base_position;
  base_position.x = 0.0;
  base_position.y = 0.0;
  base_position.z = 0.0;
  auto traj = postprocess::create_ego_trajectory(
    agent_poses, stamp, base_position, 0, velocity_smoothing_window, enable_force_stop,
    stopping_threshold);
  ASSERT_EQ(traj.points.size(), expected_points);
}

TEST(PostprocessingUtilsTest, WindowOnePreservesPerStepVelocity)
{
  constexpr auto prediction_shape = OUTPUT_SHAPE;
  const auto batch_size = prediction_shape[0];
  const auto agent_size = prediction_shape[1];
  const auto rows = prediction_shape[2];
  const auto cols = prediction_shape[3];
  std::vector<float> data(batch_size * agent_size * rows * cols, 0.0f);

  // Use non-uniform per-step displacements so a window larger than one would
  // produce observably different velocities.
  constexpr std::array<float, 4> x_positions = {0.1F, 0.3F, 0.6F, 1.0F};
  for (int64_t time_idx = 0; time_idx < rows; ++time_idx) {
    const auto x = time_idx < static_cast<int64_t>(x_positions.size())
                     ? x_positions[static_cast<size_t>(time_idx)]
                     : x_positions.back() +
                         0.1F * static_cast<float>(
                                  time_idx - static_cast<int64_t>(x_positions.size()) + 1);
    const auto index = (time_idx * cols);
    data[index] = x;
    data[index + 2] = 1.0F;
  }

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  const auto agent_poses = postprocess::parse_predictions(data, transform);
  geometry_msgs::msg::Point base_position;
  const auto trajectory = postprocess::create_ego_trajectory(
    agent_poses, rclcpp::Time(123, 0), base_position, 0, 1, false, 0.0);

  EXPECT_NEAR(trajectory.points[0].longitudinal_velocity_mps, 1.0, 1e-6);
  EXPECT_NEAR(trajectory.points[1].longitudinal_velocity_mps, 2.0, 1e-6);
  EXPECT_NEAR(trajectory.points[2].longitudinal_velocity_mps, 3.0, 1e-6);
  EXPECT_NEAR(trajectory.points[3].longitudinal_velocity_mps, 4.0, 1e-6);
}

TEST(PostprocessingUtilsTest, DisabledForceStopPreservesTemporalRestart)
{
  constexpr auto prediction_shape = OUTPUT_SHAPE;
  const auto batch_size = prediction_shape[0];
  const auto agent_size = prediction_shape[1];
  const auto rows = prediction_shape[2];
  const auto cols = prediction_shape[3];
  std::vector<float> data(
    static_cast<size_t>(batch_size * agent_size * rows * cols), 0.0F);

  // A brief low-speed step is followed by a restart.  HDP must preserve this
  // learned temporal sequence when the legacy force-stop heuristic is disabled.
  constexpr std::array<float, 4> x_positions = {0.4F, 0.42F, 0.82F, 1.22F};
  for (int64_t time_idx = 0; time_idx < rows; ++time_idx) {
    const auto x = time_idx < static_cast<int64_t>(x_positions.size())
                     ? x_positions[static_cast<size_t>(time_idx)]
                     : x_positions.back() +
                         0.4F * static_cast<float>(
                                  time_idx - static_cast<int64_t>(x_positions.size()) + 1);
    data[static_cast<size_t>(time_idx * cols)] = x;
    data[static_cast<size_t>(time_idx * cols + 2)] = 1.0F;
  }

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  const auto agent_poses = postprocess::parse_predictions(data, transform);
  geometry_msgs::msg::Point base_position;
  const auto trajectory = postprocess::create_ego_trajectory(
    agent_poses, rclcpp::Time(123, 0), base_position, 0, 1, false, 0.3);

  EXPECT_NEAR(trajectory.points[1].longitudinal_velocity_mps, 0.2, 1e-6);
  EXPECT_NEAR(trajectory.points[2].longitudinal_velocity_mps, 4.0, 1e-6);
  EXPECT_NEAR(trajectory.points[2].pose.position.x, 0.82, 1e-6);
}

}  // namespace autoware::diffusion_planner::test
