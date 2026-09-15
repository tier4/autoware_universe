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
  auto traj = postprocess::create_ego_trajectory(agent_poses, stamp, 0);
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

}  // namespace autoware::ml_planner::test
