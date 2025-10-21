// Copyright 2023 Tier IV, Inc.
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

#include "autoware/path_smoother/utils/trajectory_utils.hpp"

#include <gtest/gtest.h>

#include <vector>

using autoware::path_smoother::trajectory_utils::apply_input_velocity;

class ApplyVelocityTest : public ::testing::Test
{
protected:
  autoware::path_smoother::EgoNearestParam params;
  geometry_msgs::msg::Pose ego_pose;
  std::vector<autoware::path_smoother::TrajectoryPoint> output_trajectory;
  std::vector<autoware::path_smoother::TrajectoryPoint> input_trajectory;

  void SetUp() override
  {
    params.dist_threshold = 1.0;
    params.yaw_threshold = M_PI;
    ego_pose.position.x = 0.0;
    ego_pose.position.y = 0.0;
  }

  static autoware::path_smoother::TrajectoryPoint create_point(double x, double y, float vel)
  {
    autoware::path_smoother::TrajectoryPoint p;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.longitudinal_velocity_mps = vel;
    return p;
  }
};

TEST_F(ApplyVelocityTest, IdenticalTrajectories)
{
  // Both trajectories are the same straight line with different velocities.
  for (int i = 0; i < 5; ++i) {
    output_trajectory.push_back(create_point(i, 0, 0.0f));
    input_trajectory.push_back(create_point(i, 0, 1.5f * static_cast<float>(i)));
  }

  apply_input_velocity(output_trajectory, input_trajectory, ego_pose, params);

  EXPECT_FLOAT_EQ(
    output_trajectory[0].longitudinal_velocity_mps, input_trajectory[0].longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(
    output_trajectory[1].longitudinal_velocity_mps, input_trajectory[0].longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(
    output_trajectory[2].longitudinal_velocity_mps, input_trajectory[1].longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(
    output_trajectory[3].longitudinal_velocity_mps, input_trajectory[2].longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(
    output_trajectory[4].longitudinal_velocity_mps, input_trajectory[3].longitudinal_velocity_mps);
}

TEST_F(ApplyVelocityTest, ShiftedTrajectories)
{
  // Output trajectory is on the x-axis
  // Input trajectory is the same shape but shifted up on the y-axis
  // The shift is within the distance threshold
  double y_offset = 0.5;
  for (int i = 0; i < 5; ++i) {
    output_trajectory.push_back(create_point(i, 0, 0.0f));
    input_trajectory.push_back(create_point(i, y_offset, 1.5f * static_cast<float>(i)));
  }

  apply_input_velocity(output_trajectory, input_trajectory, ego_pose, params);

  // Each output point's closest input point is directly "above" it.
  EXPECT_FLOAT_EQ(
    output_trajectory[0].longitudinal_velocity_mps, input_trajectory[0].longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(
    output_trajectory[1].longitudinal_velocity_mps, input_trajectory[0].longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(
    output_trajectory[2].longitudinal_velocity_mps, input_trajectory[1].longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(
    output_trajectory[3].longitudinal_velocity_mps, input_trajectory[2].longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(
    output_trajectory[4].longitudinal_velocity_mps, input_trajectory[3].longitudinal_velocity_mps);
}

TEST_F(ApplyVelocityTest, InputIsFar)
{
  // Input trajectory is shifted far away, outside the distance threshold
  double y_offset = 2.0;  // 2.0 > 1.0 threshold
  for (int i = 0; i < 5; ++i) {
    output_trajectory.push_back(create_point(i, 0, 0.0f));
    input_trajectory.push_back(create_point(i, y_offset, 1.5f * static_cast<float>(i)));
  }

  apply_input_velocity(output_trajectory, input_trajectory, ego_pose, params);

  // Velocities are still copied
  EXPECT_FLOAT_EQ(
    output_trajectory[0].longitudinal_velocity_mps, input_trajectory[0].longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(
    output_trajectory[1].longitudinal_velocity_mps, input_trajectory[0].longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(
    output_trajectory[2].longitudinal_velocity_mps, input_trajectory[1].longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(
    output_trajectory[3].longitudinal_velocity_mps, input_trajectory[2].longitudinal_velocity_mps);
  EXPECT_FLOAT_EQ(
    output_trajectory[4].longitudinal_velocity_mps, input_trajectory[3].longitudinal_velocity_mps);
}

TEST_F(ApplyVelocityTest, EmptyInputTrajectory)
{
  // Output trajectory has points, but input is empty
  for (int i = 0; i < 5; ++i) {
    output_trajectory.push_back(create_point(i, 0, 0.0f));
  }
  input_trajectory.clear();  // Ensure input is empty

  EXPECT_THROW(apply_input_velocity(output_trajectory, input_trajectory, ego_pose, params);
               , std::invalid_argument);
}

TEST_F(ApplyVelocityTest, SingleClosestInputPoint)
{
  // The entire input trajectory is just one point, which is close to the output trajectory
  output_trajectory.push_back(create_point(0, 0, 0.0f));
  output_trajectory.push_back(create_point(1, 0, 0.0f));
  output_trajectory.push_back(create_point(2, 0, 0.0f));

  // A single point that is closest to output_trajectory[1]
  input_trajectory.push_back(create_point(1.1, 0.1, 1.0f));

  apply_input_velocity(output_trajectory, input_trajectory, ego_pose, params);

  // All output points should get the velocity from that single input point
  for (const auto & point : output_trajectory) {
    EXPECT_FLOAT_EQ(
      point.longitudinal_velocity_mps, input_trajectory.front().longitudinal_velocity_mps);
  }
}

TEST_F(ApplyVelocityTest, DifferentSizedTrajectories)
{
  // Output has more points than input.
  output_trajectory.push_back(create_point(0.0, 0, 0.0f));
  output_trajectory.push_back(create_point(1.0, 0, 0.0f));
  output_trajectory.push_back(create_point(2.0, 0, 0.0f));
  output_trajectory.push_back(create_point(3.0, 0, 0.0f));

  input_trajectory.push_back(create_point(0.6, 0.0, 1.0f));
  input_trajectory.push_back(create_point(2.4, 0.0, 2.0f));

  apply_input_velocity(output_trajectory, input_trajectory, ego_pose, params);

  EXPECT_FLOAT_EQ(output_trajectory[0].longitudinal_velocity_mps, 1.0f);
  EXPECT_FLOAT_EQ(output_trajectory[1].longitudinal_velocity_mps, 1.0f);
  EXPECT_FLOAT_EQ(output_trajectory[2].longitudinal_velocity_mps, 1.0f);
  EXPECT_FLOAT_EQ(output_trajectory[3].longitudinal_velocity_mps, 1.0f);

  input_trajectory.push_back(create_point(3.1, 0.0, 0.0f));
  input_trajectory.push_back(create_point(3.4, 0.0, 1.5f));

  apply_input_velocity(output_trajectory, input_trajectory, ego_pose, params);

  EXPECT_FLOAT_EQ(output_trajectory[0].longitudinal_velocity_mps, 1.0f);
  EXPECT_FLOAT_EQ(output_trajectory[1].longitudinal_velocity_mps, 1.0f);
  EXPECT_FLOAT_EQ(output_trajectory[2].longitudinal_velocity_mps, 1.0f);
  EXPECT_FLOAT_EQ(output_trajectory[3].longitudinal_velocity_mps, 0.0f);
}
