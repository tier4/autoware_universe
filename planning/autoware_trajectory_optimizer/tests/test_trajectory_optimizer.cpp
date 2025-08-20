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

#include "autoware/trajectory_optimizer/trajectory_optimizer_structs.hpp"
#include "autoware/trajectory_optimizer/utils.hpp"
#include "autoware/velocity_smoother/smoother/jerk_filtered_smoother.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <limits>

using namespace autoware::trajectory_optimizer::utils;
using namespace autoware::trajectory_optimizer;
using namespace autoware_planning_msgs::msg;
using namespace nav_msgs::msg;
using namespace geometry_msgs::msg;

class TrajectoryOptimizerUtilsTest : public ::testing::Test
{
protected:
  TrajectoryPoints create_sample_trajectory(double resolution = 1.0, double offset = 0.0)
  {
    TrajectoryPoints points;
    for (int i = 0; i < 10; ++i) {
      TrajectoryPoint point;
      point.pose.position.x = i * resolution + offset;
      point.pose.position.y = i * resolution + offset;
      point.longitudinal_velocity_mps = 1.0;
      point.acceleration_mps2 = 0.1;
      points.push_back(point);
    }
    return points;
  }
};

TEST_F(TrajectoryOptimizerUtilsTest, RemoveInvalidPoints)
{
  TrajectoryPoints points = create_sample_trajectory();
  const auto points_size = points.size();
  utils::remove_invalid_points(points);
  ASSERT_EQ(points.size(), points_size);
}

TEST_F(TrajectoryOptimizerUtilsTest, RemoveCloseProximityPoints)
{
  TrajectoryPoints points = create_sample_trajectory();
  const auto points_size = points.size();

  utils::remove_close_proximity_points(points, 1E-2);
  ASSERT_EQ(points.size(), points_size);

  utils::remove_close_proximity_points(points, std::numeric_limits<double>::max());
  ASSERT_EQ(points.size(), 1);
}

TEST_F(TrajectoryOptimizerUtilsTest, ClampVelocities)
{
  TrajectoryPoints points = create_sample_trajectory();
  utils::clamp_velocities(points, 2.0f, 0.5f);
  for (const auto & point : points) {
    ASSERT_GE(point.longitudinal_velocity_mps, 2.0f);
    ASSERT_GE(point.acceleration_mps2, 0.5f);
  }
}

TEST_F(TrajectoryOptimizerUtilsTest, SetMaxVelocity)
{
  TrajectoryPoints points = create_sample_trajectory();
  utils::set_max_velocity(points, 2.0f);
  for (const auto & point : points) {
    ASSERT_LE(point.longitudinal_velocity_mps, 2.0f);
  }
}

TEST_F(TrajectoryOptimizerUtilsTest, ValidatePoint)
{
  TrajectoryPoint valid_point;
  valid_point.longitudinal_velocity_mps = 1.0;
  valid_point.acceleration_mps2 = 0.1;
  valid_point.pose.position.x = 1.0;
  valid_point.pose.position.y = 1.0;
  valid_point.pose.position.z = 1.0;
  valid_point.pose.orientation.x = 0.0;
  valid_point.pose.orientation.y = 0.0;
  valid_point.pose.orientation.z = 0.0;
  valid_point.pose.orientation.w = 1.0;
  ASSERT_TRUE(utils::validate_point(valid_point));

  TrajectoryPoint invalid_point;
  invalid_point.pose.position.x = std::nan("");
  ASSERT_FALSE(utils::validate_point(invalid_point));
}

TEST_F(TrajectoryOptimizerUtilsTest, ApplySpline)
{
  TrajectoryPoints points = create_sample_trajectory();
  TrajectoryOptimizerParams params;
  params.spline_interpolation_resolution_m = 0.1;
  utils::apply_spline(points, params);
  ASSERT_GE(points.size(), 2);
}

TEST_F(TrajectoryOptimizerUtilsTest, AddEgoStateToTrajectory)
{
  TrajectoryPoints points = create_sample_trajectory();
  Odometry current_odometry;
  current_odometry.pose.pose.position.x = 1.0;
  current_odometry.pose.pose.position.y = 1.0;
  TrajectoryOptimizerParams params;
  utils::add_ego_state_to_trajectory(points, current_odometry, params);
  ASSERT_FALSE(points.empty());
}

TEST_F(TrajectoryOptimizerUtilsTest, ExpandTrajectoryWithEgoHistory)
{
  TrajectoryPoints points = create_sample_trajectory();
  TrajectoryPoints ego_history_points = create_sample_trajectory(1.0, -10.0);
  Odometry current_odometry;
  TrajectoryOptimizerParams params;
  params.backward_trajectory_extension_m = 15.0;
  current_odometry.pose.pose.position.x = points.front().pose.position.x;
  current_odometry.pose.pose.position.y = points.front().pose.position.y;
  utils::expand_trajectory_with_ego_history(points, ego_history_points, current_odometry, params);
  ASSERT_GE(points.size(), 20);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
