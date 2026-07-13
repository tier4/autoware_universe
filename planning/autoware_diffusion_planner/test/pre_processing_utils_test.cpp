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

#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::diffusion_planner::test
{

class PreprocessingUtilsTest : public ::testing::Test
{
protected:
  void SetUp() override {}
};

TEST_F(PreprocessingUtilsTest, NormalizesSingleFeatureCorrectly)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  input_data_map["feature"] = {2.0f, 4.0f, 6.0f};
  normalization_map["feature"] = {{2.0f, 2.0f, 2.0f}, {2.0f, 2.0f, 2.0f}};

  preprocess::normalize_input_data(input_data_map, normalization_map);

  EXPECT_FLOAT_EQ(input_data_map["feature"][0], 0.0f);
  EXPECT_FLOAT_EQ(input_data_map["feature"][1], 1.0f);
  EXPECT_FLOAT_EQ(input_data_map["feature"][2], 2.0f);
}

TEST_F(PreprocessingUtilsTest, NormalizesMultipleFeaturesAndRows)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  // 2 rows, 3 cols
  input_data_map["f"] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
  normalization_map["f"] = {{1.0f, 2.0f, 3.0f}, {1.0f, 1.0f, 1.0f}};

  preprocess::normalize_input_data(input_data_map, normalization_map);

  // First row: (1-1)/1=0, (2-2)/1=0, (3-3)/1=0
  // Second row: (4-1)/1=3, (5-2)/1=3, (6-3)/1=3
  EXPECT_FLOAT_EQ(input_data_map["f"][0], 0.0f);
  EXPECT_FLOAT_EQ(input_data_map["f"][1], 0.0f);
  EXPECT_FLOAT_EQ(input_data_map["f"][2], 0.0f);
  EXPECT_FLOAT_EQ(input_data_map["f"][3], 3.0f);
  EXPECT_FLOAT_EQ(input_data_map["f"][4], 3.0f);
  EXPECT_FLOAT_EQ(input_data_map["f"][5], 3.0f);
}

TEST_F(PreprocessingUtilsTest, SkipsZeroRows)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  // 2 rows, 2 cols, first row is all zeros
  input_data_map["f"] = {0.0f, 0.0f, 5.0f, 7.0f};
  normalization_map["f"] = {{1.0f, 2.0f}, {1.0f, 2.0f}};

  preprocess::normalize_input_data(input_data_map, normalization_map);

  // First row should remain zeros
  EXPECT_FLOAT_EQ(input_data_map["f"][0], 0.0f);
  EXPECT_FLOAT_EQ(input_data_map["f"][1], 0.0f);
  // Second row: (5-1)/1=4, (7-2)/2=2.5
  EXPECT_FLOAT_EQ(input_data_map["f"][2], 4.0f);
  EXPECT_FLOAT_EQ(input_data_map["f"][3], 2.5f);
}

TEST_F(PreprocessingUtilsTest, ThrowsIfMissingNormalizationKey)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  input_data_map["f"] = {1.0f, 2.0f};

  EXPECT_THROW(
    preprocess::normalize_input_data(input_data_map, normalization_map), std::runtime_error);
}

TEST_F(PreprocessingUtilsTest, HandlesZeroStdDev)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  input_data_map["f"] = {5.0f, 5.0f};
  normalization_map["f"] = {{5.0f, 5.0f}, {0.0f, 0.0f}};

  // Should not throw, but result should be inf or nan depending on implementation
  EXPECT_THROW(
    preprocess::normalize_input_data(input_data_map, normalization_map), std::runtime_error);
}

TEST_F(PreprocessingUtilsTest, HandlesSingleMeanStdForAllCols)
{
  preprocess::InputDataMap input_data_map;
  preprocess::NormalizationMap normalization_map;

  input_data_map["f"] = {2.0f, 4.0f, 6.0f, 8.0f};
  normalization_map["f"] = {{2.0f}, {2.0f}};  // mean=2, std=2 for all

  preprocess::normalize_input_data(input_data_map, normalization_map);

  EXPECT_FLOAT_EQ(input_data_map["f"][0], 0.0f);
  EXPECT_FLOAT_EQ(input_data_map["f"][1], 1.0f);
  EXPECT_FLOAT_EQ(input_data_map["f"][2], 2.0f);
  EXPECT_FLOAT_EQ(input_data_map["f"][3], 3.0f);
}

TEST_F(PreprocessingUtilsTest, CreateEgoCurrentState)
{
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = 5.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = 0.1;

  geometry_msgs::msg::AccelWithCovarianceStamped accel;
  accel.accel.accel.linear.x = 1.0;
  accel.accel.accel.linear.y = 0.5;

  const float wheel_base = 2.5f;
  const auto result = preprocess::create_ego_current_state(odom, accel, wheel_base);

  // Should return 10 elements: x, y, cos_yaw, sin_yaw, vx, vy, ax, ay, steering, yaw_rate
  ASSERT_EQ(result.size(), 10u);

  // x, y should be 0 (ego frame origin)
  EXPECT_FLOAT_EQ(result[0], 0.0f);
  EXPECT_FLOAT_EQ(result[1], 0.0f);

  // cos_yaw, sin_yaw should be 1, 0 (ego frame heading)
  EXPECT_FLOAT_EQ(result[2], 1.0f);
  EXPECT_FLOAT_EQ(result[3], 0.0f);

  // vx, vy from odometry
  EXPECT_FLOAT_EQ(result[4], 5.0f);
  EXPECT_FLOAT_EQ(result[5], 0.0f);

  // ax, ay from acceleration
  EXPECT_FLOAT_EQ(result[6], 1.0f);
  EXPECT_FLOAT_EQ(result[7], 0.5f);

  // steering_angle = atan(yaw_rate * wheel_base / |linear_vel|) = atan(0.1 * 2.5 / 5.0)
  const float expected_steering = std::atan(0.1f * 2.5f / 5.0f);
  EXPECT_FLOAT_EQ(result[8], expected_steering);

  // yaw_rate from odometry (clamped)
  EXPECT_FLOAT_EQ(result[9], 0.1f);
}

TEST_F(PreprocessingUtilsTest, CreateSyntheticEgoHistoryStraightLine)
{
  const size_t num_timesteps = 31;
  const double speed_mps = 1.5;
  const double time_step_s = 0.1;
  const rclcpp::Time current_time(123, 0, RCL_ROS_TIME);

  geometry_msgs::msg::Pose current_pose;
  current_pose.position.x = 10.0;
  current_pose.position.y = -5.0;
  current_pose.orientation.w = 1.0;  // identity: heading along +x

  const auto history = preprocess::create_synthetic_ego_history(
    current_pose, current_time, num_timesteps, speed_mps, time_step_s);

  ASSERT_EQ(history.size(), num_timesteps);

  // Last entry is the current pose at the current time
  EXPECT_DOUBLE_EQ(history.back().pose.pose.position.x, current_pose.position.x);
  EXPECT_DOUBLE_EQ(history.back().pose.pose.position.y, current_pose.position.y);
  EXPECT_EQ(rclcpp::Time(history.back().header.stamp), current_time);

  for (size_t t = 0; t < num_timesteps; ++t) {
    const auto & odom = history[t];
    const double steps_back = static_cast<double>(num_timesteps - 1 - t);
    // Poses are shifted backwards along the heading by speed * dt each step
    EXPECT_NEAR(
      odom.pose.pose.position.x, current_pose.position.x - steps_back * speed_mps * time_step_s,
      1e-9);
    EXPECT_NEAR(odom.pose.pose.position.y, current_pose.position.y, 1e-9);
    // Timestamps are spaced time_step_s apart, ending at current_time
    EXPECT_NEAR(
      (current_time - rclcpp::Time(odom.header.stamp)).seconds(), steps_back * time_step_s, 1e-9);
    // Constant longitudinal speed
    EXPECT_DOUBLE_EQ(odom.twist.twist.linear.x, speed_mps);
  }
}

TEST_F(PreprocessingUtilsTest, CreateSyntheticEgoHistoryFollowsHeading)
{
  const size_t num_timesteps = 5;
  const double speed_mps = 2.0;
  const double time_step_s = 0.1;
  const rclcpp::Time current_time(10, 0);

  geometry_msgs::msg::Pose current_pose;
  // Heading along +y (yaw = pi/2)
  current_pose.orientation.z = std::sin(M_PI_4);
  current_pose.orientation.w = std::cos(M_PI_4);

  const auto history = preprocess::create_synthetic_ego_history(
    current_pose, current_time, num_timesteps, speed_mps, time_step_s);

  ASSERT_EQ(history.size(), num_timesteps);
  // Oldest entry is shifted backwards along +y
  EXPECT_NEAR(history.front().pose.pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(
    history.front().pose.pose.position.y,
    -static_cast<double>(num_timesteps - 1) * speed_mps * time_step_s, 1e-9);
}

}  // namespace autoware::diffusion_planner::test
