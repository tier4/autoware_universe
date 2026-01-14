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

TEST_F(PreprocessingUtilsTest, CreatesEgoCurrentState)
{
  nav_msgs::msg::Odometry odometry_msg;
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration_msg;

  odometry_msg.twist.twist.linear.x = 3.0;
  odometry_msg.twist.twist.linear.y = 0.0;
  odometry_msg.twist.twist.angular.z = 0.1;

  acceleration_msg.accel.accel.linear.x = 0.5;
  acceleration_msg.accel.accel.linear.y = 0.2;

  const auto ego_state = preprocess::create_ego_current_state(odometry_msg, acceleration_msg, 2.5f);

  ASSERT_EQ(ego_state.size(), 10U);
  EXPECT_FLOAT_EQ(ego_state[0], 0.0f);
  EXPECT_FLOAT_EQ(ego_state[1], 0.0f);
  EXPECT_FLOAT_EQ(ego_state[2], 1.0f);
  EXPECT_FLOAT_EQ(ego_state[3], 0.0f);
  EXPECT_FLOAT_EQ(ego_state[4], 3.0f);
  EXPECT_FLOAT_EQ(ego_state[5], 0.0f);
  EXPECT_FLOAT_EQ(ego_state[6], 0.5f);
  EXPECT_FLOAT_EQ(ego_state[7], 0.2f);
  EXPECT_NEAR(ego_state[8], std::atan(0.1 * 2.5 / 3.0), 1e-5);
  EXPECT_NEAR(ego_state[9], 0.1, 1e-5);
}

}  // namespace autoware::diffusion_planner::test
