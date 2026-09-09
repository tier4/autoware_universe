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

#include "autoware/ml_planner/utils/utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::ml_planner::test
{

class UtilsTest : public ::testing::Test
{
protected:
  void SetUp() override {}
};

TEST_F(UtilsTest, GetTransformMatrixIdentity)
{
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 1.0;

  const Eigen::Matrix4d bl2map = utils::pose_to_matrix4d(odom.pose.pose);
  const Eigen::Matrix4d map2bl = utils::inverse(bl2map);

  Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) EXPECT_NEAR(bl2map(i, j), I(i, j), 1e-6);
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) EXPECT_NEAR(map2bl(i, j), I(i, j), 1e-6);
}

TEST_F(UtilsTest, GetTransformMatrixTranslation)
{
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 1.0;
  odom.pose.pose.position.y = 2.0;
  odom.pose.pose.position.z = 3.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 1.0;

  const Eigen::Matrix4d bl2map = utils::pose_to_matrix4d(odom.pose.pose);
  const Eigen::Matrix4d map2bl = utils::inverse(bl2map);

  EXPECT_FLOAT_EQ(bl2map(0, 3), 1.0f);
  EXPECT_FLOAT_EQ(bl2map(1, 3), 2.0f);
  EXPECT_FLOAT_EQ(bl2map(2, 3), 3.0f);

  Eigen::Vector3f t(1.0f, 2.0f, 3.0f);
  Eigen::Vector3f inv_t = -t;
  EXPECT_FLOAT_EQ(map2bl(0, 3), inv_t.x());
  EXPECT_FLOAT_EQ(map2bl(1, 3), inv_t.y());
  EXPECT_FLOAT_EQ(map2bl(2, 3), inv_t.z());
}

TEST_F(UtilsTest, GetTransformMatrixRotation)
{
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;
  // 90 degree rotation around Z axis
  double angle = M_PI_2;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = std::sin(angle / 2);
  odom.pose.pose.orientation.w = std::cos(angle / 2);

  const Eigen::Matrix4d bl2map = utils::pose_to_matrix4d(odom.pose.pose);
  const Eigen::Matrix4d map2bl = utils::inverse(bl2map);

  // The rotation part should be a 90 degree rotation matrix
  Eigen::Matrix3f R;
  R << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) EXPECT_NEAR(bl2map(i, j), R(i, j), 1e-6);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) EXPECT_NEAR(map2bl(i, j), R.transpose()(i, j), 1e-6);
}

TEST_F(UtilsTest, CheckInputMapValid)
{
  std::unordered_map<std::string, xt::xarray<float>> input_map;
  input_map["a"] = {1.0f, 2.0f, 3.0f};
  input_map["b"] = {0.0f, -1.0f, 42.0f};
  EXPECT_TRUE(utils::check_input_map(input_map));
}

TEST_F(UtilsTest, CheckInputMapWithInf)
{
  std::unordered_map<std::string, xt::xarray<float>> input_map;
  input_map["a"] = {1.0f, std::numeric_limits<float>::infinity()};
  EXPECT_FALSE(utils::check_input_map(input_map));
}

TEST_F(UtilsTest, CheckInputMapWithNaN)
{
  std::unordered_map<std::string, xt::xarray<float>> input_map;
  input_map["a"] = {1.0f, std::nanf("")};
  EXPECT_FALSE(utils::check_input_map(input_map));
}

TEST_F(UtilsTest, CheckInputMapEmpty)
{
  std::unordered_map<std::string, xt::xarray<float>> input_map;
  EXPECT_TRUE(utils::check_input_map(input_map));
}

TEST_F(UtilsTest, ReplicateForBatchPreservesSampleShape)
{
  const xt::xarray<float> single_data = {{1.0f, 2.0f}, {3.0f, 4.0f}, {5.0f, 6.0f}};

  const auto batch_data = utils::replicate_for_batch(single_data, 2);

  ASSERT_EQ(batch_data.dimension(), 3);
  EXPECT_EQ(batch_data.shape()[0], 2);
  EXPECT_EQ(batch_data.shape()[1], 3);
  EXPECT_EQ(batch_data.shape()[2], 2);
  EXPECT_EQ(batch_data(0, 2, 1), 6.0f);
  EXPECT_EQ(batch_data(1, 2, 1), 6.0f);
}

}  // namespace autoware::ml_planner::test
