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

#include "test_utils.hpp"

#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <vector>

namespace autoware::predicted_trajectory_validator::collision_validator
{
using namespace test_utils;  // NOLINT(build/namespaces)

TEST(PointCloudPreprocessorTest, TwoBlobsGiveTwoPointClustersAndIsolatedPointIsDropped)
{
  auto preprocessor = make_preprocessor();
  const auto cloud = make_cloud(two_blobs_and_an_isolated_point(), Layout::XYZIRC);

  const auto & result = preprocessor.process(cloud, kOrigin, 10.0, 5.0, test_vehicle_info());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->stamp, rclcpp::Time(cloud.header.stamp));
  const auto clusters = sorted_by_x(result->clusters);
  ASSERT_EQ(clusters.size(), 2U);

  EXPECT_NEAR(clusters[0].centroid.x(), 5.2, 0.1);
  EXPECT_NEAR(clusters[0].centroid.y(), 0.2, 0.1);
  EXPECT_NEAR(clusters[1].centroid.x(), 8.2, 0.1);
  EXPECT_NEAR(clusters[1].centroid.y(), 2.2, 0.1);
  // The points of the blob are kept, one per 0.1 m voxel.
  EXPECT_GE(clusters[0].points.size(), 9U);
  EXPECT_LE(clusters[0].points.size(), 25U);
  EXPECT_EQ(clusters[0].raw_point_count, 25U);
  for (const auto & p : clusters[0].points) {
    EXPECT_GE(p.x(), 4.9);
    EXPECT_LE(p.x(), 5.5);
    EXPECT_GE(p.y(), -0.1);
    EXPECT_LE(p.y(), 0.5);
  }
}

struct RoiCase
{
  std::string name;
  std::vector<float> blob_xs;
  double velocity;
  std::vector<double> expected_centroid_xs;
};

class RoiTest : public ::testing::TestWithParam<RoiCase>
{
};

TEST_P(RoiTest, OnlyClustersInsideTheRoiRemain)
{
  std::vector<Xyz> points;
  for (const auto x : GetParam().blob_xs) {
    add_blob(points, x, 0.0F);
  }
  auto preprocessor = make_preprocessor();
  const auto & result = preprocessor.process(
    make_cloud(points, Layout::XYZIRC), kOrigin, GetParam().velocity, 5.0, test_vehicle_info());
  ASSERT_TRUE(result.has_value());
  const auto clusters = sorted_by_x(result->clusters);
  const auto & expected = GetParam().expected_centroid_xs;
  ASSERT_EQ(clusters.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(clusters[i].centroid.x(), expected[i], 0.1) << "cluster " << i;
  }
}

INSTANTIATE_TEST_SUITE_P(
  PointCloudPreprocessorTest, RoiTest,
  ::testing::Values(
    // v = 2 m/s, horizon 5 s -> x <= 3.79 + 10 + 2 = 15.79 m: the blob at 30 m is outside.
    RoiCase{"Forward", {5.0F, 30.0F}, 2.0, {5.2}},
    RoiCase{"Stopped", {-9.0F, -4.0F, -3.0F, 4.0F, 6.5F}, 0.0, {-2.8, 4.2}},
    RoiCase{"Reversing", {-9.0F, -4.0F, -3.0F, 4.0F, 6.5F}, -1.0, {-3.8, -2.8, 4.2}}),
  [](const ::testing::TestParamInfo<RoiCase> & info) { return info.param.name; });

TEST(PointCloudPreprocessorTest, PointsInsideTheEgoFootprintAreDropped)
{
  auto preprocessor = make_preprocessor();
  std::vector<Xyz> points;
  add_blob(points, 1.0F, -0.2F);  // inside the footprint (x in [-1.1, 3.79], |y| <= 0.948)
  add_blob(points, 5.0F, 0.0F);
  const auto cloud = make_cloud(points, Layout::XYZIRC);

  const auto & result = preprocessor.process(cloud, kOrigin, 10.0, 5.0, test_vehicle_info());
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->clusters.size(), 1U);
  EXPECT_NEAR(result->clusters.front().centroid.x(), 5.2, 0.1);
  EXPECT_EQ(result->clusters.front().raw_point_count, 25U);
}

TEST(PointCloudPreprocessorTest, HeightRangeFollowsVehicleInfo)
{
  std::vector<Xyz> points;
  add_blob(points, 5.0F, 0.0F, 2.7F);  // above a 2.5 m vehicle, below a 3.0 m vehicle

  auto low = make_preprocessor();
  const auto & low_result = low.process(
    make_cloud(points, Layout::XYZIRC), kOrigin, 10.0, 5.0, test_vehicle_info(1.0, 2.5));
  ASSERT_TRUE(low_result.has_value());
  EXPECT_TRUE(low_result->clusters.empty());

  const auto & high_result = low.process(
    make_cloud(points, Layout::XYZIRC, at(11.0)), kOrigin, 10.0, 5.0, test_vehicle_info(1.0, 3.0));
  ASSERT_TRUE(high_result.has_value());
  EXPECT_EQ(high_result->clusters.size(), 1U);

  auto param = default_pointcloud_param();
  param.z_margin_high = 0.5;
  auto margin = make_preprocessor(param);
  const auto & margin_result = margin.process(
    make_cloud(points, Layout::XYZIRC), kOrigin, 10.0, 5.0, test_vehicle_info(1.0, 2.5));
  ASSERT_TRUE(margin_result.has_value());
  EXPECT_EQ(margin_result->clusters.size(), 1U);
}

TEST(PointCloudPreprocessorTest, RejectedFramesReturnThePreviousResult)
{
  auto preprocessor = make_preprocessor();
  const auto points = two_blobs_and_an_isolated_point();
  const auto vehicle_info = test_vehicle_info();

  EXPECT_FALSE(
    preprocessor
      .process(
        make_cloud(points, Layout::XYZIRC, at(10.0), "lidar_top"), kOrigin, 10.0, 5.0, vehicle_info)
      .has_value());
  EXPECT_FALSE(
    preprocessor
      .process(make_cloud(points, Layout::XYZIRC, at(10.0)), std::nullopt, 10.0, 5.0, vehicle_info)
      .has_value());

  const auto & processed = preprocessor.process(
    make_cloud(points, Layout::XYZIRC, at(10.0)), kOrigin, 10.0, 5.0, vehicle_info);
  ASSERT_TRUE(processed.has_value());
  EXPECT_EQ(processed->clusters.size(), 2U);
  const rclcpp::Time processed_stamp = processed->stamp;

  std::vector<Xyz> one_blob;
  add_blob(one_blob, 5.0F, 0.0F);
  const auto expect_previous = [&](const std::optional<PointCloudClusters> & result) {
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->stamp, processed_stamp);
    EXPECT_EQ(result->clusters.size(), 2U);
  };
  expect_previous(preprocessor.process(
    make_cloud(one_blob, Layout::XYZIRC, at(10.0)), kOrigin, 10.0, 5.0, vehicle_info));
  expect_previous(preprocessor.process(
    make_cloud(one_blob, Layout::XYZIRC, at(11.0), "lidar_top"), kOrigin, 10.0, 5.0, vehicle_info));
  expect_previous(preprocessor.process(
    make_cloud(one_blob, Layout::XYZIRC, at(11.0)), std::nullopt, 10.0, 5.0, vehicle_info));

  const auto & next = preprocessor.process(
    make_cloud(one_blob, Layout::XYZIRC, at(11.0)), kOrigin, 10.0, 5.0, vehicle_info);
  ASSERT_TRUE(next.has_value());
  EXPECT_EQ(next->clusters.size(), 1U);
}

TEST(PointCloudPreprocessorTest, EmptyFrameGivesNoClusters)
{
  auto preprocessor = make_preprocessor();
  const auto vehicle_info = test_vehicle_info();

  const auto & first =
    preprocessor.process(make_cloud({}, Layout::XYZIRC, at(9.0)), kOrigin, 10.0, 5.0, vehicle_info);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->stamp, rclcpp::Time(9'000'000'000, RCL_ROS_TIME));
  EXPECT_TRUE(first->clusters.empty());

  const auto & processed = preprocessor.process(
    make_cloud(two_blobs_and_an_isolated_point(), Layout::XYZIRC, at(10.0)), kOrigin, 10.0, 5.0,
    vehicle_info);
  ASSERT_TRUE(processed.has_value());
  EXPECT_EQ(processed->clusters.size(), 2U);

  const auto & empty = preprocessor.process(
    make_cloud({}, Layout::XYZIRC, at(11.0)), kOrigin, 10.0, 5.0, vehicle_info);
  ASSERT_TRUE(empty.has_value());
  EXPECT_EQ(empty->stamp, rclcpp::Time(11'000'000'000, RCL_ROS_TIME));
  EXPECT_TRUE(empty->clusters.empty());
}

TEST(PointCloudPreprocessorTest, ClustersAreFilteredByTheirRawPointCount)
{
  auto param = default_pointcloud_param();
  param.min_cluster_size = 5;
  auto preprocessor = make_preprocessor(param);
  std::vector<Xyz> points;
  points.push_back({5.05F, 0.05F, 1.05F});
  points.push_back({5.15F, 0.05F, 1.05F});
  for (const float x : {8.05F, 8.15F}) {
    for (const float d : {-0.02F, -0.01F, 0.01F, 0.02F}) {
      points.push_back({x + d, 0.05F, 1.05F});
    }
  }
  const auto cloud = make_cloud(points, Layout::XYZIRC);

  const auto & result = preprocessor.process(cloud, kOrigin, 10.0, 5.0, test_vehicle_info());
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->clusters.size(), 1U);
  const auto & cluster = result->clusters.front();
  EXPECT_NEAR(cluster.centroid.x(), 8.1, 0.05);
  EXPECT_EQ(cluster.points.size(), 2U);
  EXPECT_EQ(cluster.raw_point_count, 8U);
}
}  // namespace autoware::predicted_trajectory_validator::collision_validator
