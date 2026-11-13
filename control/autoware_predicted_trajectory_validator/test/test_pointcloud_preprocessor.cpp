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

#include "autoware/predicted_trajectory_validator/collision_validator/pointcloud_preprocessor.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/logging.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <boost/geometry.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace autoware::predicted_trajectory_validator::collision_validator
{
namespace
{
struct Xyz
{
  float x;
  float y;
  float z;
};

enum class Layout { XYZIRC, XYZIRCAEDT };

// Same vehicle as autoware_test_utils/config/test_vehicle_info.param.yaml: footprint
// x in [-1.1, 3.79], |y| <= 0.948, height 2.5 m.
vehicle_info_utils::VehicleInfo test_vehicle_info(const double height = 2.5)
{
  return vehicle_info_utils::createVehicleInfo(
    0.383, 0.235, 2.79, 1.64, 1.0, 1.1, 0.128, 0.128, height, 0.70);
}

sensor_msgs::msg::PointCloud2 make_cloud(
  const std::vector<Xyz> & points, const Layout layout, const double stamp_sec = 10.0,
  const std::string & frame_id = "base_link")
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp_sec * 1e9), RCL_ROS_TIME);
  cloud.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  using sensor_msgs::msg::PointField;
  if (layout == Layout::XYZIRC) {
    modifier.setPointCloud2Fields(
      6, "x", 1, PointField::FLOAT32, "y", 1, PointField::FLOAT32, "z", 1, PointField::FLOAT32,
      "intensity", 1, PointField::UINT8, "return_type", 1, PointField::UINT8, "channel", 1,
      PointField::UINT16);
  } else {
    modifier.setPointCloud2Fields(
      10, "x", 1, PointField::FLOAT32, "y", 1, PointField::FLOAT32, "z", 1, PointField::FLOAT32,
      "intensity", 1, PointField::UINT8, "return_type", 1, PointField::UINT8, "channel", 1,
      PointField::UINT16, "azimuth", 1, PointField::FLOAT32, "elevation", 1, PointField::FLOAT32,
      "distance", 1, PointField::FLOAT32, "time_stamp", 1, PointField::UINT32);
  }
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
  for (const auto & p : points) {
    *it_x = p.x;
    *it_y = p.y;
    *it_z = p.z;
    ++it_x;
    ++it_y;
    ++it_z;
  }
  cloud.height = 1;
  cloud.width = points.size();
  return cloud;
}

// 5 x 5 grid at 0.1 m spacing, 25 points.
void add_blob(std::vector<Xyz> & points, const float x0, const float y0, const float z = 1.0F)
{
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      points.push_back({x0 + 0.1F * i, y0 + 0.1F * j, z});
    }
  }
}

std::vector<Xyz> two_blobs_and_an_isolated_point()
{
  std::vector<Xyz> points;
  add_blob(points, 5.0F, 0.0F);
  add_blob(points, 8.0F, 2.0F);
  points.push_back({3.0F, -3.0F, 1.0F});
  return points;
}

PointCloudParam default_param()
{
  PointCloudParam param;
  param.roi_rear = 2.0;
  param.roi_front_margin = 2.0;
  param.roi_lateral = 5.0;
  param.z_margin_low = 0.0;
  param.z_margin_high = 0.0;
  param.max_voxels = 20000;
  param.voxel_size = 0.10;
  param.cluster_tolerance = 0.30;
  param.min_cluster_size = 3;
  return param;
}

PointCloudPreprocessor make_preprocessor(const PointCloudParam & param = default_param())
{
  return PointCloudPreprocessor(rclcpp::get_logger("test_pointcloud_preprocessor"), param);
}

geometry_msgs::msg::Pose make_pose(const double x, const double y, const double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
  return pose;
}

const std::optional<geometry_msgs::msg::Pose> kOrigin = make_pose(0.0, 0.0, 0.0);

std::vector<PointCluster> sorted_by_x(std::vector<PointCluster> clusters)
{
  std::sort(clusters.begin(), clusters.end(), [](const auto & a, const auto & b) {
    return a.centroid.x() < b.centroid.x();
  });
  return clusters;
}

bool contains_point(const PointCluster & cluster, const double x, const double y)
{
  return std::any_of(cluster.points.begin(), cluster.points.end(), [&](const auto & p) {
    return std::abs(p.x() - x) < 1e-6 && std::abs(p.y() - y) < 1e-6;
  });
}
}  // namespace

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

TEST(PointCloudPreprocessorTest, BothPointLayoutsGiveTheSameClusters)
{
  auto preprocessor_irc = make_preprocessor();
  auto preprocessor_aedt = make_preprocessor();
  const auto points = two_blobs_and_an_isolated_point();

  const auto & result_irc = preprocessor_irc.process(
    make_cloud(points, Layout::XYZIRC), kOrigin, 10.0, 5.0, test_vehicle_info());
  const auto & result_aedt = preprocessor_aedt.process(
    make_cloud(points, Layout::XYZIRCAEDT), kOrigin, 10.0, 5.0, test_vehicle_info());
  ASSERT_TRUE(result_irc.has_value());
  ASSERT_TRUE(result_aedt.has_value());

  const auto a = sorted_by_x(result_irc->clusters);
  const auto b = sorted_by_x(result_aedt->clusters);
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    ASSERT_EQ(a[i].points.size(), b[i].points.size());
    EXPECT_EQ(a[i].raw_point_count, b[i].raw_point_count);
    for (size_t j = 0; j < a[i].points.size(); ++j) {
      EXPECT_TRUE(boost::geometry::equals(a[i].points[j], b[i].points[j]));
    }
  }
}

TEST(PointCloudPreprocessorTest, ClustersAreExpressedInMapThroughEgoPose)
{
  auto preprocessor = make_preprocessor();
  std::vector<Xyz> points;
  add_blob(points, 5.0F, 0.0F);
  const auto cloud = make_cloud(points, Layout::XYZIRC);

  // Ego at (100, 50) heading +y: base_link (5.2, 0.2) -> map (100 - 0.2, 50 + 5.2).
  const auto & result =
    preprocessor.process(cloud, make_pose(100.0, 50.0, M_PI / 2.0), 10.0, 5.0, test_vehicle_info());
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->clusters.size(), 1U);
  const auto & c = result->clusters.front().centroid;
  EXPECT_NEAR(c.x(), 100.0 - 0.2, 0.1);
  EXPECT_NEAR(c.y(), 50.0 + 5.2, 0.1);
}

TEST(PointCloudPreprocessorTest, RoiDependsOnVelocityAndHorizon)
{
  auto preprocessor = make_preprocessor();
  std::vector<Xyz> points;
  add_blob(points, 5.0F, 0.0F);
  add_blob(points, 30.0F, 0.0F);
  const auto cloud = make_cloud(points, Layout::XYZIRC);

  // v = 2 m/s, horizon 5 s -> x <= 3.79 + 10 + 2 = 15.79 m: the blob at 30 m is outside.
  const auto & result = preprocessor.process(cloud, kOrigin, 2.0, 5.0, test_vehicle_info());
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->clusters.size(), 1U);
  EXPECT_NEAR(result->clusters.front().centroid.x(), 5.2, 0.1);
}

TEST(PointCloudPreprocessorTest, RoiIsMeasuredFromTheVehicleEndsAndExtendsBehindWhenReversing)
{
  std::vector<Xyz> points;
  add_blob(points, -9.0F, 0.0F);
  add_blob(points, -4.0F, 0.0F);
  add_blob(points, -3.0F, 0.0F);
  add_blob(points, 4.0F, 0.0F);
  add_blob(points, 6.5F, 0.0F);
  const auto cloud = make_cloud(points, Layout::XYZIRC);

  auto stopped = make_preprocessor();
  const auto & stopped_result = stopped.process(cloud, kOrigin, 0.0, 5.0, test_vehicle_info());
  ASSERT_TRUE(stopped_result.has_value());
  const auto stopped_clusters = sorted_by_x(stopped_result->clusters);
  ASSERT_EQ(stopped_clusters.size(), 2U);
  EXPECT_NEAR(stopped_clusters[0].centroid.x(), -2.8, 0.1);
  EXPECT_NEAR(stopped_clusters[1].centroid.x(), 4.2, 0.1);

  auto reversing = make_preprocessor();
  const auto & reversing_result = reversing.process(cloud, kOrigin, -1.0, 5.0, test_vehicle_info());
  ASSERT_TRUE(reversing_result.has_value());
  const auto reversing_clusters = sorted_by_x(reversing_result->clusters);
  ASSERT_EQ(reversing_clusters.size(), 3U);
  EXPECT_NEAR(reversing_clusters[0].centroid.x(), -3.8, 0.1);
  EXPECT_NEAR(reversing_clusters[1].centroid.x(), -2.8, 0.1);
  EXPECT_NEAR(reversing_clusters[2].centroid.x(), 4.2, 0.1);
}

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
  const auto & low_result =
    low.process(make_cloud(points, Layout::XYZIRC), kOrigin, 10.0, 5.0, test_vehicle_info(2.5));
  ASSERT_TRUE(low_result.has_value());
  EXPECT_TRUE(low_result->clusters.empty());

  const auto & high_result = low.process(
    make_cloud(points, Layout::XYZIRC, 11.0), kOrigin, 10.0, 5.0, test_vehicle_info(3.0));
  ASSERT_TRUE(high_result.has_value());
  EXPECT_EQ(high_result->clusters.size(), 1U);

  auto param = default_param();
  param.z_margin_high = 0.5;
  auto margin = make_preprocessor(param);
  const auto & margin_result =
    margin.process(make_cloud(points, Layout::XYZIRC), kOrigin, 10.0, 5.0, test_vehicle_info(2.5));
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
        make_cloud(points, Layout::XYZIRC, 10.0, "lidar_top"), kOrigin, 10.0, 5.0, vehicle_info)
      .has_value());
  EXPECT_FALSE(
    preprocessor
      .process(make_cloud(points, Layout::XYZIRC, 10.0), std::nullopt, 10.0, 5.0, vehicle_info)
      .has_value());

  const auto & processed = preprocessor.process(
    make_cloud(points, Layout::XYZIRC, 10.0), kOrigin, 10.0, 5.0, vehicle_info);
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
    make_cloud(one_blob, Layout::XYZIRC, 10.0), kOrigin, 10.0, 5.0, vehicle_info));
  expect_previous(preprocessor.process(
    make_cloud(one_blob, Layout::XYZIRC, 11.0, "lidar_top"), kOrigin, 10.0, 5.0, vehicle_info));
  expect_previous(preprocessor.process(
    make_cloud(one_blob, Layout::XYZIRC, 11.0), std::nullopt, 10.0, 5.0, vehicle_info));

  const auto & next = preprocessor.process(
    make_cloud(one_blob, Layout::XYZIRC, 11.0), kOrigin, 10.0, 5.0, vehicle_info);
  ASSERT_TRUE(next.has_value());
  EXPECT_EQ(next->clusters.size(), 1U);
}

TEST(PointCloudPreprocessorTest, EmptyFrameGivesNoClusters)
{
  auto preprocessor = make_preprocessor();
  const auto vehicle_info = test_vehicle_info();

  const auto & first =
    preprocessor.process(make_cloud({}, Layout::XYZIRC, 9.0), kOrigin, 10.0, 5.0, vehicle_info);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->stamp, rclcpp::Time(9'000'000'000, RCL_ROS_TIME));
  EXPECT_TRUE(first->clusters.empty());

  const auto & processed = preprocessor.process(
    make_cloud(two_blobs_and_an_isolated_point(), Layout::XYZIRC, 10.0), kOrigin, 10.0, 5.0,
    vehicle_info);
  ASSERT_TRUE(processed.has_value());
  EXPECT_EQ(processed->clusters.size(), 2U);

  const auto & empty =
    preprocessor.process(make_cloud({}, Layout::XYZIRC, 11.0), kOrigin, 10.0, 5.0, vehicle_info);
  ASSERT_TRUE(empty.has_value());
  EXPECT_EQ(empty->stamp, rclcpp::Time(11'000'000'000, RCL_ROS_TIME));
  EXPECT_TRUE(empty->clusters.empty());
}

TEST(PointCloudPreprocessorTest, VoxelPointIsTheCentroidAndRawCountIsThePointCountWithNegatives)
{
  auto preprocessor = make_preprocessor();
  const std::vector<Xyz> voxel_a = {
    {-1.58F, -0.28F, 1.02F},
    {-1.52F, -0.21F, 1.08F},
    {-1.55F, -0.25F, 1.05F},
    {-1.51F, -0.29F, 1.01F},
    {-1.59F, -0.22F, 1.09F}};
  const std::vector<Xyz> voxel_b = {{-1.55F, -0.05F, 1.05F}, {-1.53F, -0.03F, 1.05F}};
  const std::vector<Xyz> voxel_c = {
    {-1.55F, 0.05F, 1.05F}, {-1.57F, 0.03F, 1.05F}, {-1.53F, 0.07F, 1.05F}};
  std::vector<Xyz> points;
  for (const auto * voxel : {&voxel_a, &voxel_b, &voxel_c}) {
    points.insert(points.end(), voxel->begin(), voxel->end());
  }

  const auto & result = preprocessor.process(
    make_cloud(points, Layout::XYZIRC), kOrigin, 10.0, 5.0, test_vehicle_info());
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->clusters.size(), 1U);
  const auto & cluster = result->clusters.front();
  EXPECT_EQ(cluster.raw_point_count, 10U);
  ASSERT_EQ(cluster.points.size(), 3U);
  for (const auto * voxel : {&voxel_a, &voxel_b, &voxel_c}) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (const auto & p : *voxel) {
      sum_x += static_cast<double>(p.x);
      sum_y += static_cast<double>(p.y);
    }
    const auto n = static_cast<double>(voxel->size());
    EXPECT_TRUE(contains_point(cluster, sum_x / n, sum_y / n));
  }
}

TEST(PointCloudPreprocessorTest, SubsamplingToMaxVoxelsKeepsTheRawCountOfEachVoxel)
{
  auto param = default_param();
  param.max_voxels = 5;
  auto preprocessor = make_preprocessor(param);
  std::vector<Xyz> points;
  for (int i = 0; i < 10; ++i) {
    const float x = 5.05F + 0.1F * static_cast<float>(i);
    for (const float d : {-0.02F, 0.0F, 0.02F}) {
      points.push_back({x + d, 0.05F, 1.05F});
    }
  }

  const auto & result = preprocessor.process(
    make_cloud(points, Layout::XYZIRC), kOrigin, 10.0, 5.0, test_vehicle_info());
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->clusters.size(), 1U);
  EXPECT_EQ(result->clusters.front().points.size(), 5U);
  EXPECT_EQ(result->clusters.front().raw_point_count, 15U);
}

TEST(PointCloudPreprocessorTest, ClustersAreFilteredByTheirRawPointCount)
{
  auto param = default_param();
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
