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

#include "autoware/predicted_trajectory_validator/collision_validator/collision_validator.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace autoware::predicted_trajectory_validator
{
using autoware_internal_control_msgs::msg::HazardSector;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;

namespace
{
constexpr double kEgoVelocity = 10.0;

vehicle_info_utils::VehicleInfo test_vehicle_info()
{
  return vehicle_info_utils::createVehicleInfo(
    0.383, 0.235, 2.79, 1.64, 1.0, 1.1, 0.128, 0.128, 2.5, 0.70);
}

rclcpp::Time at(const double sec)
{
  return rclcpp::Time(static_cast<int64_t>(sec * 1e9), RCL_ROS_TIME);
}

::predicted_trajectory_validator::Params default_params()
{
  ::predicted_trajectory_validator::Params params;
  auto & p = params.collision_validator;
  p.enable_warning = true;
  p.braking.reaction_times = {2.0, 1.5, 1.0};
  p.braking.deceleration = 6.0;
  p.collision_cos_threshold = 0.0;
  p.prediction_horizon = 5.0;
  p.time_resolution = 0.1;
  p.stop_velocity = 0.1;
  p.proximity_distance = 0.3;
  p.vehicle_yaw.rear_axle_ratio = 0.3;
  p.enabled_object_classes = {"car",        "truck",   "bus",       "trailer",
                              "motorcycle", "bicycle", "pedestrian"};
  p.enable_pointcloud = true;
  p.max_evaluations.object = 10;
  p.max_evaluations.pointcloud = 10;
  p.max_objects_stamp_gap_s = 0.5;
  p.pointcloud.roi.rear = 2.0;
  p.pointcloud.roi.front_margin = 2.0;
  p.pointcloud.roi.lateral = 5.0;
  p.pointcloud.roi.z_margin_low = 0.0;
  p.pointcloud.roi.z_margin_high = -0.35;
  p.pointcloud.max_voxels = 20000;
  p.pointcloud.voxel_size = 0.10;
  p.pointcloud.cluster_tolerance = 0.30;
  p.pointcloud.min_cluster_size = 10;
  p.pointcloud.max_stamp_gap_s = 0.1;
  return params;
}

CollisionValidator make_validator(
  const ::predicted_trajectory_validator::Params & params = default_params())
{
  return CollisionValidator(rclcpp::get_logger("test_collision_validator"), params);
}

geometry_msgs::msg::Pose make_pose(const double x, const double y, const double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
  return pose;
}

Odometry make_odom(const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & pose)
{
  Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "map";
  odom.pose.pose = pose;
  odom.twist.twist.linear.x = kEgoVelocity;
  return odom;
}

Trajectory make_trajectory(const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & start)
{
  const double yaw = autoware_utils_geometry::get_rpy(start).z;
  Trajectory trajectory;
  trajectory.header.stamp = stamp;
  trajectory.header.frame_id = "map";
  for (size_t i = 0; i <= 50; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    TrajectoryPoint point;
    point.pose = start;
    point.pose.position.x += kEgoVelocity * t * std::cos(yaw);
    point.pose.position.y += kEgoVelocity * t * std::sin(yaw);
    point.longitudinal_velocity_mps = static_cast<float>(kEgoVelocity);
    point.time_from_start = rclcpp::Duration::from_seconds(t);
    trajectory.points.push_back(point);
  }
  return trajectory;
}

PredictedObject make_car(const double x, const double y, const uint8_t uuid = 0)
{
  PredictedObject object;
  ObjectClassification classification;
  classification.label = ObjectClassification::CAR;
  classification.probability = 1.0;
  object.classification.push_back(classification);
  object.object_id.uuid[0] = uuid;
  object.kinematics.initial_pose_with_covariance.pose = make_pose(x, y, 0.0);
  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 1.0;
  object.shape.dimensions.y = 1.0;
  object.shape.dimensions.z = 1.5;
  return object;
}

PredictedObjects::ConstSharedPtr make_objects(
  const rclcpp::Time & stamp, const std::vector<PredictedObject> & objects)
{
  auto msg = std::make_shared<PredictedObjects>();
  msg->header.stamp = stamp;
  msg->header.frame_id = "map";
  msg->objects = objects;
  return msg;
}

PointCloud2::ConstSharedPtr make_cloud(const rclcpp::Time & stamp, const double x0)
{
  auto cloud = std::make_shared<PointCloud2>();
  cloud->header.stamp = stamp;
  cloud->header.frame_id = "base_link";
  sensor_msgs::PointCloud2Modifier modifier(*cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(25);
  sensor_msgs::PointCloud2Iterator<float> it_x(*cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(*cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(*cloud, "z");
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      *it_x = static_cast<float>(x0) + 0.1F * static_cast<float>(i);
      *it_y = -0.2F + 0.1F * static_cast<float>(j);
      *it_z = 1.0F;
      ++it_x;
      ++it_y;
      ++it_z;
    }
  }
  cloud->height = 1;
  cloud->width = 25;
  return cloud;
}

size_t count_points(const std::vector<HazardSource> & sources)
{
  return std::count_if(sources.begin(), sources.end(), [](const auto & source) {
    return source.shape.outer().size() == 1;
  });
}
}  // namespace

TEST(CollisionValidatorTest, RaisedCandidatesAreHandedOverInBaseLink)
{
  auto validator = make_validator();
  const auto pose = make_pose(100.0, 50.0, M_PI / 2.0);
  const auto now = at(10.0);
  const auto objects = make_objects(now, {make_car(100.0, 65.0), make_car(120.0, 65.0)});

  HazardWarningSources sources;
  validator.validate(
    make_trajectory(now, pose), make_odom(now, pose), objects, test_vehicle_info(), nullptr,
    sources);

  ASSERT_TRUE(sources.collision.has_value());
  EXPECT_FALSE(sources.departure.has_value());
  ASSERT_EQ(sources.collision->size(), 1U);
  const auto & source = sources.collision->front();
  EXPECT_EQ(source.risk_level, HazardSector::HIGH);
  EXPECT_EQ(source.sources, HazardSector::SOURCE_COLLISION);
  EXPECT_GT(source.shape.outer().size(), 1U);
  for (const auto & vertex : source.shape.outer()) {
    EXPECT_NEAR(std::abs(vertex.x() - 15.0), 0.5, 1e-6);
    EXPECT_NEAR(std::abs(vertex.y()), 0.5, 1e-6);
  }

  const auto moved = make_pose(100.0, 55.0, M_PI / 2.0);
  HazardWarningSources moved_sources;
  validator.validate(
    make_trajectory(now, moved), make_odom(at(10.1), moved), objects, test_vehicle_info(), nullptr,
    moved_sources);
  ASSERT_TRUE(moved_sources.collision.has_value());
  ASSERT_EQ(moved_sources.collision->size(), 1U);
  for (const auto & vertex : moved_sources.collision->front().shape.outer()) {
    EXPECT_NEAR(std::abs(vertex.x() - 10.0), 0.5, 1e-6);
  }
}

TEST(CollisionValidatorTest, NothingRaisedGivesAnEmptyArray)
{
  auto validator = make_validator();
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto now = at(10.0);

  HazardWarningSources sources;
  validator.validate(
    make_trajectory(now, pose), make_odom(now, pose), make_objects(now, {make_car(15.0, 5.0)}),
    test_vehicle_info(), nullptr, sources);
  ASSERT_TRUE(sources.collision.has_value());
  EXPECT_TRUE(sources.collision->empty());
}

TEST(CollisionValidatorTest, SkippedCyclesLeaveTheCollisionSourcesUnset)
{
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto now = at(10.0);
  const auto trajectory = make_trajectory(now, pose);
  const auto odom = make_odom(now, pose);
  const auto objects = make_objects(now, {make_car(15.0, 0.0)});
  const auto vehicle_info = test_vehicle_info();

  {
    auto validator = make_validator();
    HazardWarningSources sources;
    validator.validate(trajectory, odom, nullptr, vehicle_info, nullptr, sources);
    EXPECT_FALSE(sources.collision.has_value());
  }
  {
    auto validator = make_validator();
    Trajectory short_trajectory = trajectory;
    short_trajectory.points.resize(1);
    HazardWarningSources sources;
    validator.validate(short_trajectory, odom, objects, vehicle_info, nullptr, sources);
    EXPECT_FALSE(sources.collision.has_value());
  }
  {
    auto validator = make_validator();
    Trajectory unsorted_trajectory = trajectory;
    unsorted_trajectory.points[1].time_from_start = rclcpp::Duration::from_seconds(100.0);
    HazardWarningSources sources;
    validator.validate(unsorted_trajectory, odom, objects, vehicle_info, nullptr, sources);
    EXPECT_FALSE(sources.collision.has_value());
  }
  {
    auto validator = make_validator(default_params());
    HazardWarningSources sources;
    validator.validate(trajectory, odom, nullptr, vehicle_info, nullptr, sources);
    EXPECT_FALSE(sources.collision.has_value());
  }
  {
    auto params = default_params();
    params.collision_validator.enable_pointcloud = false;
    auto validator = make_validator(params);
    HazardWarningSources sources;
    validator.validate(trajectory, odom, nullptr, vehicle_info, make_cloud(now, 15.0), sources);
    EXPECT_FALSE(sources.collision.has_value());
  }
}

TEST(CollisionValidatorTest, EitherInputAloneIsEvaluated)
{
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto now = at(10.0);
  const auto vehicle_info = test_vehicle_info();

  {
    auto validator = make_validator(default_params());
    HazardWarningSources sources;
    validator.validate(
      make_trajectory(now, pose), make_odom(now, pose), make_objects(now, {make_car(15.0, 0.0)}),
      vehicle_info, nullptr, sources);
    ASSERT_TRUE(sources.collision.has_value());
    EXPECT_EQ(sources.collision->size(), 1U);
    EXPECT_EQ(count_points(*sources.collision), 0U);
  }
  {
    auto validator = make_validator(default_params());
    HazardWarningSources sources;
    validator.validate(
      make_trajectory(now, pose), make_odom(now, pose), nullptr, vehicle_info,
      make_cloud(now, 15.0), sources);
    ASSERT_TRUE(sources.collision.has_value());
    EXPECT_FALSE(sources.collision->empty());
    EXPECT_EQ(count_points(*sources.collision), sources.collision->size());
  }
}

TEST(CollisionValidatorTest, PointcloudWithoutXyzIsSkippedAndObjectsAreStillEvaluated)
{
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto now = at(10.0);
  auto cloud = std::make_shared<PointCloud2>();
  cloud->header.stamp = now;
  cloud->header.frame_id = "base_link";
  sensor_msgs::PointCloud2Modifier modifier(*cloud);
  modifier.setPointCloud2Fields(1, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(1);

  auto validator = make_validator(default_params());
  HazardWarningSources sources;
  EXPECT_NO_THROW(validator.validate(
    make_trajectory(now, pose), make_odom(now, pose), make_objects(now, {make_car(15.0, 0.0)}),
    test_vehicle_info(), cloud, sources));
  ASSERT_TRUE(sources.collision.has_value());
  EXPECT_EQ(sources.collision->size(), 1U);
  EXPECT_EQ(count_points(*sources.collision), 0U);
}

TEST(CollisionValidatorTest, DisabledWarningLeavesTheCollisionSourcesUnset)
{
  auto params = default_params();
  params.collision_validator.enable_warning = false;
  auto validator = make_validator(params);
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto now = at(10.0);

  HazardWarningSources sources;
  validator.validate(
    make_trajectory(now, pose), make_odom(now, pose), make_objects(now, {make_car(15.0, 0.0)}),
    test_vehicle_info(), make_cloud(now, 15.0), sources);
  EXPECT_FALSE(sources.collision.has_value());
}

TEST(CollisionValidatorTest, PointcloudPointsAreHandedOverAsSingleVertices)
{
  auto validator = make_validator();
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto now = at(10.0);

  HazardWarningSources sources;
  validator.validate(
    make_trajectory(now, pose), make_odom(now, pose), make_objects(now, {}), test_vehicle_info(),
    make_cloud(now, 15.0), sources);
  ASSERT_TRUE(sources.collision.has_value());
  ASSERT_FALSE(sources.collision->empty());
  EXPECT_EQ(count_points(*sources.collision), sources.collision->size());
  for (const auto & source : *sources.collision) {
    EXPECT_EQ(source.risk_level, HazardSector::HIGH);
    EXPECT_TRUE(source.sources & HazardSector::SOURCE_COLLISION);
    EXPECT_GE(source.shape.outer().front().x(), 14.9);
    EXPECT_LE(source.shape.outer().front().x(), 15.5);
  }
}

TEST(CollisionValidatorTest, PointcloudWithoutCloseOdometryIsDropped)
{
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto now = at(10.0);
  const auto run = [&](const rclcpp::Time & cloud_stamp) {
    auto validator = make_validator();
    HazardWarningSources sources;
    validator.validate(
      make_trajectory(now, pose), make_odom(now, pose), make_objects(now, {}), test_vehicle_info(),
      make_cloud(cloud_stamp, 15.0), sources);
    return sources;
  };

  const auto close = run(at(9.95));
  ASSERT_TRUE(close.collision.has_value());
  EXPECT_FALSE(close.collision->empty());

  const auto far = run(at(9.8));
  ASSERT_TRUE(far.collision.has_value());
  EXPECT_TRUE(far.collision->empty());
}

TEST(CollisionValidatorTest, OdometryHistoryKeepsTheLatestDistinctStamps)
{
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto vehicle_info = test_vehicle_info();
  const auto feed = [&](CollisionValidator & validator, const std::vector<double> & stamps) {
    for (const double t : stamps) {
      HazardWarningSources ignored;
      validator.validate(
        make_trajectory(at(t), pose), make_odom(at(t), pose), make_objects(at(t), {}), vehicle_info,
        nullptr, ignored);
    }
  };
  const auto points_at = [&](CollisionValidator & validator, const double cloud_t) {
    HazardWarningSources sources;
    validator.validate(
      make_trajectory(at(20.0), pose), make_odom(at(20.0), pose), make_objects(at(20.0), {}),
      vehicle_info, make_cloud(at(cloud_t), 15.0), sources);
    return sources.collision ? count_points(*sources.collision) : 0U;
  };

  {
    auto validator = make_validator();
    feed(validator, {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0});
    EXPECT_EQ(points_at(validator, 0.0), 0U);
  }
  {
    auto validator = make_validator();
    feed(validator, {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0});
    EXPECT_GT(points_at(validator, 0.0), 0U);
  }
  {
    auto validator = make_validator();
    feed(validator, {0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    EXPECT_GT(points_at(validator, 0.0), 0U);
  }
}

TEST(CollisionValidatorTest, NearCandidatesAreHandedOverAsProximityWithoutALevel)
{
  auto validator = make_validator();
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto now = at(10.0);

  HazardWarningSources sources;
  validator.validate(
    make_trajectory(now, pose), make_odom(now, pose),
    make_objects(now, {make_car(1.0, 1.648), make_car(1.0, -1.948)}), test_vehicle_info(), nullptr,
    sources);
  ASSERT_TRUE(sources.collision.has_value());
  ASSERT_EQ(sources.collision->size(), 1U);
  EXPECT_EQ(sources.collision->front().risk_level, HazardSector::NONE);
  EXPECT_EQ(sources.collision->front().sources, HazardSector::SOURCE_PROXIMITY);
  for (const auto & vertex : sources.collision->front().shape.outer()) {
    EXPECT_GT(vertex.y(), 1.0);
  }
}
}  // namespace autoware::predicted_trajectory_validator
