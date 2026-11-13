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

#include "autoware/predicted_trajectory_validator/collision_validator/collision_evaluator.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/duration.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::predicted_trajectory_validator::collision_validator
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using detail::PoseTrajectory;
using detail::TimeTrajectory;
using detail::TrajectoryData;

namespace
{
// Same vehicle as autoware_test_utils/config/test_vehicle_info.param.yaml:
// front offset 3.79 m, rear overhang 1.1 m, width 1.896 m.
vehicle_info_utils::VehicleInfo test_vehicle_info()
{
  return vehicle_info_utils::createVehicleInfo(
    0.383, 0.235, 2.79, 1.64, 1.0, 1.1, 0.128, 0.128, 2.5, 0.70);
}

constexpr double kDt = 0.1;
constexpr double kEgoVelocity = 10.0;
constexpr double kFrontOffset = 3.79;

builtin_interfaces::msg::Time to_msg_time(const double sec)
{
  return rclcpp::Time(static_cast<int64_t>(sec * 1e9), RCL_ROS_TIME);
}

Trajectory make_straight_trajectory(
  const double stamp_sec, const double velocity, const double duration, const double y = 0.0)
{
  Trajectory trajectory;
  trajectory.header.stamp = to_msg_time(stamp_sec);
  trajectory.header.frame_id = "map";
  const auto n = static_cast<size_t>(std::round(duration / kDt)) + 1;
  for (size_t i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) * kDt;
    TrajectoryPoint point;
    point.pose.position.x = velocity * t;
    point.pose.position.y = y;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = static_cast<float>(velocity);
    point.time_from_start = rclcpp::Duration::from_seconds(t);
    trajectory.points.push_back(point);
  }
  return trajectory;
}

// Left turn of radius `radius` at constant speed, starting at the origin heading +x.
Trajectory make_arc_trajectory(
  const double stamp_sec, const double velocity, const double radius, const double duration)
{
  Trajectory trajectory;
  trajectory.header.stamp = to_msg_time(stamp_sec);
  trajectory.header.frame_id = "map";
  const auto n = static_cast<size_t>(std::round(duration / kDt)) + 1;
  for (size_t i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) * kDt;
    const double theta = velocity * t / radius;
    TrajectoryPoint point;
    point.pose.position.x = radius * std::sin(theta);
    point.pose.position.y = radius * (1.0 - std::cos(theta));
    point.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(theta);
    point.longitudinal_velocity_mps = static_cast<float>(velocity);
    point.time_from_start = rclcpp::Duration::from_seconds(t);
    trajectory.points.push_back(point);
  }
  return trajectory;
}

// Grid of points covering [x_min, x_max] x [y_min, y_max] at 0.25 m spacing, as one cluster.
PointCluster make_cluster(
  const double x_min, const double x_max, const double y_min, const double y_max)
{
  PointCluster cluster;
  double sx = 0.0;
  double sy = 0.0;
  for (double x = x_min; x <= x_max + 1e-9; x += 0.25) {
    for (double y = y_min; y <= y_max + 1e-9; y += 0.25) {
      cluster.points.emplace_back(x, y);
      sx += x;
      sy += y;
    }
  }
  const auto n = static_cast<double>(cluster.points.size());
  cluster.centroid = Point2d{sx / n, sy / n};
  return cluster;
}

PointCluster make_single_point_cluster(const double x, const double y)
{
  PointCluster cluster;
  cluster.points.emplace_back(x, y);
  cluster.centroid = Point2d{x, y};
  return cluster;
}

PredictedObject make_object(
  const double x, const double y, const double yaw, const double vx, const uint8_t label,
  const double size_x = 1.0, const double size_y = 1.0)
{
  PredictedObject object;
  ObjectClassification classification;
  classification.label = label;
  classification.probability = 1.0;
  object.classification.push_back(classification);
  object.kinematics.initial_pose_with_covariance.pose.position.x = x;
  object.kinematics.initial_pose_with_covariance.pose.position.y = y;
  object.kinematics.initial_pose_with_covariance.pose.orientation =
    autoware_utils_geometry::create_quaternion_from_yaw(yaw);
  object.kinematics.initial_twist_with_covariance.twist.linear.x = vx;
  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = size_x;
  object.shape.dimensions.y = size_y;
  object.shape.dimensions.z = 1.5;
  return object;
}

// Straight predicted path along the object heading at the object velocity (10 s).
void add_straight_path(PredictedObject & object, const double vx, const double yaw)
{
  PredictedPath path;
  path.confidence = 1.0;
  path.time_step = rclcpp::Duration::from_seconds(kDt);
  const auto & start = object.kinematics.initial_pose_with_covariance.pose;
  for (size_t i = 0; i < 100; ++i) {
    const double t = static_cast<double>(i) * kDt;
    geometry_msgs::msg::Pose pose = start;
    pose.position.x = start.position.x + vx * t * std::cos(yaw);
    pose.position.y = start.position.y + vx * t * std::sin(yaw);
    path.path.push_back(pose);
  }
  object.kinematics.predicted_paths.push_back(path);
}

void add_arc_path(
  PredictedObject & object, const double velocity, const double radius, const double yaw0)
{
  PredictedPath path;
  path.confidence = 1.0;
  path.time_step = rclcpp::Duration::from_seconds(kDt);
  const auto & start = object.kinematics.initial_pose_with_covariance.pose;
  const double cx = start.position.x - radius * std::sin(yaw0);
  const double cy = start.position.y + radius * std::cos(yaw0);
  for (size_t i = 0; i < 100; ++i) {
    const double theta = velocity * static_cast<double>(i) * kDt / radius;
    geometry_msgs::msg::Pose pose = start;
    pose.position.x = cx + radius * std::sin(yaw0 + theta);
    pose.position.y = cy - radius * std::cos(yaw0 + theta);
    pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw0 + theta);
    path.path.push_back(pose);
  }
  object.kinematics.predicted_paths.push_back(path);
}

CollisionEvaluatorParam default_param()
{
  CollisionEvaluatorParam param;
  param.max_time = 5.0;
  param.time_resolution = kDt;
  param.n_eval_object = 10;
  param.n_eval_pointcloud = 10;
  param.enabled_labels = {
    ObjectClassification::UNKNOWN, ObjectClassification::CAR, ObjectClassification::PEDESTRIAN};
  param.max_objects_stamp_gap_s = 0.5;
  param.stop_velocity = 0.1;
  param.proximity_distance = 0.3;
  param.reaction_times = {2.0, 1.0, 0.5};
  param.braking_deceleration = 6.0;
  param.cos_threshold = -1.0;
  param.rear_axle_ratio = 0.3;
  return param;
}

const builtin_interfaces::msg::Time kCloudStamp = to_msg_time(99.9);

std::vector<CollisionCandidate> evaluate(
  const CollisionEvaluatorParam & param, const Trajectory & predicted,
  const nav_msgs::msg::Odometry & odom, const PredictedObjects & objects,
  const std::vector<PointCluster> & clusters)
{
  const std::optional<PointCloudClusters> point_clusters =
    PointCloudClusters{clusters, rclcpp::Time(kCloudStamp)};
  return CollisionEvaluator(param).evaluate(
    predicted, odom, test_vehicle_info(), std::make_shared<const PredictedObjects>(objects),
    point_clusters);
}

PredictedObjects no_objects(const Trajectory & predicted)
{
  PredictedObjects objects;
  objects.header.stamp = predicted.header.stamp;
  return objects;
}

double expected_static_ttc(const double gap_from_front)
{
  return std::ceil(gap_from_front / kEgoVelocity / kDt - 1e-9) * kDt;
}

bool is_point(const CollisionCandidate & candidate)
{
  return candidate.polygon_now.outer().size() == 1;
}

std::vector<CollisionCandidate> points_of(const std::vector<CollisionCandidate> & candidates)
{
  std::vector<CollisionCandidate> out;
  std::copy_if(candidates.begin(), candidates.end(), std::back_inserter(out), is_point);
  return out;
}

std::vector<CollisionCandidate> objects_of(const std::vector<CollisionCandidate> & candidates)
{
  std::vector<CollisionCandidate> out;
  std::copy_if(candidates.begin(), candidates.end(), std::back_inserter(out), [](const auto & c) {
    return !is_point(c);
  });
  return out;
}

TrajectoryData ego_trajectory_data(const Trajectory & predicted)
{
  TimeTrajectory times;
  PoseTrajectory poses;
  for (const auto & point : predicted.points) {
    times.push_back(rclcpp::Duration(point.time_from_start).seconds());
    poses.push_back(point.pose);
  }
  return TrajectoryData(
    times, poses,
    detail::compute_footprint_trajectory(
      poses, detail::EgoDimensions{kFrontOffset, 1.1, 0.948, 0.948}));
}

std::optional<double> point_collision_time(const Trajectory & predicted, const Point2d & point)
{
  const auto ego = ego_trajectory_data(predicted);
  const auto k = detail::find_first_collision(ego, point);
  return k ? std::optional<double>{ego.getTimes()[*k]} : std::nullopt;
}

std::optional<double> object_collision_time(
  const Trajectory & predicted, const TrajectoryData & object_trajectory)
{
  const auto ego = ego_trajectory_data(predicted);
  const auto step = detail::find_first_collision(ego, object_trajectory);
  return step ? std::optional<double>{ego.getTimes()[step->ref_index]} : std::nullopt;
}

TrajectoryData path_trajectory(const PredictedObject & object, const double time_offset)
{
  return detail::generate_predicted_path_trajectory(
    object, object.kinematics.predicted_paths.front(), 5.0 + std::abs(time_offset), time_offset,
    kDt, std::nullopt);
}

std::optional<double> collision_cos_of(
  CollisionEvaluatorParam param, const Trajectory & predicted, const nav_msgs::msg::Odometry & odom,
  const PredictedObjects & objects)
{
  param.reaction_times = {10.0, 10.0, 10.0};
  const auto raised = [&](const double cos_threshold) {
    param.cos_threshold = cos_threshold;
    return !objects_of(evaluate(param, predicted, odom, objects, {})).empty();
  };
  if (!raised(-1.0)) {
    return std::nullopt;
  }
  double passing = -1.0;
  double blocked = 1.0 + 1e-9;
  for (int i = 0; i < 40; ++i) {
    const double mid = 0.5 * (passing + blocked);
    (raised(mid) ? passing : blocked) = mid;
  }
  return passing;
}

Trajectory make_starting_trajectory()
{
  Trajectory predicted;
  predicted.header.stamp = to_msg_time(100.0);
  predicted.header.frame_id = "map";
  for (size_t i = 0; i <= 50; ++i) {
    const double t = static_cast<double>(i) * kDt;
    const bool moving = t > 0.5 + 1e-9;
    TrajectoryPoint point;
    point.pose.position.x = moving ? kEgoVelocity * (t - 0.5) : 0.0;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = static_cast<float>(moving ? kEgoVelocity : 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(t);
    predicted.points.push_back(point);
  }
  return predicted;
}
}  // namespace

TEST(CollisionEvaluatorTest, StaticPointsAheadHaveTtcOfDistanceOverVelocity)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = kEgoVelocity;

  const std::vector<PointCluster> clusters = {make_cluster(20.0, 21.0, -0.5, 0.5)};
  const auto candidates =
    evaluate(default_param(), predicted, odom, no_objects(predicted), clusters);

  ASSERT_FALSE(candidates.empty());
  EXPECT_EQ(points_of(candidates).size(), candidates.size());
  double first_collision_time = std::numeric_limits<double>::infinity();
  for (const auto & point : clusters.front().points) {
    if (const auto t = point_collision_time(predicted, point)) {
      first_collision_time = std::min(first_collision_time, *t);
    }
  }
  EXPECT_NEAR(first_collision_time, expected_static_ttc(20.0 - kFrontOffset), 1e-6);
}

TEST(CollisionEvaluatorTest, LateralPointsHaveNoCandidate)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;

  const std::vector<PointCluster> clusters = {make_cluster(20.0, 21.0, 3.0, 4.0)};
  const auto candidates =
    evaluate(default_param(), predicted, odom, no_objects(predicted), clusters);

  EXPECT_TRUE(candidates.empty());
}

TEST(CollisionEvaluatorTest, EgoFootprintFollowsTheLeftAndRightOverhangs)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = kEgoVelocity;
  const auto vehicle_info = vehicle_info_utils::createVehicleInfo(
    0.383, 0.235, 2.79, 1.64, 1.0, 1.1, 1.0, 0.128, 2.5, 0.70);
  const auto evaluate_cluster = [&](const PointCluster & cluster) {
    const std::optional<PointCloudClusters> point_clusters =
      PointCloudClusters{{cluster}, rclcpp::Time(kCloudStamp)};
    return CollisionEvaluator(default_param())
      .evaluate(
        predicted, odom, vehicle_info,
        std::make_shared<const PredictedObjects>(no_objects(predicted)), point_clusters);
  };

  EXPECT_FALSE(evaluate_cluster(make_cluster(20.0, 21.0, 1.5, 1.75)).empty());
  EXPECT_TRUE(evaluate_cluster(make_cluster(20.0, 21.0, -1.75, -1.5)).empty());
}

TEST(CollisionEvaluatorTest, PointsBeyondHorizonHaveNoCandidate)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;

  // 5 s * 10 m/s + 3.79 m = 53.79 m is the farthest point swept.
  const std::vector<PointCluster> clusters = {make_cluster(60.0, 61.0, -0.5, 0.5)};
  const auto candidates =
    evaluate(default_param(), predicted, odom, no_objects(predicted), clusters);

  EXPECT_TRUE(candidates.empty());
}

TEST(CollisionEvaluatorTest, PointOnlyInsideTheWindowHullCollides)
{
  const double velocity = 60.0;
  const auto predicted = make_straight_trajectory(100.0, velocity, 2.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = velocity;

  const double x = 16.3;
  for (size_t k = 0; k <= 20; ++k) {
    const double base = velocity * static_cast<double>(k) * kDt;
    EXPECT_FALSE(x >= base - 1.1 && x <= base + kFrontOffset) << "k = " << k;
  }
  const auto candidates = evaluate(
    default_param(), predicted, odom, no_objects(predicted), {make_single_point_cluster(x, 0.0)});

  ASSERT_EQ(candidates.size(), 1U);
  const auto t = point_collision_time(predicted, Point2d{x, 0.0});
  ASSERT_TRUE(t.has_value());
  EXPECT_NEAR(*t, 0.3, 1e-6);
}

TEST(CollisionEvaluatorTest, FrontPointHasCosNearOne)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  const Point2d point{20.0, 0.0};

  const auto t = point_collision_time(predicted, point);
  ASSERT_TRUE(t.has_value());
  const auto k = static_cast<size_t>(std::round(*t / kDt));
  EXPECT_NEAR(compute_cos_collision(predicted.points[k].pose, true, point), 1.0, 1e-3);
}

TEST(CollisionEvaluatorTest, PointAheadOnTheCurveHasCosNearOneAtCollisionTime)
{
  // Left turn of radius 20 m: after 60 deg (t = 2.09 s) the ego is at (17.3, 10.0) heading 60
  // deg. A point slightly ahead on the arc is currently at bearing ~31 deg (cos 0.86) but is
  // straight ahead at the collision time.
  const auto predicted = make_arc_trajectory(100.0, kEgoVelocity, 20.0, 5.0);
  const double theta = 60.0 * M_PI / 180.0 + 2.0 / 20.0;  // 2 m further along the arc
  const Point2d point{20.0 * std::sin(theta), 20.0 * (1.0 - std::cos(theta))};

  const auto t = point_collision_time(predicted, point);
  ASSERT_TRUE(t.has_value());
  const auto k = static_cast<size_t>(std::round(*t / kDt));
  EXPECT_GT(compute_cos_collision(predicted.points[k].pose, true, point), 0.97);
  const double bearing_now = std::atan2(point.y(), point.x());
  EXPECT_LT(std::cos(bearing_now), 0.9);
}

TEST(CollisionEvaluatorTest, ObjectGrazingTheSideHasCosNearZero)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = kEgoVelocity;

  // Object coming from the left (y = 5) straight down at 5 m/s. Its lower edge reaches the ego
  // side (y = 0.948) at t = 0.71 s, so the first colliding window ends at t_k = 0.8 s, when the
  // ego base_link is at x = 8: the object is placed there so that the hit is beside the axle.
  PredictedObjects objects;
  objects.header.stamp = predicted.header.stamp;
  auto object = make_object(8.0, 5.0, -M_PI / 2.0, 5.0, ObjectClassification::PEDESTRIAN);
  add_straight_path(object, 5.0, -M_PI / 2.0);
  objects.objects.push_back(object);

  const auto cos_collision = collision_cos_of(default_param(), predicted, odom, objects);
  ASSERT_TRUE(cos_collision.has_value());
  EXPECT_LT(std::abs(*cos_collision), 0.35);
}

TEST(CollisionEvaluatorTest, CrossingObjectTtcMatchesAnalyticCrossingTime)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = kEgoVelocity;

  // Object at (25, -10) moving +y at 5 m/s. Ego x-range covers [24.5, 25.5] for t in
  // [2.07, 2.66]; the object y-range covers [-0.948, 0.948] for t in [1.71, 2.29].
  // The first overlap is at t = 2.07 -> first window [2.0, 2.1] -> ttc = 2.1.
  const double object_vy = 5.0;
  PredictedObjects objects;
  objects.header.stamp = predicted.header.stamp;
  auto object = make_object(25.0, -10.0, M_PI / 2.0, object_vy, ObjectClassification::CAR);
  add_straight_path(object, object_vy, M_PI / 2.0);
  objects.objects.push_back(object);

  const auto candidates = evaluate(default_param(), predicted, odom, objects, {});

  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_FALSE(is_point(candidates.front()));
  const auto t = object_collision_time(predicted, path_trajectory(object, 0.0));
  ASSERT_TRUE(t.has_value());
  EXPECT_NEAR(*t, 2.1, 1e-6);
  const auto cos_collision = collision_cos_of(default_param(), predicted, odom, objects);
  ASSERT_TRUE(cos_collision.has_value());
  EXPECT_GT(*cos_collision, 0.9);  // hits the front of the ego
}

TEST(CollisionEvaluatorTest, OppositeLaneObjectHasNoTtc)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;

  // Oncoming object in the adjacent lane (y = 3.5), approaching but never crossing.
  PredictedObjects objects;
  objects.header.stamp = predicted.header.stamp;
  auto object = make_object(40.0, 3.5, M_PI, 10.0, ObjectClassification::CAR, 4.0, 1.8);
  add_straight_path(object, 10.0, M_PI);
  objects.objects.push_back(object);

  const auto candidates = evaluate(default_param(), predicted, odom, objects, {});

  EXPECT_TRUE(candidates.empty());
  EXPECT_FALSE(object_collision_time(predicted, path_trajectory(object, 0.0)).has_value());
}

TEST(CollisionEvaluatorTest, ObjectStampOffsetIsCompensated)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;
  const double object_vy = 5.0;

  for (const double offset : {-0.2, 0.0, 0.2}) {
    // The object observed `offset` seconds after the trajectory stamp has already moved by
    // offset * vy, so the physical crossing time stays the same.
    PredictedObjects objects;
    objects.header.stamp = to_msg_time(100.0 + offset);
    auto object = make_object(
      25.0, -10.0 + object_vy * offset, M_PI / 2.0, object_vy, ObjectClassification::CAR);
    add_straight_path(object, object_vy, M_PI / 2.0);
    objects.objects.push_back(object);

    const auto candidates = evaluate(default_param(), predicted, odom, objects, {});
    ASSERT_EQ(candidates.size(), 1U) << "offset " << offset;
    const auto t = object_collision_time(predicted, path_trajectory(object, offset));
    ASSERT_TRUE(t.has_value()) << "offset " << offset;
    EXPECT_NEAR(*t, 2.1, 1e-6) << "offset " << offset;
  }
}

TEST(CollisionEvaluatorTest, ObjectsOlderThanStampGapAreNotEvaluated)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;

  PredictedObjects objects;
  objects.header.stamp = to_msg_time(100.0 - 0.6);
  objects.objects.push_back(make_object(20.0, 0.0, 0.0, 0.0, ObjectClassification::CAR));

  const auto candidates = evaluate(default_param(), predicted, odom, objects, {});
  EXPECT_TRUE(candidates.empty());
}

TEST(CollisionEvaluatorTest, StationaryObjectWithoutPathIsEvaluatedAsStatic)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;

  PredictedObjects objects;
  objects.header.stamp = predicted.header.stamp;
  objects.objects.push_back(make_object(20.5, 0.0, 0.0, 0.0, ObjectClassification::CAR));

  const auto candidates = evaluate(default_param(), predicted, odom, objects, {});

  ASSERT_EQ(candidates.size(), 1U);
  const auto & object = objects.objects.front();
  const auto static_trajectory = detail::generate_static_trajectory(
    detail::to_polygon2d(object.kinematics.initial_pose_with_covariance.pose, object.shape),
    ego_trajectory_data(predicted).getTimes());
  const auto t = object_collision_time(predicted, static_trajectory);
  ASSERT_TRUE(t.has_value());
  EXPECT_NEAR(*t, expected_static_ttc(20.0 - kFrontOffset), 1e-6);
}

TEST(CollisionEvaluatorTest, ObjectTouchingTheCurrentFootprintIsExcluded)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;

  PredictedObjects objects;
  objects.header.stamp = predicted.header.stamp;
  // Overlaps the ego footprint (x in [-1.1, 3.79], |y| <= 0.948) at the current time.
  objects.objects.push_back(make_object(2.0, 1.2, 0.0, 0.0, ObjectClassification::CAR));
  objects.objects.push_back(make_object(20.5, 0.0, 0.0, 0.0, ObjectClassification::CAR));

  const auto candidates = evaluate(default_param(), predicted, odom, objects, {});

  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_NEAR(candidates.front().distance_now, 20.0 - kFrontOffset, 1e-6);
}

TEST(CollisionEvaluatorTest, DisabledLabelsAndEvaluationLimitAreApplied)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;

  PredictedObjects objects;
  objects.header.stamp = predicted.header.stamp;
  objects.objects.push_back(make_object(30.5, 0.0, 0.0, 0.0, ObjectClassification::CAR));
  objects.objects.push_back(make_object(20.5, 0.0, 0.0, 0.0, ObjectClassification::CAR));
  objects.objects.push_back(make_object(10.5, 0.0, 0.0, 0.0, ObjectClassification::BUS));
  objects.objects.push_back(make_object(40.5, 0.0, 0.0, 0.0, ObjectClassification::CAR));

  auto param = default_param();
  param.enabled_labels = {ObjectClassification::CAR};
  param.n_eval_object = 2;

  const auto candidates = evaluate(param, predicted, odom, objects, {});

  // BUS is disabled, and only the two closest CARs remain.
  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_NEAR(candidates[0].distance_now, 20.0 - kFrontOffset, 1e-6);
  EXPECT_NEAR(candidates[1].distance_now, 30.0 - kFrontOffset, 1e-6);
}

TEST(CollisionEvaluatorTest, ClosestClustersAreKeptUpToLimit)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;

  const std::vector<PointCluster> clusters = {
    make_single_point_cluster(30.0, 0.0), make_single_point_cluster(10.0, 0.0),
    make_single_point_cluster(20.0, 0.0)};
  auto param = default_param();
  param.n_eval_pointcloud = 2;

  const auto candidates = evaluate(param, predicted, odom, no_objects(predicted), clusters);
  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_NEAR(candidates[0].polygon_now.outer().front().x(), 10.0, 1e-9);
  EXPECT_NEAR(candidates[1].polygon_now.outer().front().x(), 20.0, 1e-9);
}

TEST(CollisionEvaluatorTest, HorizonIsClippedToTrajectoryEnd)
{
  // 2 s trajectory: the sweep ends at 23.79 m even though max_time is 5 s.
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 2.0);
  nav_msgs::msg::Odometry odom;

  const std::vector<PointCluster> clusters = {
    make_single_point_cluster(20.0, 0.0), make_single_point_cluster(30.0, 0.0)};
  const auto candidates =
    evaluate(default_param(), predicted, odom, no_objects(predicted), clusters);

  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_NEAR(candidates.front().polygon_now.outer().front().x(), 20.0, 1e-9);
}

TEST(CollisionEvaluatorTest, StoppedEgoTrajectoryGivesNoTtc)
{
  const auto predicted = make_straight_trajectory(100.0, 0.0, 5.0);
  nav_msgs::msg::Odometry odom;

  PredictedObjects objects;
  objects.header.stamp = predicted.header.stamp;
  auto follower = make_object(-15.0, 0.0, 0.0, 5.0, ObjectClassification::CAR, 4.0, 1.8);
  add_straight_path(follower, 5.0, 0.0);
  objects.objects.push_back(follower);
  const std::vector<PointCluster> clusters = {make_single_point_cluster(3.5, 0.0)};

  const auto candidates = evaluate(default_param(), predicted, odom, objects, clusters);
  EXPECT_TRUE(candidates.empty());

  auto no_stop = default_param();
  no_stop.stop_velocity = 0.0;
  const auto unmasked = evaluate(no_stop, predicted, odom, objects, clusters);
  ASSERT_EQ(unmasked.size(), 1U);
  EXPECT_TRUE(is_point(unmasked.front()));
  EXPECT_GE(unmasked.front().brake_level, 1);
}

TEST(CollisionEvaluatorTest, CollisionStartsWhenTheEgoTrajectoryStartsMoving)
{
  const auto predicted = make_starting_trajectory();
  nav_msgs::msg::Odometry odom;

  const std::vector<PointCluster> clusters = {make_single_point_cluster(kFrontOffset + 1.2, 0.0)};
  const auto candidates =
    evaluate(default_param(), predicted, odom, no_objects(predicted), clusters);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_FALSE(candidates.front().near);
  EXPECT_GE(candidates.front().brake_level, 1);
}

TEST(CollisionEvaluatorTest, ReversingEgoTrajectoryIsNotTreatedAsStopped)
{
  const auto predicted = make_straight_trajectory(100.0, -kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = -kEgoVelocity;

  const auto candidates = evaluate(
    default_param(), predicted, odom, no_objects(predicted),
    {make_single_point_cluster(-20.0, 0.0)});
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_GE(candidates.front().brake_level, 1);
  const auto t = point_collision_time(predicted, Point2d{-20.0, 0.0});
  ASSERT_TRUE(t.has_value());
  EXPECT_NEAR(*t, 1.9, 1e-6);
}

TEST(CollisionEvaluatorTest, VehicleYawKeepsTheDirectionOfAnAlignedStraightPath)
{
  PoseTrajectory poses(100);
  for (size_t k = 0; k < poses.size(); ++k) {
    poses[k].position.x = 0.1 * static_cast<double>(k);
    poses[k].orientation.w = 1.0;
  }
  detail::correct_vehicle_yaw(poses, 0.0, 1.35);
  for (size_t k = 0; k < poses.size(); ++k) {
    EXPECT_NEAR(poses[k].position.x, 0.1 * static_cast<double>(k), 1e-12);
    EXPECT_NEAR(autoware_utils_geometry::get_rpy(poses[k]).z, 0.0, 1e-9);
  }
}

TEST(CollisionEvaluatorTest, VehicleYawOffsetDecaysAlongAStraightPath)
{
  const double rear_axle_length = 1.35;
  const double initial_yaw = M_PI / 6.0;
  PoseTrajectory poses(100);
  for (size_t k = 0; k < poses.size(); ++k) {
    poses[k].position.x = 0.1 * static_cast<double>(k);
    poses[k].orientation.w = 1.0;
  }
  detail::correct_vehicle_yaw(poses, initial_yaw, rear_axle_length);
  for (size_t k = 0; k < poses.size(); ++k) {
    const double s = 0.1 * static_cast<double>(k);
    const double expected =
      2.0 * std::atan(std::tan(initial_yaw / 2.0) * std::exp(-s / rear_axle_length));
    EXPECT_NEAR(poses[k].position.x, s, 1e-12);
    EXPECT_NEAR(poses[k].position.y, 0.0, 1e-12);
    EXPECT_NEAR(autoware_utils_geometry::get_rpy(poses[k]).z, expected, 0.01) << "s = " << s;
  }
}

TEST(CollisionEvaluatorTest, VehicleYawLagsTheTangentOnACurve)
{
  const double radius = 10.0;
  PoseTrajectory poses(200);
  for (size_t k = 0; k < poses.size(); ++k) {
    const double theta = 0.1 * static_cast<double>(k) / radius;
    poses[k].position.x = radius * std::sin(theta);
    poses[k].position.y = radius * (1.0 - std::cos(theta));
    poses[k].orientation = autoware_utils_geometry::create_quaternion_from_yaw(theta);
  }
  const auto original = poses;
  detail::correct_vehicle_yaw(poses, 0.0, 1.35);
  for (size_t k = 1; k < poses.size(); ++k) {
    const double tangent = 0.1 * static_cast<double>(k) / radius;
    const double lag = tangent - autoware_utils_geometry::get_rpy(poses[k]).z;
    EXPECT_GT(lag, 0.0) << "k = " << k;
    EXPECT_LT(lag, 0.2) << "k = " << k;
    EXPECT_NEAR(poses[k].position.x, original[k].position.x, 1e-12);
    EXPECT_NEAR(poses[k].position.y, original[k].position.y, 1e-12);
  }
}

TEST(CollisionEvaluatorTest, PredictedPathWithoutRearAxleKeepsThePathYaw)
{
  auto object = make_object(30.0, -15.0, M_PI / 2.0, 8.0, ObjectClassification::CAR, 4.0, 1.8);
  add_arc_path(object, 8.0, 20.0, M_PI / 2.0);
  const auto & path = object.kinematics.predicted_paths.front();

  const auto max_yaw_difference = [&](const std::optional<double> rear_axle_length) {
    const auto trajectory =
      detail::generate_predicted_path_trajectory(object, path, 5.0, 0.0, kDt, rear_axle_length);
    double max_difference = 0.0;
    for (const auto & pose : trajectory.getPoses()) {
      const double s = std::atan2(pose.position.y + 15.0, pose.position.x - 10.0);
      const double path_yaw = s + M_PI / 2.0;
      const double d = autoware_utils_geometry::get_rpy(pose).z - path_yaw;
      max_difference = std::max(max_difference, std::abs(std::atan2(std::sin(d), std::cos(d))));
    }
    return max_difference;
  };
  EXPECT_LT(max_yaw_difference(std::nullopt), 1e-4);
  EXPECT_GT(max_yaw_difference(1.2), 1e-3);
}

TEST(CollisionEvaluatorTest, VehicleYawIsAppliedOnlyToForwardVehicleBoxes)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = kEgoVelocity;
  auto param = default_param();
  param.enabled_labels = {
    ObjectClassification::CAR, ObjectClassification::BUS, ObjectClassification::PEDESTRIAN,
    ObjectClassification::BICYCLE, ObjectClassification::TRAILER};

  const auto collision_cos = [&](const PredictedObject & object) {
    PredictedObjects objects;
    objects.header.stamp = predicted.header.stamp;
    objects.objects.push_back(object);
    const auto cos_collision = collision_cos_of(param, predicted, odom, objects);
    EXPECT_TRUE(cos_collision.has_value());
    return cos_collision;
  };
  const auto same_collision = [&](const PredictedObject & a, const PredictedObject & b) {
    const auto cos_a = collision_cos(a);
    const auto cos_b = collision_cos(b);
    return cos_a.has_value() && cos_b.has_value() && std::abs(*cos_a - *cos_b) < 1e-6;
  };
  const auto turning = [](const uint8_t label) {
    auto object = make_object(30.0, -15.0, M_PI / 2.0, 8.0, label, 4.0, 1.8);
    add_arc_path(object, 8.0, 20.0, M_PI / 2.0);
    return object;
  };
  const auto relabeled = [](PredictedObject object, const uint8_t label) {
    object.classification.front().label = label;
    return object;
  };
  const auto reference = turning(ObjectClassification::PEDESTRIAN);

  EXPECT_FALSE(same_collision(turning(ObjectClassification::CAR), reference));
  EXPECT_FALSE(same_collision(turning(ObjectClassification::BUS), reference));
  for (const auto label : {ObjectClassification::BICYCLE, ObjectClassification::TRAILER}) {
    EXPECT_TRUE(same_collision(turning(label), reference)) << "label " << static_cast<int>(label);
  }

  auto reversing = turning(ObjectClassification::CAR);
  reversing.kinematics.initial_pose_with_covariance.pose.orientation =
    autoware_utils_geometry::create_quaternion_from_yaw(-M_PI / 2.0);
  EXPECT_TRUE(same_collision(reversing, relabeled(reversing, ObjectClassification::PEDESTRIAN)));

  auto cylinder = turning(ObjectClassification::CAR);
  cylinder.shape.type = Shape::CYLINDER;
  EXPECT_TRUE(same_collision(cylinder, relabeled(cylinder, ObjectClassification::PEDESTRIAN)));
}

TEST(CollisionEvaluatorTest, BrakeLevelOfAStaticPointFollowsTheStoppingDistance)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = kEgoVelocity;

  for (const auto & [gap, expected_level] :
       {std::pair{12.0, 3}, std::pair{16.0, 2}, std::pair{25.0, 1}}) {
    const auto candidates = evaluate(
      default_param(), predicted, odom, no_objects(predicted),
      {make_single_point_cluster(kFrontOffset + gap, 0.0)});
    ASSERT_EQ(candidates.size(), 1U) << "gap " << gap;
    EXPECT_EQ(candidates.front().brake_level, expected_level) << "gap " << gap;
  }

  const Point2d avoidable{kFrontOffset + 35.0, 0.0};
  EXPECT_TRUE(point_collision_time(predicted, avoidable).has_value());
  EXPECT_TRUE(evaluate(
                default_param(), predicted, odom, no_objects(predicted),
                {make_single_point_cluster(avoidable.x(), avoidable.y())})
                .empty());
}

TEST(CollisionEvaluatorTest, CrossingObjectAvoidedByEarlierBrakingHasLevelOne)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = kEgoVelocity;

  PredictedObjects objects;
  objects.header.stamp = predicted.header.stamp;
  auto object = make_object(25.0, -10.0, M_PI / 2.0, 5.0, ObjectClassification::CAR);
  add_straight_path(object, 5.0, M_PI / 2.0);
  objects.objects.push_back(object);

  const auto candidates = evaluate(default_param(), predicted, odom, objects, {});
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().brake_level, 1);
}

TEST(CollisionEvaluatorTest, FastOncomingObjectInTheEgoLaneHitsBeforeTheEgoStops)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = kEgoVelocity;

  PredictedObjects objects;
  objects.header.stamp = predicted.header.stamp;
  auto object = make_object(40.0, 0.0, M_PI, 10.0, ObjectClassification::CAR, 4.0, 1.8);
  add_straight_path(object, 10.0, M_PI);
  objects.objects.push_back(object);

  const auto candidates = evaluate(default_param(), predicted, odom, objects, {});
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().brake_level, 3);
}

TEST(CollisionEvaluatorTest, SlowOncomingObjectHittingTheStoppedEgoIsNotCounted)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = kEgoVelocity;
  auto param = default_param();
  param.reaction_times = {0.5, 0.3, 0.1};

  PredictedObjects objects;
  objects.header.stamp = predicted.header.stamp;
  auto object = make_object(34.0, 0.0, M_PI, 6.0, ObjectClassification::CAR, 4.0, 1.8);
  add_straight_path(object, 6.0, M_PI);
  objects.objects.push_back(object);

  EXPECT_TRUE(object_collision_time(predicted, path_trajectory(object, 0.0)).has_value());
  EXPECT_TRUE(evaluate(param, predicted, odom, objects, {}).empty());
}

TEST(CollisionEvaluatorTest, OncomingObjectReachingTheCurrentEgoAfterTheHorizonIsNotCounted)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 3.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = kEgoVelocity;
  auto param = default_param();
  param.reaction_times = {0.5, 0.3, 0.1};

  const auto oncoming = [&](const size_t path_points) {
    auto object = make_object(34.12, 0.0, M_PI, 6.0, ObjectClassification::CAR, 4.0, 1.8);
    PredictedPath path;
    path.confidence = 1.0;
    path.time_step = rclcpp::Duration::from_seconds(kDt);
    for (size_t i = 0; i < path_points; ++i) {
      auto pose = object.kinematics.initial_pose_with_covariance.pose;
      pose.position.x -= 6.0 * static_cast<double>(i) * kDt;
      path.path.push_back(pose);
    }
    object.kinematics.predicted_paths.push_back(path);
    PredictedObjects objects;
    objects.header.stamp = predicted.header.stamp;
    objects.objects.push_back(object);
    return objects;
  };

  EXPECT_TRUE(evaluate(param, predicted, odom, oncoming(100), {}).empty());
  const auto short_path = evaluate(param, predicted, odom, oncoming(40), {});
  ASSERT_EQ(short_path.size(), 1U);
  EXPECT_GE(short_path.front().brake_level, 1);
}

TEST(CollisionEvaluatorTest, ObjectReachingTheStoppedEgoAwayFromItsCurrentPositionIsCounted)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = kEgoVelocity;
  auto param = default_param();
  param.reaction_times = {0.5, 0.3, 0.1};

  PredictedObjects objects;
  objects.header.stamp = predicted.header.stamp;
  auto object = make_object(34.0, 0.0, M_PI, 6.0, ObjectClassification::CAR, 4.0, 1.8);
  PredictedPath path;
  path.confidence = 1.0;
  path.time_step = rclcpp::Duration::from_seconds(kDt);
  for (size_t i = 0; i < 30; ++i) {
    auto pose = object.kinematics.initial_pose_with_covariance.pose;
    pose.position.x -= 6.0 * static_cast<double>(i) * kDt;
    path.path.push_back(pose);
  }
  object.kinematics.predicted_paths.push_back(path);
  objects.objects.push_back(object);

  const auto candidates = evaluate(param, predicted, odom, objects, {});
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().brake_level, 2);
}

TEST(CollisionEvaluatorTest, NullObjectsEvaluatesOnlyThePoints)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;

  const std::optional<PointCloudClusters> point_clusters =
    PointCloudClusters{{make_single_point_cluster(20.0, 0.0)}, rclcpp::Time(kCloudStamp)};
  const auto candidates =
    CollisionEvaluator(default_param())
      .evaluate(predicted, odom, test_vehicle_info(), nullptr, point_clusters);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_TRUE(is_point(candidates.front()));
}

TEST(CollisionEvaluatorTest, NullPointClustersEvaluatesOnlyTheObjects)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;

  PredictedObjects objects;
  objects.header.stamp = predicted.header.stamp;
  objects.objects.push_back(make_object(20.5, 0.0, 0.0, 0.0, ObjectClassification::CAR));
  const auto candidates = CollisionEvaluator(default_param())
                            .evaluate(
                              predicted, odom, test_vehicle_info(),
                              std::make_shared<const PredictedObjects>(objects), std::nullopt);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_FALSE(is_point(candidates.front()));
}

TEST(CollisionEvaluatorTest, ReversingEgoBrakesAlongThePathWithItsSpeed)
{
  const auto predicted = make_straight_trajectory(100.0, -kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = -kEgoVelocity;

  const auto candidates = evaluate(
    default_param(), predicted, odom, no_objects(predicted),
    {make_single_point_cluster(-17.0, 0.0)});
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().brake_level, 2);
}

TEST(CollisionEvaluatorTest, ObjectsBelowTheCosThresholdHaveNoBrakeLevel)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = kEgoVelocity;

  PredictedObjects objects;
  objects.header.stamp = predicted.header.stamp;
  auto object = make_object(25.0, -10.0, M_PI / 2.0, 5.0, ObjectClassification::CAR);
  add_straight_path(object, 5.0, M_PI / 2.0);
  objects.objects.push_back(object);

  const auto run = [&](const double cos_threshold) {
    auto param = default_param();
    param.cos_threshold = cos_threshold;
    return evaluate(param, predicted, odom, objects, {});
  };
  const auto passing = run(-1.0);
  ASSERT_EQ(passing.size(), 1U);
  EXPECT_GE(passing.front().brake_level, 1);
  const auto cos_collision = collision_cos_of(default_param(), predicted, odom, objects);
  ASSERT_TRUE(cos_collision.has_value());
  EXPECT_LT(*cos_collision, 1.0);
  EXPECT_TRUE(run(1.0).empty());
}

TEST(CollisionEvaluatorTest, StaticPointsGetABrakeLevelBelowTheCosThreshold)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = kEgoVelocity;
  const std::vector<PointCluster> clusters = {make_single_point_cluster(20.0, 0.5)};

  auto param = default_param();
  param.cos_threshold = 1.0;
  const auto candidates = evaluate(param, predicted, odom, no_objects(predicted), clusters);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().brake_level, 2);
  const auto & point = clusters.front().points.front();
  const auto t = point_collision_time(predicted, point);
  ASSERT_TRUE(t.has_value());
  const auto k = static_cast<size_t>(std::round(*t / kDt));
  EXPECT_LT(compute_cos_collision(predicted.points[k].pose, true, point), 1.0);
}

TEST(CollisionEvaluatorTest, ProximityIsRaisedOnlyWithinTheDistance)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = kEgoVelocity;

  PredictedObjects objects;
  objects.header.stamp = predicted.header.stamp;
  objects.objects.push_back(make_object(1.0, 1.648, 0.0, 0.0, ObjectClassification::CAR));
  objects.objects.push_back(make_object(1.0, -1.948, 0.0, 0.0, ObjectClassification::CAR));
  PointCluster cluster;
  cluster.points = {{1.0, 1.148}, {1.0, -1.448}};
  cluster.centroid = Point2d{1.0, -0.15};

  const auto candidates = evaluate(default_param(), predicted, odom, objects, {cluster});

  const auto object_candidates = objects_of(candidates);
  ASSERT_EQ(object_candidates.size(), 1U);
  EXPECT_NEAR(object_candidates.front().distance_now, 0.2, 1e-6);
  EXPECT_TRUE(object_candidates.front().near);
  const auto point_candidates = points_of(candidates);
  ASSERT_EQ(point_candidates.size(), 1U);
  EXPECT_NEAR(point_candidates.front().polygon_now.outer().front().y(), 1.148, 1e-9);
  EXPECT_TRUE(point_candidates.front().near);
  for (const auto & candidate : candidates) {
    EXPECT_EQ(candidate.brake_level, 0);
  }
}

TEST(CollisionEvaluatorTest, ProximityIsNotRaisedWhileTheWholeTrajectoryIsStopped)
{
  const auto predicted = make_straight_trajectory(100.0, 0.0, 5.0);
  nav_msgs::msg::Odometry odom;

  PredictedObjects objects;
  objects.header.stamp = predicted.header.stamp;
  objects.objects.push_back(make_object(1.0, 1.648, 0.0, 0.0, ObjectClassification::CAR));

  const auto candidates =
    evaluate(default_param(), predicted, odom, objects, {make_single_point_cluster(1.0, 1.148)});

  EXPECT_TRUE(candidates.empty());
}

TEST(CollisionEvaluatorTest, ProximityIsRaisedWhenTheStoppedEgoStartsMoving)
{
  const auto predicted = make_starting_trajectory();
  nav_msgs::msg::Odometry odom;

  PredictedObjects objects;
  objects.header.stamp = predicted.header.stamp;
  objects.objects.push_back(make_object(1.0, 1.648, 0.0, 0.0, ObjectClassification::CAR));

  const auto candidates =
    evaluate(default_param(), predicted, odom, objects, {make_single_point_cluster(1.0, 1.148)});

  ASSERT_EQ(candidates.size(), 2U);
  for (const auto & candidate : candidates) {
    EXPECT_TRUE(candidate.near);
  }
}

TEST(CollisionEvaluatorTest, EveryPointWithinTheDistanceIsOneCandidate)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = kEgoVelocity;

  PointCluster cluster;
  cluster.points = {{0.0, 1.048}, {1.0, 1.048}, {2.0, 1.048}, {kFrontOffset + 0.2, 0.0}};
  cluster.centroid = Point2d{1.5, 0.8};

  const auto candidates =
    evaluate(default_param(), predicted, odom, no_objects(predicted), {cluster});

  ASSERT_EQ(candidates.size(), 4U);
  size_t colliding = 0;
  for (const auto & candidate : candidates) {
    EXPECT_TRUE(candidate.near);
    if (candidate.brake_level >= 1) {
      ++colliding;
      EXPECT_NEAR(candidate.polygon_now.outer().front().x(), kFrontOffset + 0.2, 1e-9);
      EXPECT_EQ(candidate.brake_level, 3);
    }
  }
  EXPECT_EQ(colliding, 1U);
}

TEST(CollisionEvaluatorTest, ReversingEgoFlipsTheHeadingForCos)
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  const Point2d behind{-5.0, 0.0};
  EXPECT_NEAR(compute_cos_collision(pose, true, behind), -1.0, 1e-9);
  EXPECT_NEAR(compute_cos_collision(pose, false, behind), 1.0, 1e-9);
  EXPECT_NEAR(compute_cos_collision(pose, true, Point2d{0.0, 0.0}), 1.0, 1e-9);
}
}  // namespace autoware::predicted_trajectory_validator::collision_validator
