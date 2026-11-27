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

#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

#include "autoware/predicted_trajectory_validator/collision_validator/collision_evaluator.hpp"
#include "autoware/predicted_trajectory_validator/collision_validator/collision_validator.hpp"
#include "autoware/predicted_trajectory_validator/collision_validator/pointcloud_preprocessor.hpp"
#include "autoware/predicted_trajectory_validator/hazard_warning/hazard_warning.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::predicted_trajectory_validator::test_utils
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_geometry::Point2d;
using collision_validator::CollisionCandidate;
using collision_validator::CollisionEvaluator;
using collision_validator::CollisionEvaluatorParam;
using collision_validator::PointCloudClusters;
using collision_validator::PointCluster;
using collision_validator::detail::PoseTrajectory;
using collision_validator::detail::TimeTrajectory;
using collision_validator::detail::TrajectoryData;

// Same vehicle as autoware_test_utils/config/test_vehicle_info.param.yaml:
// front offset 3.79 m, rear overhang 1.1 m, width 1.896 m.
inline vehicle_info_utils::VehicleInfo test_vehicle_info(
  const double front_overhang = 1.0, const double vehicle_height = 2.5)
{
  return vehicle_info_utils::createVehicleInfo(
    0.383, 0.235, 2.79, 1.64, front_overhang, 1.1, 0.128, 0.128, vehicle_height, 0.70);
}

constexpr double kDt = 0.1;
constexpr double kEgoVelocity = 10.0;
constexpr double kFrontOffset = 3.79;

inline rclcpp::Time at(const double sec)
{
  return rclcpp::Time(static_cast<int64_t>(sec * 1e9), RCL_ROS_TIME);
}

inline builtin_interfaces::msg::Time to_msg_time(const double sec)
{
  return at(sec);
}

inline Trajectory make_straight_trajectory(
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
inline Trajectory make_arc_trajectory(
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

// Stopped for 0.5 s, then moving forward at kEgoVelocity.
inline Trajectory make_starting_trajectory()
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

inline nav_msgs::msg::Odometry make_odometry(const double velocity = 0.0)
{
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = velocity;
  return odom;
}

// Grid of points covering [x_min, x_max] x [y_min, y_max] at 0.25 m spacing, as one cluster.
inline PointCluster make_cluster(
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

inline PointCluster make_single_point_cluster(const double x, const double y)
{
  PointCluster cluster;
  cluster.points.emplace_back(x, y);
  cluster.centroid = Point2d{x, y};
  return cluster;
}

inline PredictedObject make_object(
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
inline void add_straight_path(PredictedObject & object, const double vx, const double yaw)
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

// Objects stamped at the trajectory stamp.
inline PredictedObjects objects_at(
  const Trajectory & predicted, const std::vector<PredictedObject> & objects = {})
{
  PredictedObjects out;
  out.header.stamp = predicted.header.stamp;
  out.objects = objects;
  return out;
}

inline CollisionEvaluatorParam default_param()
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

inline const builtin_interfaces::msg::Time kCloudStamp = to_msg_time(99.9);

inline std::vector<CollisionCandidate> evaluate(
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

inline double expected_static_ttc(const double gap_from_front)
{
  return std::ceil(gap_from_front / kEgoVelocity / kDt - 1e-9) * kDt;
}

inline bool is_point(const CollisionCandidate & candidate)
{
  return candidate.polygon_now.outer().size() == 1;
}

inline std::vector<CollisionCandidate> points_of(const std::vector<CollisionCandidate> & candidates)
{
  std::vector<CollisionCandidate> out;
  std::copy_if(candidates.begin(), candidates.end(), std::back_inserter(out), is_point);
  return out;
}

inline std::vector<CollisionCandidate> objects_of(
  const std::vector<CollisionCandidate> & candidates)
{
  std::vector<CollisionCandidate> out;
  std::copy_if(candidates.begin(), candidates.end(), std::back_inserter(out), [](const auto & c) {
    return !is_point(c);
  });
  return out;
}

inline TrajectoryData ego_trajectory_data(const Trajectory & predicted)
{
  TimeTrajectory times;
  PoseTrajectory poses;
  for (const auto & point : predicted.points) {
    times.push_back(rclcpp::Duration(point.time_from_start).seconds());
    poses.push_back(point.pose);
  }
  return TrajectoryData(
    times, poses,
    collision_validator::detail::compute_footprint_trajectory(
      poses, collision_validator::detail::EgoDimensions{kFrontOffset, 1.1, 0.948, 0.948}));
}

inline std::optional<double> point_collision_time(
  const Trajectory & predicted, const Point2d & point)
{
  const auto ego = ego_trajectory_data(predicted);
  const auto k = collision_validator::detail::find_first_collision(ego, point);
  return k ? std::optional<double>{ego.getTimes()[*k]} : std::nullopt;
}

inline std::optional<double> object_collision_time(
  const Trajectory & predicted, const TrajectoryData & object_trajectory)
{
  const auto ego = ego_trajectory_data(predicted);
  const auto step = collision_validator::detail::find_first_collision(ego, object_trajectory);
  return step ? std::optional<double>{ego.getTimes()[step->ref_index]} : std::nullopt;
}

inline TrajectoryData path_trajectory(const PredictedObject & object, const double time_offset)
{
  return collision_validator::detail::generate_predicted_path_trajectory(
    object, object.kinematics.predicted_paths.front(), 5.0 + std::abs(time_offset), time_offset,
    kDt, std::nullopt);
}

inline std::optional<double> collision_cos_of(
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

inline geometry_msgs::msg::Pose make_pose(const double x, const double y, const double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
  return pose;
}

inline ::predicted_trajectory_validator::Params default_params()
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
  params.hazard_warning.min_alert_duration = 0.5;
  return params;
}

// Point cloud

struct Xyz
{
  float x;
  float y;
  float z;
};

enum class Layout { XYZ, XYZIRC, XYZIRCAEDT };

inline sensor_msgs::msg::PointCloud2 make_cloud(
  const std::vector<Xyz> & points, const Layout layout, const rclcpp::Time & stamp = at(10.0),
  const std::string & frame_id = "base_link")
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  using sensor_msgs::msg::PointField;
  if (layout == Layout::XYZ) {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  } else if (layout == Layout::XYZIRC) {
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
inline void add_blob(
  std::vector<Xyz> & points, const float x0, const float y0, const float z = 1.0F)
{
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      points.push_back({x0 + 0.1F * i, y0 + 0.1F * j, z});
    }
  }
}

inline std::vector<Xyz> two_blobs_and_an_isolated_point()
{
  std::vector<Xyz> points;
  add_blob(points, 5.0F, 0.0F);
  add_blob(points, 8.0F, 2.0F);
  points.push_back({3.0F, -3.0F, 1.0F});
  return points;
}

inline collision_validator::PointCloudParam default_pointcloud_param()
{
  collision_validator::PointCloudParam param;
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

inline collision_validator::PointCloudPreprocessor make_preprocessor(
  const collision_validator::PointCloudParam & param = default_pointcloud_param())
{
  return collision_validator::PointCloudPreprocessor(
    rclcpp::get_logger("test_pointcloud_preprocessor"), param);
}

inline const std::optional<geometry_msgs::msg::Pose> kOrigin = make_pose(0.0, 0.0, 0.0);

inline std::vector<PointCluster> sorted_by_x(std::vector<PointCluster> clusters)
{
  std::sort(clusters.begin(), clusters.end(), [](const auto & a, const auto & b) {
    return a.centroid.x() < b.centroid.x();
  });
  return clusters;
}

// Collision validator

inline CollisionValidator make_validator(
  const ::predicted_trajectory_validator::Params & params = default_params())
{
  return CollisionValidator(rclcpp::get_logger("test_collision_validator"), params);
}

inline nav_msgs::msg::Odometry make_odom(
  const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & pose)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "map";
  odom.pose.pose = pose;
  odom.twist.twist.linear.x = kEgoVelocity;
  return odom;
}

inline Trajectory make_trajectory(
  const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & start)
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

inline PredictedObject make_car(const double x, const double y)
{
  return make_object(x, y, 0.0, 0.0, ObjectClassification::CAR);
}

inline PredictedObjects::ConstSharedPtr make_objects(
  const rclcpp::Time & stamp, const std::vector<PredictedObject> & objects)
{
  auto msg = std::make_shared<PredictedObjects>();
  msg->header.stamp = stamp;
  msg->header.frame_id = "map";
  msg->objects = objects;
  return msg;
}

// 25 points (5 x 5 at 0.1 m spacing) from (x0, -0.2) in base_link.
inline sensor_msgs::msg::PointCloud2::ConstSharedPtr make_cloud(
  const rclcpp::Time & stamp, const double x0)
{
  std::vector<Xyz> points;
  add_blob(points, static_cast<float>(x0), -0.2F);
  return std::make_shared<sensor_msgs::msg::PointCloud2>(make_cloud(points, Layout::XYZ, stamp));
}

inline size_t count_points(const std::vector<HazardSource> & sources)
{
  return std::count_if(sources.begin(), sources.end(), [](const auto & source) {
    return source.shape.outer().size() == 1;
  });
}

// Hazard warning

inline HazardWarning make_warning(
  const ::predicted_trajectory_validator::Params & params = default_params())
{
  return HazardWarning(rclcpp::get_logger("test_hazard_warning"), params);
}

inline HazardSource make_box(
  const double x_min, const double x_max, const double y_min, const double y_max,
  const uint8_t risk_level,
  const uint8_t sources = autoware_internal_control_msgs::msg::HazardSector::SOURCE_COLLISION)
{
  HazardSource source;
  source.shape.outer() = {
    {x_min, y_min}, {x_min, y_max}, {x_max, y_max}, {x_max, y_min}, {x_min, y_min}};
  boost::geometry::correct(source.shape);
  source.risk_level = risk_level;
  source.sources = sources;
  return source;
}

inline HazardSource make_point(
  const double x, const double y, const uint8_t risk_level,
  const uint8_t sources =
    autoware_internal_control_msgs::msg::HazardSector::SOURCE_BOUNDARY_DEPARTURE)
{
  HazardSource source;
  source.shape.outer().emplace_back(x, y);
  source.risk_level = risk_level;
  source.sources = sources;
  return source;
}

inline HazardWarningSources collision_only(const std::vector<HazardSource> & sources)
{
  HazardWarningSources hazard_sources;
  hazard_sources.collision = sources;
  return hazard_sources;
}
}  // namespace autoware::predicted_trajectory_validator::test_utils

#endif  // TEST_UTILS_HPP_
