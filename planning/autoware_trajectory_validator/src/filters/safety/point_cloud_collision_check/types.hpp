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

#ifndef FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__TYPES_HPP_
#define FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__TYPES_HPP_

#include <autoware/signal_processing/lowpass_filter_1d.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using ObjectClassification = autoware_perception_msgs::msg::ObjectClassification;
using Point2d = autoware_utils_geometry::Point2d;
using Polygon2d = autoware_utils_geometry::Polygon2d;
using Shape = autoware_perception_msgs::msg::Shape;
using UUID = unique_identifier_msgs::msg::UUID;

struct StopObstacleClassification
{
  enum class Type {
    UNKNOWN,
    CAR,
    TRUCK,
    BUS,
    TRAILER,
    MOTORCYCLE,
    BICYCLE,
    PEDESTRIAN,
    ANIMAL,
    HAZARD,
    POINTCLOUD
  };

  inline static const std::unordered_map<Type, std::string> to_string_map = {
    {Type::UNKNOWN, "unknown"},      {Type::CAR, "car"},
    {Type::TRUCK, "truck"},          {Type::BUS, "bus"},
    {Type::TRAILER, "trailer"},      {Type::MOTORCYCLE, "motorcycle"},
    {Type::BICYCLE, "bicycle"},      {Type::PEDESTRIAN, "pedestrian"},
    {Type::ANIMAL, "animal"},        {Type::HAZARD, "hazard"},
    {Type::POINTCLOUD, "pointcloud"}};

  explicit StopObstacleClassification(const ObjectClassification object_classification)
  {
    switch (object_classification.label) {
      case ObjectClassification::UNKNOWN:
        label = Type::UNKNOWN;
        break;
      case ObjectClassification::CAR:
        label = Type::CAR;
        break;
      case ObjectClassification::TRUCK:
        label = Type::TRUCK;
        break;
      case ObjectClassification::BUS:
        label = Type::BUS;
        break;
      case ObjectClassification::TRAILER:
        label = Type::TRAILER;
        break;
      case ObjectClassification::MOTORCYCLE:
        label = Type::MOTORCYCLE;
        break;
      case ObjectClassification::BICYCLE:
        label = Type::BICYCLE;
        break;
      case ObjectClassification::PEDESTRIAN:
        label = Type::PEDESTRIAN;
        break;
      case ObjectClassification::ANIMAL:
        label = Type::ANIMAL;
        break;
      case ObjectClassification::HAZARD:
        label = Type::HAZARD;
        break;
      default:
        throw std::invalid_argument("Undefined ObjectClassification label");
    }
  }
  explicit StopObstacleClassification(
    const std::vector<ObjectClassification> & object_classifications)
  : StopObstacleClassification(object_classifications.at(0))
  {
  }
  explicit StopObstacleClassification(Type v) : label(v) {}
  StopObstacleClassification() = default;

  std::string to_string() const { return to_string_map.at(label); }

  Type label{};

  bool operator==(const StopObstacleClassification & other) const { return label == other.label; }
  bool operator!=(const StopObstacleClassification & other) const { return !(*this == other); }
};

struct CollisionPointWithDist
{
  geometry_msgs::msg::Point point{};
  double dist_to_collide{};
};

struct PointcloudStopCandidate
{
  std::vector<double> initial_velocities{};
  autoware::signal_processing::LowpassFilter1d vel_lpf{0.0};
  rclcpp::Time latest_collision_pointcloud_time;
  CollisionPointWithDist latest_collision_point;
};

struct PolygonParam
{
  std::optional<double> trimming_length{};
  double lateral_margin{};
  double off_track_scale{};

  bool operator<(const PolygonParam & other) const
  {
    return std::tie(trimming_length, lateral_margin, off_track_scale) <
           std::tie(other.trimming_length, other.lateral_margin, other.off_track_scale);
  }
};

struct StopObstacle
{
  StopObstacle(
    const UUID & arg_uuid, const rclcpp::Time & arg_stamp,
    const StopObstacleClassification & arg_object_classification,
    const geometry_msgs::msg::Pose & arg_pose, const Shape & arg_shape,
    const double arg_lon_velocity, const geometry_msgs::msg::Point & arg_collision_point,
    const double arg_dist_to_collide_on_decimated_traj, const PolygonParam & arg_polygon_param,
    const std::optional<double> arg_braking_dist = std::nullopt)
  : uuid(arg_uuid),
    stamp(arg_stamp),
    pose(arg_pose),
    velocity(arg_lon_velocity),
    shape(arg_shape),
    collision_point(arg_collision_point),
    dist_to_collide_on_decimated_traj(arg_dist_to_collide_on_decimated_traj),
    classification(arg_object_classification),
    polygon_param(arg_polygon_param),
    braking_dist(arg_braking_dist)
  {
  }
  StopObstacle(
    const rclcpp::Time & arg_stamp, const StopObstacleClassification & arg_object_classification,
    const double arg_lon_velocity, const geometry_msgs::msg::Point & arg_collision_point,
    const double arg_dist_to_collide_on_decimated_traj, const PolygonParam & arg_polygon_param,
    const std::optional<double> arg_braking_dist = std::nullopt)
  : stamp(arg_stamp),
    velocity(arg_lon_velocity),
    collision_point(arg_collision_point),
    dist_to_collide_on_decimated_traj(arg_dist_to_collide_on_decimated_traj),
    classification(arg_object_classification),
    polygon_param(arg_polygon_param),
    braking_dist(arg_braking_dist)
  {
    if (arg_object_classification.label != StopObstacleClassification::Type::POINTCLOUD) {
      throw std::invalid_argument(
        "Constructor for pointcloud StopObstacle must be called with POINTCLOUD label");
    }
    pose.position = arg_collision_point;
    shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  }
  UUID uuid{};
  rclcpp::Time stamp;
  geometry_msgs::msg::Pose pose;
  double velocity;

  Shape shape;
  geometry_msgs::msg::Point collision_point;
  double dist_to_collide_on_decimated_traj;
  StopObstacleClassification classification;
  PolygonParam polygon_param;
  std::optional<double> braking_dist;
};

struct DetectionPolygon
{
  const std::vector<TrajectoryPoint> traj_points;
  const std::vector<Polygon2d> polygons;
  DetectionPolygon(std::vector<TrajectoryPoint> && points, std::vector<Polygon2d> && polys)
  : traj_points(std::move(points)), polygons(std::move(polys))
  {
    if (traj_points.size() != polygons.size()) {
      throw std::invalid_argument("Vector sizes must be identical for DetectionPolygon.");
    }
  }
};

}  // namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check

#endif  // FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__TYPES_HPP_
