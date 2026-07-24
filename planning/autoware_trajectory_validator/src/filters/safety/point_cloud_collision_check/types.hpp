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

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <Eigen/Core>

#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using Polygon2d = autoware_utils_geometry::Polygon2d;

// 点群由来障害物のラベル。認識物体は扱わないため POINTCLOUD 固定。
struct StopObstacleClassification
{
  enum class Type { POINTCLOUD };
  Type label{Type::POINTCLOUD};

  std::string to_string() const { return "pointcloud"; }
  bool operator==(const StopObstacleClassification & other) const { return label == other.label; }
  bool operator!=(const StopObstacleClassification & other) const { return !(*this == other); }
};

// 衝突点と、その衝突点までの（軌道に沿った）距離。
struct CollisionPointWithDist
{
  geometry_msgs::msg::Point point{};
  double dist_to_collide{};
};

// 点群由来の停止候補。map 系の世界点で時系列追跡し、速度は map 系の世界変位ベクトル
// (Δp/dt) を成分ごとに LPF する（決定10）。dist_to_collide は候補依存量のため保持せず、
// 読み出し時に各候補軌道へ射影して求める。
struct PointcloudStopCandidate
{
  std::vector<Eigen::Vector2d> initial_velocities{};
  autoware::signal_processing::LowpassFilter1d vel_lpf_x{0.0};
  autoware::signal_processing::LowpassFilter1d vel_lpf_y{0.0};
  rclcpp::Time latest_collision_pointcloud_time;
  geometry_msgs::msg::Point latest_world_point{};

  bool has_velocity() const
  {
    return vel_lpf_x.getValue().has_value() && vel_lpf_y.getValue().has_value();
  }
};

// 検出ポリゴン生成パラメータ。生成結果キャッシュのキーにもなるため operator< を持つ。
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

// 停止対象として確定した点群障害物。
struct StopObstacle
{
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
    pose.position = arg_collision_point;
  }

  rclcpp::Time stamp;
  geometry_msgs::msg::Pose pose;
  double velocity;
  geometry_msgs::msg::Point collision_point;
  double dist_to_collide_on_decimated_traj;
  StopObstacleClassification classification;
  PolygonParam polygon_param;
  std::optional<double> braking_dist;
};

// 衝突検出に用いる軌道ポリゴン群と、その元の軌道点列。両者は 1 対 1 対応する。
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
