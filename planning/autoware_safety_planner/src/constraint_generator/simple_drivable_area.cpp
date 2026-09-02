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

#include "simple_drivable_area.hpp"

#include <autoware_utils_visualization/marker_helper.hpp>

#include <boost/geometry/algorithms/correct.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

//! 折れ線を marker の点列へ (z は ego 高さを流用する)
void append_points(Marker & marker, const std::vector<Point2d> & points, const double z)
{
  for (const auto & p : points) {
    geometry_msgs::msg::Point q;
    q.x = p.x();
    q.y = p.y();
    q.z = z;
    marker.points.push_back(q);
  }
}

}  // namespace

DrivableAreaShape make_drivable_area_shape(
  const std::vector<Pose2d> & centerline, const double half_width_m,
  const double forward_extension_m, const double backward_extension_m)
{
  DrivableAreaShape shape;
  if (centerline.size() < 2) {
    return shape;
  }

  // 前後の延長は端点の接線方向へまっすぐ伸ばす (中心線を延長してから左右へ振る)
  std::vector<Pose2d> extended;
  extended.reserve(centerline.size() + 2);
  if (backward_extension_m > 0.0) {
    const auto & f = centerline.front();
    extended.push_back(
      Pose2d{
        Point2d{
          f.position.x() - backward_extension_m * std::cos(f.yaw),
          f.position.y() - backward_extension_m * std::sin(f.yaw)},
        f.yaw});
  }
  extended.insert(extended.end(), centerline.begin(), centerline.end());
  if (forward_extension_m > 0.0) {
    const auto & b = centerline.back();
    extended.push_back(
      Pose2d{
        Point2d{
          b.position.x() + forward_extension_m * std::cos(b.yaw),
          b.position.y() + forward_extension_m * std::sin(b.yaw)},
        b.yaw});
  }

  shape.left.reserve(extended.size());
  shape.right.reserve(extended.size());
  for (const auto & pose : extended) {
    const double nx = -std::sin(pose.yaw);  // 左向き法線
    const double ny = std::cos(pose.yaw);
    shape.left.emplace_back(
      pose.position.x() + half_width_m * nx, pose.position.y() + half_width_m * ny);
    shape.right.emplace_back(
      pose.position.x() - half_width_m * nx, pose.position.y() - half_width_m * ny);
  }

  // 閉ポリゴン: 左辺を進行方向へ → 右辺を逆向きへ。bg::correct が CW・閉へ正規化する
  auto & ring = shape.polygon.outer();
  ring.reserve(2 * extended.size() + 1);
  ring.insert(ring.end(), shape.left.begin(), shape.left.end());
  ring.insert(ring.end(), shape.right.rbegin(), shape.right.rend());
  boost::geometry::correct(shape.polygon);

  return shape;
}

ConstraintGeneratorOutput SimpleDrivableAreaConstraintGenerator::generate_constraints(
  const PlannerContext & context)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  ConstraintGeneratorOutput output;

  const auto & p = params_.simple_drivable_area;
  const auto & path = context.reference_path;
  const double length = path.length();
  if (!(length > 0.0)) {
    return output;
  }

  // 中心線を定間隔でサンプルする (端点は必ず含める)
  const auto num_division =
    static_cast<std::size_t>(std::ceil(length / std::max(p.sample_interval_m, 1e-3)));
  std::vector<Pose2d> centerline;
  centerline.reserve(num_division + 1);
  for (std::size_t i = 0; i <= num_division; ++i) {
    const double s = std::min(static_cast<double>(i) * p.sample_interval_m, length);
    const auto position = path.compute(s).point.pose.position;
    centerline.push_back(Pose2d{Point2d{position.x, position.y}, path.azimuth(s)});
  }

  const auto shape = make_drivable_area_shape(
    centerline, p.half_width_m, p.forward_extension_m, p.backward_extension_m);
  if (shape.left.size() < 2) {
    return output;
  }

  // 制約 IR にポリゴン内包の payload は無いので、左辺・右辺を Boundary 2 本へ分解する。
  // 禁止側は「ポリゴンの外側」= 左辺の左 / 右辺の右
  const auto add_boundary =
    [&](const std::vector<Point2d> & points, const Side side, const std::string & detail) {
      Constraint constraint;
      // time は既定 (常時。周期内静的)
      Boundary boundary;
      boundary.polyline.reserve(points.size());
      for (const auto & point : points) {
        boundary.polyline.push_back(point);
      }
      boundary.forbidden_side = side;
      boundary.margin = p.margin_m;
      constraint.payload = std::move(boundary);
      // 簡易版は reference_path から幾何を作るので、対象の lanelet id を持たない (target_id は空)
      constraint.source = Source{get_name(), Category::SAFETY, "", detail};
      output.constraints.push_back(std::move(constraint));
    };
  add_boundary(shape.left, Side::LEFT, "left_bound");
  add_boundary(shape.right, Side::RIGHT, "right_bound");

  // --- debug marker: 走行可能領域のポリゴンと左右境界 ---
  {
    using autoware_utils_visualization::create_default_marker;
    using autoware_utils_visualization::create_marker_color;
    using autoware_utils_visualization::create_marker_scale;

    const double z = context.odometry.pose.pose.position.z;
    MarkerArray marker_array;

    auto polygon_marker = create_default_marker(
      "map", rclcpp::Time(0, 0, RCL_ROS_TIME), "simple_drivable_area_polygon", 0,
      Marker::LINE_STRIP, create_marker_scale(0.1, 0.0, 0.0),
      create_marker_color(0.0, 1.0, 0.5, 0.6));
    std::vector<Point2d> ring_points(shape.polygon.outer().begin(), shape.polygon.outer().end());
    append_points(polygon_marker, ring_points, z);
    if (polygon_marker.points.size() >= 2) {
      // bg::correct 済みなので始点 == 終点 (閉じている)
      marker_array.markers.push_back(polygon_marker);
    }

    auto bound_marker = create_default_marker(
      "map", rclcpp::Time(0, 0, RCL_ROS_TIME), "simple_drivable_area_bounds", 0, Marker::LINE_LIST,
      create_marker_scale(0.15, 0.0, 0.0), create_marker_color(1.0, 0.6, 0.0, 0.9));
    for (const auto & points : {shape.left, shape.right}) {
      for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        append_points(bound_marker, {points[i], points[i + 1]}, z);
      }
    }
    if (!bound_marker.points.empty()) {
      marker_array.markers.push_back(bound_marker);
    }

    output.debug_markers = std::move(marker_array);
  }

  return output;
}

}  // namespace autoware::safety_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::SimpleDrivableAreaConstraintGenerator,
  autoware::safety_planner::ConstraintGeneratorInterface)
