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

#include "obstacle_stop.hpp"

#include "../utils/sl_view_utils.hpp"

#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iterator>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

Pose2d to_pose2d(const geometry_msgs::msg::Pose & pose)
{
  return Pose2d{
    Point2d{pose.position.x, pose.position.y}, autoware_utils_geometry::get_rpy(pose).z};
}

bool is_finite(const Pose2d & pose)
{
  return std::isfinite(pose.position.x()) && std::isfinite(pose.position.y()) &&
         std::isfinite(pose.yaw);
}

//! 確率最大の予測経路 (同率は msg 順の先勝ち = 決定的)。無ければ nullptr
const autoware_perception_msgs::msg::PredictedPath * select_most_confident_path(
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  const autoware_perception_msgs::msg::PredictedPath * best = nullptr;
  for (const auto & path : object.kinematics.predicted_paths) {
    if (path.path.empty() || !std::isfinite(path.confidence)) {
      continue;
    }
    if (best == nullptr || path.confidence > best->confidence) {
      best = &path;
    }
  }
  return best;
}

}  // namespace

std::vector<Constraint> make_obstacle_keep_out_constraints(
  const PredictedObjects & objects, const rclcpp::Time & t_plan, const double margin_m)
{
  // S1 §2-1: 時刻は t_plan 基準の相対秒。perception のスタンプは odometry のスタンプと
  // 一致しないので、その差をオフセットとして waypoints の t に入れる
  // (負のスタンプは rclcpp::Time が throw するのでオフセット無しに倒す)
  const double base_offset_s =
    objects.header.stamp.sec < 0 ? 0.0 : (rclcpp::Time(objects.header.stamp) - t_plan).seconds();

  std::vector<Constraint> constraints;
  constraints.reserve(objects.objects.size());
  for (const auto & object : objects.objects) {
    // 物体ローカル形状: 恒等 pose で to_polygon2d を呼ぶと shape 種別
    // (BOUNDING_BOX / CYLINDER / POLYGON) を吸収してローカル多角形が得られる。
    // CW・閉で返るので S1 の契約 (constraint.hpp) をそのまま満たす。
    // 退化した shape (頂点 0 の POLYGON 等) は送出しうるので、その物体を落として続行する
    Polygon2d shape;
    try {
      shape = autoware_utils_geometry::to_polygon2d(geometry_msgs::msg::Pose{}, object.shape);
    } catch (const std::exception &) {
      continue;
    }
    if (shape.outer().size() < 4) {  // 閉リングで 4 未満 = 面を持たない
      continue;
    }

    const auto initial_pose = to_pose2d(object.kinematics.initial_pose_with_covariance.pose);
    if (!is_finite(initial_pose)) {
      continue;
    }

    std::vector<TimedPose> waypoints;
    if (const auto * path = select_most_confident_path(object)) {
      const double time_step_s = rclcpp::Duration(path->time_step).seconds();
      waypoints.reserve(path->path.size());
      for (std::size_t i = 0; i < path->path.size(); ++i) {
        TimedPose waypoint;
        waypoint.t = base_offset_s + static_cast<double>(i) * time_step_s;
        waypoint.pose = to_pose2d(path->path[i]);
        if (!std::isfinite(waypoint.t) || !is_finite(waypoint.pose)) {
          continue;  // 退化した点は落として続行
        }
        waypoints.push_back(waypoint);
      }
    }
    if (waypoints.empty()) {
      // 予測経路が無い = 静的物体。waypoint 1 点で無期限に現在位置を占有する (constraint.hpp)
      waypoints.push_back(TimedPose{0.0, initial_pose});
    }

    Constraint constraint;
    constraint.certainty = Certainty::DEFINITE;
    // time は既定 (常時)。waypoints の時刻範囲外は RigidBody 自体が無効になるので窓では切らない
    constraint.payload = KeepOut{RigidBody{std::move(shape), std::move(waypoints)}, margin_m};
    constraint.source = Source{
      "obstacle_stop", Category::SAFETY, autoware_utils_uuid::to_hex_string(object.object_id),
      "dynamic_obstacle"};
    constraints.push_back(std::move(constraint));
  }
  return constraints;
}

std::vector<Constraint> make_obstacle_stop_line_constraints(
  const PredictedObjects & objects, const PathPointTrajectory & reference_path, const double s_ego,
  const double corridor_half_width_m, const double stop_distance_m,
  const double max_object_speed_mps)
{
  std::vector<Constraint> constraints;
  for (const auto & object : objects.objects) {
    if (
      std::abs(object.kinematics.initial_twist_with_covariance.twist.linear.x) >
      max_object_speed_mps) {
      continue;
    }
    // World-frame footprint at the current pose (the same conversion the KeepOut uses)
    Polygon2d footprint;
    try {
      footprint = autoware_utils_geometry::to_polygon2d(
        object.kinematics.initial_pose_with_covariance.pose, object.shape);
    } catch (const std::exception &) {
      continue;
    }
    if (footprint.outer().size() < 4) {
      continue;
    }

    // (s, l) extent of the footprint on the reference path. The chord approximation of
    // closest() is enough here: the stop line is placed stop_distance_m away anyway
    double s_min = std::numeric_limits<double>::infinity();
    double s_max = -std::numeric_limits<double>::infinity();
    double l_min = std::numeric_limits<double>::infinity();
    double l_max = -std::numeric_limits<double>::infinity();
    bool finite = true;
    for (const auto & vertex : footprint.outer()) {
      if (!std::isfinite(vertex.x()) || !std::isfinite(vertex.y())) {
        finite = false;
        break;
      }
      geometry_msgs::msg::Point q;
      q.x = vertex.x();
      q.y = vertex.y();
      const double s = experimental::trajectory::closest(reference_path, q);
      s_min = std::min(s_min, s);
      s_max = std::max(s_max, s);
      const double l = lateral_offset_at(reference_path, s, vertex);
      l_min = std::min(l_min, l);
      l_max = std::max(l_max, l);
    }
    if (!finite || s_max <= s_ego) {
      continue;  // behind the ego
    }
    if (l_max < -corridor_half_width_m || l_min > corridor_half_width_m) {
      continue;  // beside the path, not on it
    }
    // If the object is already closer than stop_distance_m, the line is placed at the ego so that
    // every forward candidate is rejected and the planner falls back to the stop trajectory
    const double s_stop = std::clamp(s_min - stop_distance_m, s_ego, reference_path.length());

    // Only the crossing with the centerline matters for the projection (StopBarEntry); the width
    // just has to cover the corridor
    const auto left = to_world_pose(reference_path, s_stop, corridor_half_width_m).position;
    const auto right = to_world_pose(reference_path, s_stop, -corridor_half_width_m).position;

    Constraint constraint;
    constraint.certainty = Certainty::DEFINITE;
    // Gate: left of first -> second is forbidden, so first = left end, second = right end makes
    // the far side (along the path) forbidden
    constraint.payload = Gate{Segment2d{left, right}, 0.0};
    constraint.source = Source{
      "obstacle_stop", Category::SAFETY, autoware_utils_uuid::to_hex_string(object.object_id),
      "stop_line"};
    constraints.push_back(std::move(constraint));
  }
  return constraints;
}

ConstraintGeneratorOutput ObstacleStopConstraintGenerator::generate_constraints(
  const PlannerContext & context)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  ConstraintGeneratorOutput output;
  if (!context.predicted_objects) {
    return output;  // 未受信の周期は空の制約列でパイプラインを継続させる (S7 §2)
  }

  output.constraints = make_obstacle_keep_out_constraints(
    *context.predicted_objects, rclcpp::Time(context.odometry.header.stamp),
    params_.obstacle_stop.margin_m);
  if (params_.obstacle_stop.stop_distance_m > 0.0) {
    auto stop_lines = make_obstacle_stop_line_constraints(
      *context.predicted_objects, context.reference_path, compute_ego_frenet_state(context).s,
      context.vehicle_info.vehicle_width_m / 2.0 + params_.obstacle_stop.margin_m,
      params_.obstacle_stop.stop_distance_m, params_.obstacle_stop.stop_line_max_object_speed_mps);
    output.constraints.insert(
      output.constraints.end(), std::make_move_iterator(stop_lines.begin()),
      std::make_move_iterator(stop_lines.end()));
  }

  // --- debug marker: 現在位置の footprint と予測経路 ---
  {
    using autoware_utils_visualization::create_default_marker;
    using autoware_utils_visualization::create_marker_color;
    using autoware_utils_visualization::create_marker_scale;

    const double z = context.odometry.pose.pose.position.z;
    MarkerArray marker_array;
    // 物体数が周期で変わると前周期のマーカーが残るので DELETEALL を先頭に挟む
    Marker delete_all;
    delete_all.action = Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);

    std::int32_t id = 0;
    for (const auto & constraint : output.constraints) {
      if (const auto * gate = std::get_if<Gate>(&constraint.payload)) {
        auto stop_line_marker = create_default_marker(
          "map", rclcpp::Time(0, 0, RCL_ROS_TIME), "obstacle_stop_stop_line", id,
          Marker::LINE_STRIP, create_marker_scale(0.2, 0.0, 0.0),
          create_marker_color(1.0, 0.0, 0.0, 0.9));
        for (const auto & end : {gate->line.first, gate->line.second}) {
          geometry_msgs::msg::Point q;
          q.x = end.x();
          q.y = end.y();
          q.z = z;
          stop_line_marker.points.push_back(q);
        }
        marker_array.markers.push_back(stop_line_marker);
        ++id;
        continue;
      }
      const auto & rigid_body =
        std::get<RigidBody>(std::get<KeepOut>(constraint.payload).occupancy);
      const auto & anchor = rigid_body.waypoints.front().pose;
      const double c = std::cos(anchor.yaw);
      const double s = std::sin(anchor.yaw);

      auto footprint_marker = create_default_marker(
        "map", rclcpp::Time(0, 0, RCL_ROS_TIME), "obstacle_stop_footprint", id, Marker::LINE_STRIP,
        create_marker_scale(0.1, 0.0, 0.0), create_marker_color(1.0, 0.2, 0.2, 0.9));
      for (const auto & point : rigid_body.shape.outer()) {
        geometry_msgs::msg::Point q;
        q.x = anchor.position.x() + c * point.x() - s * point.y();
        q.y = anchor.position.y() + s * point.x() + c * point.y();
        q.z = z;
        footprint_marker.points.push_back(q);
      }
      marker_array.markers.push_back(footprint_marker);

      auto path_marker = create_default_marker(
        "map", rclcpp::Time(0, 0, RCL_ROS_TIME), "obstacle_stop_predicted_path", id,
        Marker::LINE_STRIP, create_marker_scale(0.05, 0.0, 0.0),
        create_marker_color(1.0, 0.5, 0.2, 0.7));
      for (const auto & waypoint : rigid_body.waypoints) {
        geometry_msgs::msg::Point q;
        q.x = waypoint.pose.position.x();
        q.y = waypoint.pose.position.y();
        q.z = z;
        path_marker.points.push_back(q);
      }
      if (path_marker.points.size() >= 2) {
        marker_array.markers.push_back(path_marker);
      }
      ++id;
    }
    output.debug_markers = std::move(marker_array);
  }

  return output;
}

}  // namespace autoware::safety_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::ObstacleStopConstraintGenerator,
  autoware::safety_planner::ConstraintGeneratorInterface)
