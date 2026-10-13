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

#include "../utils/frenet_utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory_modifier/trajectory_modifier_utils/utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner::experimental
{

namespace
{

using autoware::trajectory_modifier::utils::clamp_stop_point_arc_length;
using autoware::trajectory_modifier::utils::obstacle_stop::build_trajectory_footprint_index;
using autoware::trajectory_modifier::utils::obstacle_stop::get_nearest_object_collision;
using autoware::trajectory_modifier::utils::obstacle_stop::LateralMarginMap;
using autoware::trajectory_modifier::utils::obstacle_stop::ObjectDecelMap;
using autoware::trajectory_modifier::utils::obstacle_stop::ObjectFilter;
using autoware::trajectory_modifier::utils::obstacle_stop::ObjectType;
using autoware::trajectory_modifier::utils::obstacle_stop::TargetObject;
using autoware::trajectory_modifier::utils::obstacle_stop::TargetObjects;
using autoware::trajectory_modifier::utils::obstacle_stop::TrajectoryShape;

TrajectoryPoints make_trajectory_points(
  const PathPointTrajectory & path, const double s_ego, const geometry_msgs::msg::Point & ego_point)
{
  constexpr double RESAMPLE_INTERVAL_M = 0.5;

  TrajectoryPoints points;
  const auto push = [&](const double s) {
    const auto & point = path.compute(s).point;
    TrajectoryPoint trajectory_point;
    trajectory_point.pose = point.pose;
    trajectory_point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
    trajectory_point.lateral_velocity_mps = point.lateral_velocity_mps;
    trajectory_point.heading_rate_rps = point.heading_rate_rps;
    points.push_back(trajectory_point);
  };
  for (double s = s_ego; s + 1e-3 < path.length(); s += RESAMPLE_INTERVAL_M) {
    push(s);
  }
  push(path.length());
  autoware::motion_utils::calculate_time_from_start(points, ego_point);
  return points;
}

MarkerArray make_debug_markers(
  const TrajectoryShape & shape, const TargetObjects & target_objects,
  const builtin_interfaces::msg::Time & stamp, const double ego_z)
{
  using autoware_utils_geometry::calc_offset_pose;
  using autoware_utils_geometry::create_point;
  using autoware_utils_visualization::create_default_marker;
  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;

  const auto white = create_marker_color(1.0, 1.0, 1.0, 1.0);
  const auto yellow = create_marker_color(1.0, 1.0, 0.0, 1.0);
  const auto magenta = create_marker_color(1.0, 0.0, 1.0, 1.0);

  MarkerArray marker_array;
  Marker delete_all;
  delete_all.action = Marker::DELETEALL;
  marker_array.markers.push_back(delete_all);

  const auto add_polygon_marker = [&](
                                    const Polygon2d & polygon, const std::string & ns, const int id,
                                    const std_msgs::msg::ColorRGBA & color) {
    auto marker = create_default_marker(
      "map", stamp, ns, id, Marker::LINE_STRIP, create_marker_scale(0.1, 0.1, 0.1), color);
    for (const auto & p : polygon.outer()) {
      marker.points.push_back(create_point(p.x(), p.y(), ego_z));
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    marker_array.markers.push_back(marker);
  };

  int id = 0;
  {
    const auto half_width = shape.ego_half_width;
    const auto front_offset = std::abs(shape.ego_front_offset);
    const auto back_offset = std::abs(shape.ego_back_offset);
    auto marker = create_default_marker(
      "map", stamp, "traj_polygon", id, Marker::LINE_LIST, create_marker_scale(0.1, 0.1, 0.1),
      yellow);
    auto dist = 0.0;
    for (const auto & footprint : shape.footprints) {
      if (footprint.arc_length - dist < 1.0) continue;
      dist = footprint.arc_length;
      const auto front_left = calc_offset_pose(footprint.pose, front_offset, -half_width, 0.0);
      const auto front_right = calc_offset_pose(footprint.pose, front_offset, half_width, 0.0);
      const auto rear_right = calc_offset_pose(footprint.pose, -back_offset, half_width, 0.0);
      const auto rear_left = calc_offset_pose(footprint.pose, -back_offset, -half_width, 0.0);
      for (const auto * pose :
           {&rear_left, &front_left, &front_left, &front_right, &front_right, &rear_right,
            &rear_right, &rear_left}) {
        marker.points.push_back(pose->position);
      }
    }
    marker_array.markers.push_back(marker);
    id++;
  }

  {
    const auto & bounding_box = shape.bounding_box;
    Polygon2d polygon;
    polygon.outer().emplace_back(bounding_box.min_corner());
    polygon.outer().emplace_back(bounding_box.min_corner().x(), bounding_box.max_corner().y());
    polygon.outer().emplace_back(bounding_box.max_corner());
    polygon.outer().emplace_back(bounding_box.max_corner().x(), bounding_box.min_corner().y());
    add_polygon_marker(polygon, "traj_bounding_box", id, white);
    id++;
  }

  for (const auto & obj : target_objects) {
    add_polygon_marker(obj.polygon, "target_objects", id, magenta);
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "safe: " << (obj.is_safe ? "true" : "false") << "\n";
    ss << "rss_safe_dist: " << obj.safe_distance << " m" << "\n";
    ss << "dist_from_ego: " << obj.distance_from_ego << " m";
    auto text = create_default_marker(
      "map", stamp, "target_objects_text", id, Marker::TEXT_VIEW_FACING,
      create_marker_scale(0.4, 0.4, 0.4), white);
    text.pose = obj.object.kinematics.initial_pose_with_covariance.pose;
    text.pose.position.z += 1.0;
    text.text = ss.str();
    marker_array.markers.push_back(text);
    if (!obj.is_safe)
      add_polygon_marker(obj.ego_footprint, "overlapping_ego_footprint", id, magenta);
    id++;
  }

  return marker_array;
}

}  // namespace

ConstraintGeneratorOutput ObstacleStopConstraintGenerator::generate_constraints(
  const PlannerContext & context)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & p = params_.obstacle_stop;
  const auto & vehicle_info = context.vehicle_info;
  const auto & ego_pose = context.odometry.pose.pose;
  const double ego_vel = context.odometry.twist.twist.linear.x;
  const double ego_accel = context.acceleration.accel.accel.linear.x;
  const rclcpp::Time now(context.odometry.header.stamp);

  const double s_ego = compute_ego_frenet_state(context).s;
  const auto traj_points = make_trajectory_points(context.reference_path, s_ego, ego_pose.position);
  // NOTE(odashima): Why not ego_accel as is: the deceleration for this very stop would shorten the
  // detection range until the obstacle drops out of it, releasing the stop mid-way
  const auto trajectory_shape = build_trajectory_footprint_index(
    traj_points, ego_pose, vehicle_info, ego_vel, std::max(ego_accel, 0.0),
    p.nominal_stopping_decel, p.stopping_jerk, p.stop_margin, p.delay_response_time);

  TargetObjects target_objects;
  std::optional<CollisionPoint> collision_point;
  std::string colliding_object_id;
  if (context.predicted_objects && !context.predicted_objects->objects.empty()) {
    const ObjectDecelMap object_decel_map = {
      {ObjectType::CAR, p.rss_params.object_decel.car},
      {ObjectType::TRUCK, p.rss_params.object_decel.truck},
      {ObjectType::BUS, p.rss_params.object_decel.bus},
      {ObjectType::TRAILER, p.rss_params.object_decel.trailer},
      {ObjectType::MOTORCYCLE, p.rss_params.object_decel.motorcycle},
      {ObjectType::BICYCLE, p.rss_params.object_decel.bicycle},
      {ObjectType::PEDESTRIAN, p.rss_params.object_decel.pedestrian}};
    const auto & lm = p.objects.lateral_margin;
    const LateralMarginMap lateral_margin_map = {
      {ObjectType::CAR, lm.car},
      {ObjectType::TRUCK, lm.truck},
      {ObjectType::BUS, lm.bus},
      {ObjectType::TRAILER, lm.trailer},
      {ObjectType::MOTORCYCLE, lm.motorcycle},
      {ObjectType::BICYCLE, lm.bicycle},
      {ObjectType::PEDESTRIAN, lm.pedestrian},
      {ObjectType::HAZARD, lm.hazard},
      {ObjectType::UNKNOWN, lm.unknown}};

    ObjectFilter object_filter(
      p.objects.target_objects.bbox, p.objects.target_objects.polygon,
      p.objects.stopped_velocity_th, p.objects.max_lateral_velocity_th, p.objects.safety_buffer);
    auto filtered_objects = *context.predicted_objects;
    object_filter.filter_objects(filtered_objects);
    target_objects.reserve(filtered_objects.objects.size());
    for (const auto & object : filtered_objects.objects) {
      target_objects.emplace_back(object);
    }
    object_filter.filter_by_target_area(
      target_objects, traj_points, vehicle_info, trajectory_shape, lateral_margin_map);

    collision_point = get_nearest_object_collision(
      target_objects, traj_points, vehicle_info, object_decel_map, p.rss_params.ego_decel,
      p.delay_response_time, p.stop_margin, p.rss_params.safety_margin,
      p.objects.stopped_velocity_th, p.rss_params.lookahead_horizon, p.rss_params.enable);
    if (collision_point) {
      const auto it = std::find_if(
        target_objects.begin(), target_objects.end(),
        [](const TargetObject & object) { return !object.is_safe; });
      if (it != target_objects.end()) {
        colliding_object_id = autoware_utils_uuid::to_hex_string(it->object.object_id);
      }
    }
  }

  update_collision_points_buffer(traj_points, collision_point, now);

  ConstraintGeneratorOutput output;
  if (const auto nearest_collision_point = get_nearest_collision_point()) {
    const auto stop_margin = p.stop_margin + vehicle_info.max_longitudinal_offset_m;
    auto target_stop_point_arc_length = clamp_stop_point_arc_length(
      nearest_collision_point->arc_length - stop_margin, trajectory_shape.trajectory_length,
      ego_vel, ego_accel, p.maximum_stopping_decel, p.stopping_jerk, p.delay_response_time);
    if (target_stop_point_arc_length < p.arrived_distance_threshold) {
      target_stop_point_arc_length = 0.0;
    }

    const auto & path = context.reference_path;
    const double s_gate = std::min(
      s_ego + target_stop_point_arc_length + vehicle_info.max_longitudinal_offset_m, path.length());
    const double half_width = vehicle_info.vehicle_width_m / 2.0;
    const auto left = to_world_pose(path, s_gate, half_width).position;
    const auto right = to_world_pose(path, s_gate, -half_width).position;

    Constraint constraint;
    constraint.certainty = Certainty::DEFINITE;
    constraint.payload = Gate{Segment2d{left, right}};
    constraint.source = Source{get_name(), colliding_object_id, "stop_line"};
    output.constraints.push_back(std::move(constraint));
  }

  output.debug_markers = make_debug_markers(
    trajectory_shape, target_objects, context.odometry.header.stamp, ego_pose.position.z);
  return output;
}

void ObstacleStopConstraintGenerator::update_collision_points_buffer(
  const TrajectoryPoints & traj_points, const std::optional<CollisionPoint> & collision_point,
  const rclcpp::Time & now)
{
  const auto & p = params_.obstacle_stop;
  constexpr double close_distance_threshold = 1.0;

  // prune obsolete collision points
  collision_points_buffer_.erase(
    std::remove_if(
      collision_points_buffer_.begin(), collision_points_buffer_.end(),
      [&](CollisionPoint & cp) {
        if (cp.is_active) return (now - cp.last_seen).seconds() > p.off_time_buffer;
        return !collision_point;
      }),
    collision_points_buffer_.end());

  if (!collision_point) return;

  // find nearest tracked collision
  auto nearest_collision_point_it = collision_points_buffer_.end();
  auto nearest_distance_diff = std::numeric_limits<double>::max();
  for (auto it = collision_points_buffer_.begin(); it != collision_points_buffer_.end(); ++it) {
    const auto estimated_shift = std::abs(it->obstacle_lon_vel) * (now - it->last_seen).seconds();
    it->arc_length = autoware::motion_utils::calcSignedArcLength(traj_points, 0LU, it->point);
    const auto expected_arc_length = it->arc_length + estimated_shift;
    const auto distance_diff = collision_point->arc_length - expected_arc_length;
    if (
      std::abs(distance_diff) > close_distance_threshold ||
      std::abs(distance_diff) > std::abs(nearest_distance_diff))
      continue;
    nearest_collision_point_it = it;
    nearest_distance_diff = distance_diff;
  }

  constexpr double eps = 1e-3;
  if (nearest_collision_point_it == collision_points_buffer_.end()) {
    collision_points_buffer_.emplace_back(*collision_point, now, now, p.on_time_buffer < eps);
    return;
  }

  if ((now - nearest_collision_point_it->first_seen).seconds() > p.on_time_buffer) {
    nearest_collision_point_it->is_active = true;
  }

  nearest_collision_point_it->last_seen = now;
  nearest_collision_point_it->obstacle_lon_vel = collision_point->obstacle_lon_vel;
  nearest_collision_point_it->is_dynamic = collision_point->is_dynamic;
  nearest_collision_point_it->point = collision_point->point;
  nearest_collision_point_it->arc_length = collision_point->arc_length;
}

std::optional<ObstacleStopConstraintGenerator::CollisionPoint>
ObstacleStopConstraintGenerator::get_nearest_collision_point() const
{
  std::optional<CollisionPoint> nearest_collision_point = std::nullopt;
  auto minimum_arc_length = std::numeric_limits<double>::max();
  for (const auto & cp : collision_points_buffer_) {
    if (cp.arc_length >= minimum_arc_length || !cp.is_active) continue;
    nearest_collision_point = cp;
    minimum_arc_length = cp.arc_length;
  }
  return nearest_collision_point;
}

}  // namespace autoware::safety_planner::experimental

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::experimental::ObstacleStopConstraintGenerator,
  autoware::safety_planner::ConstraintGeneratorInterface)
