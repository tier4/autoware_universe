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

#include "surround_obstacle_stop.hpp"

#include "../utils/frenet_utils.hpp"

#include <autoware/trajectory_modifier/trajectory_modifier_utils/utils.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::safety_planner::experimental
{

namespace
{

using autoware::obstacle_proximity_checker::ObstacleTypeParameters;
using autoware::obstacle_proximity_checker::Parameters;
using SurroundObstacleStopParams = autoware::safety_planner::Params::SurroundObstacleStop;

ObstacleTypeParameters to_obstacle_type_parameters(
  const double front_distance, const double side_distance, const double back_distance,
  const bool enable_bbox_check = true, const bool enable_polygon_check = true)
{
  ObstacleTypeParameters parameters;
  parameters.surround_check_front_distance = front_distance;
  parameters.surround_check_side_distance = side_distance;
  parameters.surround_check_back_distance = back_distance;
  parameters.enable_bbox_check = enable_bbox_check;
  parameters.enable_polygon_check = enable_polygon_check;
  return parameters;
}

Parameters to_proximity_checker_parameters(const SurroundObstacleStopParams & p)
{
  Parameters parameters;
  parameters.pointcloud_enable_check = p.use_pointcloud;

  const std::unordered_set<std::string> bbox_enabled_object_types(
    p.target_objects.bbox.begin(), p.target_objects.bbox.end());
  const std::unordered_set<std::string> polygon_enabled_object_types(
    p.target_objects.polygon.begin(), p.target_objects.polygon.end());

  const auto & front = p.front_distance_th;
  const auto & side = p.side_distance_th;
  const auto & back = p.back_distance_th;
  const auto get_object_distance_thresholds = [&](const std::string & label) {
    if (label == "car") return std::make_tuple(front.car, side.car, back.car);
    if (label == "truck") return std::make_tuple(front.truck, side.truck, back.truck);
    if (label == "bus") return std::make_tuple(front.bus, side.bus, back.bus);
    if (label == "trailer") return std::make_tuple(front.trailer, side.trailer, back.trailer);
    if (label == "motorcycle")
      return std::make_tuple(front.motorcycle, side.motorcycle, back.motorcycle);
    if (label == "bicycle") return std::make_tuple(front.bicycle, side.bicycle, back.bicycle);
    if (label == "pedestrian")
      return std::make_tuple(front.pedestrian, side.pedestrian, back.pedestrian);
    if (label == "hazard") return std::make_tuple(front.hazard, side.hazard, back.hazard);
    if (label == "animal") return std::make_tuple(front.animal, side.animal, back.animal);
    return std::make_tuple(front.unknown, side.unknown, back.unknown);
  };

  for (const std::string label :
       {"unknown", "car", "truck", "bus", "trailer", "motorcycle", "bicycle", "pedestrian",
        "hazard", "animal"}) {
    const bool bbox_enabled = bbox_enabled_object_types.count(label) > 0;
    const bool polygon_enabled = polygon_enabled_object_types.count(label) > 0;
    const auto [front, side, back] = get_object_distance_thresholds(label);
    parameters.object_type_enable_check[label] = bbox_enabled || polygon_enabled;
    parameters.obstacle_types_map[label] =
      to_obstacle_type_parameters(front, side, back, bbox_enabled, polygon_enabled);
  }

  parameters.obstacle_types_map["pointcloud"] =
    to_obstacle_type_parameters(front.pointcloud, side.pointcloud, back.pointcloud);

  return parameters;
}

MarkerArray make_debug_markers(
  const obstacle_proximity_checker::CheckResult & result, const bool is_active,
  const rclcpp::Time & stamp)
{
  using autoware_utils_visualization::create_default_marker;
  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;

  MarkerArray marker_array;
  Marker delete_all;
  delete_all.action = Marker::DELETEALL;
  marker_array.markers.push_back(delete_all);

  if (!result.nearest_obstacle) {
    return marker_array;
  }

  const auto create_nearest_obstacle_position_marker = [&]() {
    const auto color =
      is_active ? create_marker_color(1.0, 0.0, 0.0, 1.0) : create_marker_color(1.0, 1.0, 0.0, 1.0);
    auto marker = create_default_marker(
      "map", stamp, "nearest_obstacle_position", 0, Marker::SPHERE,
      create_marker_scale(0.5, 0.5, 0.5), color);
    marker.pose.position = result.nearest_obstacle->nearest_point;
    marker.pose.orientation.w = 1.0;
    return marker;
  };

  const auto create_nearest_obstacle_info_marker = [&]() {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "active: " << (is_active ? "true" : "false") << "\n";
    ss << "distance: " << result.nearest_obstacle->nearest_distance << " m";
    auto marker = create_default_marker(
      "map", stamp, "nearest_obstacle_info", 0, Marker::TEXT_VIEW_FACING,
      create_marker_scale(0.4, 0.4, 0.4), create_marker_color(1.0, 1.0, 1.0, 1.0));
    marker.pose.position = result.nearest_obstacle->nearest_point;
    marker.pose.position.z += 1.0;
    marker.pose.orientation.w = 1.0;
    marker.text = ss.str();
    return marker;
  };

  marker_array.markers.push_back(create_nearest_obstacle_position_marker());
  marker_array.markers.push_back(create_nearest_obstacle_info_marker());

  return marker_array;
}

}  // namespace

ConstraintGeneratorOutput SurroundObstacleStopConstraintGenerator::generate_constraints(
  const PlannerContext & context)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & p = params_.surround_obstacle_stop;
  ConstraintGeneratorOutput output;

  if (!p.enable) {
    reset();
    return output;
  }

  if (!check_inputs(context)) {
    reset();
    return output;
  }

  // Only runs while the ego is (about to be) stopped: moving, a gate at the current position would
  // have to sit behind the ego to hold it, which the trajectory planner cannot use
  if (autoware::trajectory_modifier::utils::is_ego_vehicle_moving(
        context.odometry.twist.twist, p.ego_stopped_vel_th)) {
    reset();
    return output;
  }

  if (!proximity_checker_) {
    proximity_checker_ = std::make_unique<obstacle_proximity_checker::ProximityChecker>(
      to_proximity_checker_parameters(p), context.vehicle_info);
  }

  if (!pointcloud_filter_) {
    pointcloud_filter_ =
      std::make_unique<trajectory_modifier::utils::obstacle_stop::PointCloudFilter>(
        p.target_objects.pointcloud);
  }

  const double contact_distance_threshold =
    last_obstacle_found_time_.has_value() ? p.hysteresis_distance : 1e-3;
  const auto result =
    proximity_checker_->check(to_proximity_checker_inputs(context), contact_distance_threshold);

  const rclcpp::Time now(context.odometry.header.stamp);
  auto obstacle_found = result.is_obstacle_found;
  if (obstacle_found) {
    last_obstacle_found_time_ = now;
  } else if (last_obstacle_found_time_.has_value()) {
    if ((now - *last_obstacle_found_time_).seconds() <= p.hysteresis_time) {
      obstacle_found = true;
    } else {
      reset();
    }
  }

  output.debug_markers = make_debug_markers(result, obstacle_found, now);

  if (!obstacle_found) {
    return output;
  }

  const auto & path = context.reference_path;
  const double s_ego = compute_ego_frenet_state(context).s;
  const double s_gate =
    std::min(s_ego + context.vehicle_info.max_longitudinal_offset_m, path.length());
  const double half_width = context.vehicle_info.vehicle_width_m / 2.0;
  const auto left = to_world_pose(path, s_gate, half_width).position;
  const auto right = to_world_pose(path, s_gate, -half_width).position;

  std::string target_id;
  if (result.nearest_obstacle && !result.nearest_obstacle->is_point_cloud) {
    target_id = autoware_utils_uuid::to_hex_string(result.nearest_obstacle->uuid);
  }

  Constraint constraint;
  constraint.certainty = Certainty::DEFINITE;
  constraint.payload = Gate{Segment2d{left, right}};
  constraint.source = Source{get_name(), target_id, "surround_obstacle"};
  output.constraints.push_back(std::move(constraint));

  return output;
}

bool SurroundObstacleStopConstraintGenerator::check_inputs(const PlannerContext & context) const
{
  const auto & p = params_.surround_obstacle_stop;
  return context.predicted_objects || (p.use_pointcloud && context.obstacle_pointcloud);
}

obstacle_proximity_checker::Inputs
SurroundObstacleStopConstraintGenerator::to_proximity_checker_inputs(
  const PlannerContext & context) const
{
  obstacle_proximity_checker::Inputs inputs;
  inputs.ego_pose = context.odometry.pose.pose;
  inputs.objects = context.predicted_objects;

  if (!context.obstacle_pointcloud || context.obstacle_pointcloud->data.empty()) {
    return inputs;
  }

  auto transformed_pointcloud =
    std::make_shared<trajectory_modifier::utils::obstacle_stop::PointCloud>();
  pcl::fromROSMsg(*context.obstacle_pointcloud, *transformed_pointcloud);

  const auto isometry =
    tf2::transformToEigen(context.obstacle_sensor_to_base_link.transform).cast<float>();
  for (auto & p : transformed_pointcloud->points) {
    const auto q = isometry * Eigen::Vector3f(p.x, p.y, p.z);
    p.x = q.x();
    p.y = q.y();
    p.z = q.z();
  }

  const auto & params = params_.surround_obstacle_stop;

  const auto ego_front_offset = context.vehicle_info.max_longitudinal_offset_m;
  const auto ego_rear_offset = context.vehicle_info.rear_overhang_m;
  const auto ego_side_offset = context.vehicle_info.vehicle_width_m / 2.0;
  const auto front_offset =
    ego_front_offset + params.front_distance_th.pointcloud + params.hysteresis_distance;
  const auto rear_offset =
    ego_rear_offset + params.back_distance_th.pointcloud + params.hysteresis_distance;
  const auto side_offset =
    ego_side_offset + params.side_distance_th.pointcloud + params.hysteresis_distance;

  pointcloud_filter_->filter_pointcloud(
    transformed_pointcloud, -rear_offset, front_offset, -side_offset, side_offset,
    params.pcd_min_height, context.vehicle_info.vehicle_height_m + params.pcd_height_buffer);

  // ProximityChecker expects PointXYZ; drop CPE fields after label/range filtering.
  auto xyz_pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  xyz_pointcloud->points.reserve(transformed_pointcloud->points.size());
  for (const auto & p : transformed_pointcloud->points) {
    xyz_pointcloud->points.emplace_back(p.x, p.y, p.z);
  }
  xyz_pointcloud->width = static_cast<std::uint32_t>(xyz_pointcloud->points.size());
  xyz_pointcloud->height = 1;
  xyz_pointcloud->is_dense = transformed_pointcloud->is_dense;

  inputs.pointcloud_in_base_link = xyz_pointcloud;

  return inputs;
}

void SurroundObstacleStopConstraintGenerator::reset()
{
  last_obstacle_found_time_ = std::nullopt;
}

}  // namespace autoware::safety_planner::experimental

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::experimental::SurroundObstacleStopConstraintGenerator,
  autoware::safety_planner::ConstraintGeneratorInterface)
