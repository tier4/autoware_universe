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

#include "autoware/trajectory_validator/filters/traffic_rule/crosswalk_filter.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/Forward.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware_utils/ros/marker_helper.hpp>


#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{
using autoware_utils_geometry::Line2d;
using autoware_perception_msgs::msg::ObjectClassification;

ObjectClassification::_label_type to_classification_label(const std::string & label_str)
{
  static const std::unordered_map<std::string, ObjectClassification::_label_type> string_to_classification_map = {
    {"unknown", ObjectClassification::UNKNOWN},
    {"car", ObjectClassification::CAR},
    {"truck", ObjectClassification::TRUCK},
    {"bus", ObjectClassification::BUS},
    {"trailer", ObjectClassification::TRAILER},
    {"motorcycle", ObjectClassification::MOTORCYCLE},
    {"bicycle", ObjectClassification::BICYCLE},
    {"pedestrian", ObjectClassification::PEDESTRIAN},
    {"animal", ObjectClassification::ANIMAL},
    {"hazard", ObjectClassification::HAZARD}};

  const auto it = string_to_classification_map.find(label_str);
  if (it == string_to_classification_map.end()) {
    return ObjectClassification::UNKNOWN;
  }
  return it->second;
}

bool is_signaled_crosswalk(const lanelet::CrosswalkConstPtr & crosswalk_reg_elem)
{
  const auto & crosswalk_lanelet = crosswalk_reg_elem->crosswalkLanelet();
  return !crosswalk_lanelet.regulatoryElementsAs<const lanelet::TrafficLight>().empty();
}

std::vector<lanelet::CrosswalkConstPtr> collect_crosswalk_reg_elems_from_route(
  const lanelet::LaneletMap & lanelet_map,
  const autoware_planning_msgs::msg::LaneletRoute & route)
{
  lanelet::ConstLanelets route_lanelets;
  for (const auto & segment : route.segments) {
    const auto ll = lanelet_map.laneletLayer.get(segment.preferred_primitive.id);
    route_lanelets.push_back(ll);
  }

  std::vector<lanelet::CrosswalkConstPtr> crosswalks_on_route;
  for (const auto & cw : lanelet::utils::query::crosswalks(route_lanelets)) {
    if (is_signaled_crosswalk(cw)) {
      continue;
    }
    crosswalks_on_route.push_back(cw);
  }
  return crosswalks_on_route;
}

using autoware::trajectory_validator::plugin::traffic_rule::CrosswalkOnTrajectory;
std::vector<CrosswalkOnTrajectory> filter_crosswalks_intersecting_trajectory(
  const std::vector<lanelet::CrosswalkConstPtr> & crosswalks_on_route,
  const lanelet::BasicLineString2d trajectory_ls)
{
  std::vector<CrosswalkOnTrajectory> crosswalks_on_trajectory;

  if (trajectory_ls.size() < 2 || crosswalks_on_route.empty()) {
    return crosswalks_on_trajectory;
  }

  std::unordered_set<lanelet::Id> seen_crosswalk_ids;
  auto checked_length = 0.0;

  auto process_cw = [&](const lanelet::CrosswalkConstPtr & cw, const lanelet::BasicLineString2d & traj_seg) {
    auto stop_lines = cw->stopLines();
    if (stop_lines.empty()) return;

    auto distance_to_stop_line = std::numeric_limits<double>::max();
    std::optional<lanelet::BasicPoints2d> stop_line;
    for (const auto & sl : stop_lines) {
      const auto stop_line_2d = lanelet::utils::to2D(sl).basicLineString();
      auto distance = 0.0;
      lanelet::BasicPoints2d intersection_points;
      boost::geometry::intersection(traj_seg, stop_line_2d, intersection_points);
      if (!intersection_points.empty()) {
        distance += static_cast<double>(
          boost::geometry::distance(traj_seg.front(), intersection_points.front()));
        if (distance < distance_to_stop_line) {
          distance_to_stop_line = checked_length + distance;
          stop_line = stop_line_2d;
        }
      }
    }
    if (stop_line) {
      crosswalks_on_trajectory.emplace_back(cw, distance_to_stop_line, *stop_line);
      seen_crosswalk_ids.insert(cw->id());
    }
  };

  for (size_t i = 0; i + 1 < trajectory_ls.size(); ++i) {
    const lanelet::BasicLineString2d segment{trajectory_ls[i], trajectory_ls[i + 1]};

    for (const auto & cw : crosswalks_on_route) {
      if (seen_crosswalk_ids.count(cw->id())) continue;
      process_cw(cw, segment);
    }
    checked_length += static_cast<double>(boost::geometry::length(segment));
  }

  return crosswalks_on_trajectory;
}

}  // namespace

namespace autoware::trajectory_validator::plugin::traffic_rule
{

CrosswalkFilter::CrosswalkFilter() : ValidatorInterface("crosswalk_filter")
{
}

void CrosswalkFilter::update_parameters(const validator::Params & params)
{
  params_ = params.crosswalk;
  object_types_.clear();
  for (const auto & object_type_string : params_.object_types) {
    object_types_.insert(to_classification_label(object_type_string));
  }
}

void CrosswalkFilter::set_vehicle_info(const VehicleInfo & vehicle_info)
{
  ValidatorInterface::set_vehicle_info(vehicle_info);
}

CrosswalkFilter::result_t CrosswalkFilter::is_feasible(
  const CandidateTrajectory & candidate_trajectory, const FilterContext & context)
{
  if (!context.lanelet_map) {
    return tl::make_unexpected("Lanelet map is not available in the context.");
  }

  if (!context.route) {
    return tl::make_unexpected("Route is not available in the context.");
  }

  const auto target_crosswalks = get_target_crosswalks(candidate_trajectory.points, context);

  std::vector<MetricReport> metrics;
  if (target_crosswalks.empty()) return ValidationResult{true, std::move(metrics)};

  update_target_objects(context, target_crosswalks);

  update_debug_data(target_crosswalks, context.odometry->header.stamp, context.odometry->pose.pose.position.z);

  return ValidationResult{true, std::move(metrics)};
}

std::vector<TargetCrosswalk> CrosswalkFilter::get_target_crosswalks(const TrajectoryPoints & traj_points, const FilterContext & context)
{
  std::vector<TargetCrosswalk> target_crosswalks;

  if (traj_points.size() < 2) return target_crosswalks;

  constexpr double zero_vel_threshold = 0.1;
  const auto start_move_it = std::find_if(traj_points.begin(), traj_points.end(), [&](const auto & p) {
    return p.longitudinal_velocity_mps > zero_vel_threshold;
  });

  // skip check for non-moving trajectory
  if (start_move_it == traj_points.end()) return target_crosswalks;

  // skip check if ego is stopping for sufficient time
  const auto start_move_time = rclcpp::Duration(start_move_it->time_from_start).seconds();
  if (start_move_time > 3.0) return target_crosswalks;

  const auto crosswalks_on_route = collect_crosswalk_reg_elems_from_route(
    *context.lanelet_map, *context.route);

  if (crosswalks_on_route.empty()) return target_crosswalks;

  const double current_vel = context.odometry->twist.twist.linear.x;
  const double current_acc = context.acceleration->accel.accel.linear.x;
  const auto decel_limit = 1.0;
  const auto jerk_limit = 1.0;
  
  auto stop_distance = autoware::motion_utils::calculate_stop_distance(current_vel, current_acc, decel_limit, jerk_limit);
  if (!stop_distance) stop_distance = std::numeric_limits<double>::max();
  const auto lookahead_distance_m = *stop_distance;

  lanelet::BasicLineString2d trajectory_ls;
  double length = 0.0;

  for (const auto & p : traj_points) {
    // skip points behind ego
    if (rclcpp::Duration(p.time_from_start).seconds() < 0.0) {
      continue;
    }

    const lanelet::BasicPoint2d lanelet_p(p.pose.position.x, p.pose.position.y);
    if (!trajectory_ls.empty()) {
      length += lanelet::geometry::distance2d(trajectory_ls.back(), lanelet_p);
    }
    trajectory_ls.emplace_back(lanelet_p);

    // skip points beyond the first stop, or skip once we reach the maximum length
    if (p.longitudinal_velocity_mps <= 1e-6 || length > lookahead_distance_m) {
      break;
    }
  }

  constexpr double overshoot_tolerance_m = 0.1;
  const auto longitudinal_offset_m = vehicle_info_ptr_->max_longitudinal_offset_m - overshoot_tolerance_m;

  if (longitudinal_offset_m > 0.0) {
    // extend the trajectory linestring by the vehicle's longitudinal offset
    const auto offset_pose = autoware_utils_geometry::calc_offset_pose(
      traj_points.back().pose, longitudinal_offset_m, 0.0, 0.0);
    const lanelet::BasicPoint2d offset_point(offset_pose.position.x, offset_pose.position.y);
    trajectory_ls.emplace_back(offset_point);
  }

  const auto intersecting_crosswalks = filter_crosswalks_intersecting_trajectory(crosswalks_on_route, trajectory_ls);

  if (intersecting_crosswalks.empty()) return target_crosswalks;

  for (const auto & cw : intersecting_crosswalks) {
    auto crosswalk_polygon = cw.crosswalk->crosswalkLanelet().polygon2d().basicPolygon();
    target_crosswalks.emplace_back(cw, crosswalk_polygon);
  }

  return target_crosswalks;
}

void CrosswalkFilter::update_target_objects(const FilterContext & context, const TargetCrosswalks & target_crosswalks)
{
  auto objects = context.predicted_objects->objects;

  if (objects.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("CrosswalkFilter"), "No objects in the context.");
    return;
  }

  const auto current_time = rclcpp::Time(context.odometry->header.stamp);

  objects.erase(
    std::remove_if(
      objects.begin(), objects.end(),
      [&](const auto & object) {
        auto label = object.classification.empty() ? ObjectClassification::UNKNOWN : object.classification.front().label;
        return object_types_.count(label) == 0;
      }),
    objects.end());

  if (objects.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("CrosswalkFilter"), "No objects after filtering by type.");
    return;
  }

  auto update_object = [&](const PredictedObject & obj, const auto cw_id) {
    if (crosswalk_objects_map_.count(cw_id) == 0) {
      crosswalk_objects_map_[cw_id] = TargetObjects{};
    }
    auto cw_objects = crosswalk_objects_map_[cw_id];
    if (cw_objects.empty()) {
      cw_objects.emplace_back(TargetObject{obj, current_time, current_time});
      return;
    }
    const auto it = std::find_if(cw_objects.begin(), cw_objects.end(), [&](const auto & cw_object) {
      return cw_object.object.object_id.uuid == obj.object_id.uuid;
    });
    if (it != cw_objects.end()) {
      it->object = obj;
      it->last_seen_time = current_time;
    } else {
      cw_objects.emplace_back(TargetObject{obj, current_time, current_time});
    }
  };

  // auto clear_old_objects = [&](const auto cw_id) {
  //   if (crosswalk_objects_map_.count(cw_id) == 0) return;
  //   auto & cw_objects = crosswalk_objects_map_[cw_id];
  //   cw_objects.erase(std::remove_if(cw_objects.begin(), cw_objects.end(), [&](const auto & cw_object) {
  //     return current_time - cw_object.last_seen_time > rclcpp::Duration::from_seconds(0.5);
  //   }), cw_objects.end());
  // };

  for (const auto & cw : target_crosswalks) {
    auto cw_polygon = cw.crosswalk_polygon;
    for (const auto & object : objects) {
      const auto obj_position = object.kinematics.initial_pose_with_covariance.pose.position;
      lanelet::BasicPoint2d obj_point(obj_position.x, obj_position.y);
      const double dist = lanelet::geometry::distance(cw_polygon, obj_point);
      RCLCPP_INFO(rclcpp::get_logger("CrosswalkFilter"), "Object distance to crosswalk: %f", dist);
      if (dist < 0.0) continue;
      if (dist > params_.object_distance_th) continue;
      update_object(object, cw.crosswalk_info.crosswalk->id());
      RCLCPP_INFO(rclcpp::get_logger("CrosswalkFilter"), "Object updated for crosswalk: %ld", cw.crosswalk_info.crosswalk->id());
    }
    // clear_old_objects(cw.crosswalk_info.crosswalk->id());
  }
}

void CrosswalkFilter::update_debug_data(
  const std::vector<TargetCrosswalk> & target_crosswalks,
  const rclcpp::Time & current_time, const double z)
{
  using visualization_msgs::msg::Marker;
  debug_markers_.markers.clear();

  auto add_polygon_marker = [&](
                              const auto & polygon,
                              const std::string & ns, const int id,
                              const std_msgs::msg::ColorRGBA & color) {
    visualization_msgs::msg::Marker marker = autoware_utils::create_default_marker(
      "map", current_time, ns, id, Marker::LINE_STRIP,
      autoware_utils::create_marker_scale(0.1, 0.1, 0.1), color);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    for (const auto & p : polygon) {
      marker.points.push_back(autoware_utils_geometry::create_point(p.x(), p.y(), z));
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    debug_markers_.markers.push_back(marker);
  };

  auto add_line_marker = [&](
    const lanelet::BasicLineString2d & line,
    const std::string & ns, const int id,
    const std_msgs::msg::ColorRGBA & color) {
    visualization_msgs::msg::Marker marker = autoware_utils::create_default_marker(
      "map", current_time, ns, id, Marker::LINE_STRIP,
      autoware_utils::create_marker_scale(0.15, 0.15, 0.15), color);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    for (const auto & p : line) {
      marker.points.push_back(autoware_utils_geometry::create_point(p.x(), p.y(), z));
    }
    debug_markers_.markers.push_back(marker);
  };

  int id = 0;
  const auto magenta = autoware_utils::create_marker_color(1.0, 0.0, 1.0, 1.0);
  const auto yellow = autoware_utils::create_marker_color(1.0, 1.0, 0.0, 1.0);
  for (const auto & cw : target_crosswalks) {
    add_polygon_marker(cw.crosswalk_polygon, "target_crosswalks", id, magenta);
    add_line_marker(cw.crosswalk_info.stop_line, "target_stop_lines", id, magenta);
    int obj_id = 0;
    if (crosswalk_objects_map_.count(cw.crosswalk_info.crosswalk->id())) {
      for (const auto & cw_object : crosswalk_objects_map_[cw.crosswalk_info.crosswalk->id()]) {
        auto obj_polygon = autoware_utils_geometry::to_polygon2d(cw_object.object.kinematics.initial_pose_with_covariance.pose, cw_object.object.shape);
        add_polygon_marker(obj_polygon.outer(), "target_objects", obj_id, yellow);
        obj_id++;
      }
    }
    id++;
  }
}

}  // namespace autoware::trajectory_validator::plugin::traffic_rule

#include <pluginlib/class_list_macros.hpp>
namespace traffic_rule = autoware::trajectory_validator::plugin::traffic_rule;
PLUGINLIB_EXPORT_CLASS(
  traffic_rule::CrosswalkFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
