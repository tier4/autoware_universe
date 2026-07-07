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
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware_utils/ros/marker_helper.hpp>


#include <boost/geometry/algorithms/intersection.hpp>

#include <lanelet2_core/geometry/LineString.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{
using autoware_utils_geometry::Line2d;

bool is_signaled_crosswalk(const lanelet::CrosswalkConstPtr & crosswalk_reg_elem)
{
  const auto & crosswalk_lanelet = crosswalk_reg_elem->crosswalkLanelet();
  return !crosswalk_lanelet.regulatoryElementsAs<const lanelet::TrafficLight>().empty();
}

std::vector<lanelet::CrosswalkConstPtr> collect_crosswalk_reg_elems_from_route(
  const lanelet::LaneletMap & lanelet_map,
  const autoware_planning_msgs::msg::LaneletRoute & route)
{
  std::vector<lanelet::CrosswalkConstPtr> crosswalks_on_route;
  std::unordered_set<lanelet::Id> seen_crosswalk_ids;

  for (const auto & segment : route.segments) {
    for (const auto & cw_reg_elem : lanelet_map.laneletLayer.get(segment.preferred_primitive.id)
                             .regulatoryElementsAs<lanelet::autoware::Crosswalk>()) {
      const auto crosswalk_id = cw_reg_elem->id();
      if (!seen_crosswalk_ids.insert(crosswalk_id).second) {
        continue;
      }
      if (is_signaled_crosswalk(cw_reg_elem)) {
        continue;
      }
      crosswalks_on_route.push_back(cw_reg_elem);
    }
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

  if (crosswalks_on_route.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("crosswalk_filter"), "No crosswalks on route found");
    return target_crosswalks;
  }

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

  if (intersecting_crosswalks.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("crosswalk_filter"), "No intersecting crosswalks found");
    return target_crosswalks;
  }

  for (const auto & cw : intersecting_crosswalks) {
    auto crosswalk_polygon = cw.crosswalk->crosswalkLanelet().polygon2d().basicPolygon();
    target_crosswalks.emplace_back(cw, crosswalk_polygon);
  }

  return target_crosswalks;
}

void CrosswalkFilter::update_debug_data(
  const std::vector<TargetCrosswalk> & target_crosswalks,
  const rclcpp::Time & current_time, const double z)
{
  using visualization_msgs::msg::Marker;
  debug_markers_.markers.clear();

  auto add_polygon_marker = [&](
                              const lanelet::BasicPolygon2d & polygon,
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

  int id = 0;
  const auto magenta = autoware_utils::create_marker_color(1.0, 0.0, 1.0, 1.0);
  for (const auto & cw : target_crosswalks) {
    add_polygon_marker(cw.crosswalk_polygon, "target_crosswalks", id, magenta);
    id++;
  }
}

}  // namespace autoware::trajectory_validator::plugin::traffic_rule

#include <pluginlib/class_list_macros.hpp>
namespace traffic_rule = autoware::trajectory_validator::plugin::traffic_rule;
PLUGINLIB_EXPORT_CLASS(
  traffic_rule::CrosswalkFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
