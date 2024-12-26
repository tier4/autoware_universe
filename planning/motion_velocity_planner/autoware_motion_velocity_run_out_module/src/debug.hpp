// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#ifndef DEBUG_HPP_
#define DEBUG_HPP_

#include "types.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_velocity_planner_common_universe/velocity_planning_result.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <magic_enum.hpp>
#include <rclcpp/duration.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <iostream>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

inline MarkerArray make_debug_footprint_markers(
  const run_out::TrajectoryCornerFootprint & ego, const std::vector<run_out::Object> & objects)
{
  MarkerArray markers;
  Marker m;
  m.header.frame_id = "map";
  m.type = Marker::LINE_STRIP;
  m.color = universe_utils::createMarkerColor(0.0, 1.0, 0.0, 1.0);
  m.scale.x = 0.2;
  m.ns = "ego_footprint_front_left";
  for (const auto & p : ego.corner_footprint.front_left_ls) {
    m.points.push_back(universe_utils::createPoint(p.x(), p.y(), 0.0));
  }
  markers.markers.push_back(m);
  m.ns = "ego_footprint_front_right";
  m.points.clear();
  for (const auto & p : ego.corner_footprint.front_right_ls) {
    m.points.push_back(universe_utils::createPoint(p.x(), p.y(), 0.0));
  }
  markers.markers.push_back(m);
  m.ns = "ego_footprint_rear_right";
  m.points.clear();
  for (const auto & p : ego.corner_footprint.rear_right_ls) {
    m.points.push_back(universe_utils::createPoint(p.x(), p.y(), 0.0));
  }
  markers.markers.push_back(m);
  m.ns = "ego_footprint_rear_left";
  m.points.clear();
  for (const auto & p : ego.corner_footprint.rear_left_ls) {
    m.points.push_back(universe_utils::createPoint(p.x(), p.y(), 0.0));
  }
  markers.markers.push_back(m);

  m.type = Marker::LINE_LIST;
  m.points.clear();
  m.ns = "objects_footprints";
  m.color.r = 1.0;
  for (const auto & object : objects) {
    for (const auto & footprint : object.corner_footprints) {
      const auto & f = footprint.corner_footprint;
      for (auto i = 0UL; i + 1 < f.front_left_ls.size(); ++i) {
        m.points.push_back(
          universe_utils::createPoint(f.front_left_ls[i].x(), f.front_left_ls[i].y(), 0.0));
        m.points.push_back(
          universe_utils::createPoint(f.front_left_ls[i + 1].x(), f.front_left_ls[i + 1].y(), 0.0));
        m.points.push_back(
          universe_utils::createPoint(f.front_right_ls[i].x(), f.front_right_ls[i].y(), 0.0));
        m.points.push_back(universe_utils::createPoint(
          f.front_right_ls[i + 1].x(), f.front_right_ls[i + 1].y(), 0.0));
        m.points.push_back(
          universe_utils::createPoint(f.rear_left_ls[i].x(), f.rear_left_ls[i].y(), 0.0));
        m.points.push_back(
          universe_utils::createPoint(f.rear_left_ls[i + 1].x(), f.rear_left_ls[i + 1].y(), 0.0));
        m.points.push_back(
          universe_utils::createPoint(f.rear_right_ls[i].x(), f.rear_right_ls[i].y(), 0.0));
        m.points.push_back(
          universe_utils::createPoint(f.rear_right_ls[i + 1].x(), f.rear_right_ls[i + 1].y(), 0.0));
      }
    }
  }
  markers.markers.push_back(m);
  return markers;
}

inline MarkerArray make_debug_object_markers(const std::vector<Object> & objects)
{
  MarkerArray markers;
  Marker m;
  m.header.frame_id = "base_link";
  m.ns = "collisions_table";
  m.type = Marker::TEXT_VIEW_FACING;
  m.color = universe_utils::createMarkerColor(1.0, 1.0, 0.0, 0.75);
  m.scale = universe_utils::createMarkerScale(0.1, 0.0, 0.5);
  std::stringstream ss;
  ss << std::setprecision(4);
  ss << std::fixed;
  ss << std::left;
  ss << std::setw(10) << "object id";
  ss << "|";
  ss << std::setw(10) << "collision";
  ss << "|";
  ss << std::setw(20) << "ego_time_interval";
  ss << "|";
  ss << std::setw(20) << "object_time_interval";
  ss << "\n";
  for (const auto & o : objects) {
    for (const auto & col : o.collisions) {
      ss << std::setw(10) << o.uuid.substr(0, 5) + "|";
      if (col.type == collision) ss << std::setw(10) << "C|";
      if (col.type == pass_first_collision) ss << std::setw(10) << "PC|";
      if (col.type == pass_first_no_collision) ss << std::setw(10) << "P-|";
      if (col.type == no_collision) ss << std::setw(10) << "-|";
      ss << std::setw(20) << col.ego_time_interval << "|";
      ss << std::setw(20) << col.object_time_interval << "|";
      ss << std::setw(20) << col.explanation;
      ss << "\n";
    }
  }
  m.text = ss.str();
  markers.markers.push_back(m);
  m.ns = "collisions_points";
  m.header.frame_id = "map";
  m.type = Marker::POINTS;
  m.color = universe_utils::createMarkerColor(1.0, 0.0, 0.0, 0.75);
  m.scale = universe_utils::createMarkerScale(0.5, 0.5, 0.5);
  for (const auto & o : objects) {
    for (const auto & c : o.collisions) {
      for (const auto & p : {
             c.object_time_interval.first_intersection.intersection,
             c.object_time_interval.last_intersection.intersection,
             c.ego_time_interval.first_intersection.intersection,
             c.ego_time_interval.last_intersection.intersection,
           }) {
        m.points.push_back(universe_utils::createPoint(p.x(), p.y(), 0.0));
      }
    }
  }
  markers.markers.push_back(m);
  return markers;
}

inline MarkerArray make_debug_decisions_markers(const ObjectDecisionsTracker & decisions_tracker)
{
  constexpr auto decision_type_to_str = [](const auto type) {
    switch (type) {
      case stop:
        return "STOP|";
      case slowdown:
        return "SLOW|";
      default:
        return "-|";
    }
  };
  const auto collision_to_str = [](const auto & c) {
    if (!c.has_value()) {
      return "";
    }
    switch (c->type) {
      case collision:
        return "C|";
      case pass_first_collision:
        return "PC|";
      case pass_first_no_collision:
        return "P-|";
      case no_collision:
        return "-|";
    }
  };
  MarkerArray markers;
  Marker m;
  m.header.frame_id = "base_link";
  m.ns = "decisions";
  m.pose.position.x = 10.0;
  m.type = Marker::TEXT_VIEW_FACING;
  m.color = universe_utils::createMarkerColor(1.0, 1.0, 0.0, 0.75);
  m.scale = universe_utils::createMarkerScale(0.1, 0.0, 0.5);
  std::stringstream ss;
  ss << std::setprecision(2);
  ss << std::fixed;
  ss << std::left;
  ss << std::setw(10) << "object uuid |";
  ss << std::setw(60) << " decisions |\n";
  ss << " |\n";
  for (const auto & [object, history] : decisions_tracker.history_per_object) {
    ss << std::setw(10) << object.substr(0, 5) + "|";
    for (auto i = 0UL; i < history.times.size(); ++i) {
      std::stringstream line;
      line << std::fixed;
      line << std::setprecision(2);
      line << history.times[i] << ":";
      line << decision_type_to_str(history.decisions[i].type);
      line << collision_to_str(history.decisions[i].collision);
      ss << line.str();
    }
    ss << " | " << history.decisions.back().explanation;
    ss << "\n";
  }
  m.text = ss.str();
  std::cout << ss.str() << std::endl;
  markers.markers.push_back(m);

  return markers;
}

inline motion_utils::VirtualWalls create_virtual_walls(
  const VelocityPlanningResult & result,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const double front_offset)
{
  motion_utils::VirtualWalls virtual_walls;
  motion_utils::VirtualWall wall;
  wall.text = "run_out";
  wall.longitudinal_offset = front_offset;
  for (const auto & stop_point : result.stop_points) {
    wall.style = motion_utils::VirtualWallType::stop;
    const auto length = motion_utils::calcSignedArcLength(trajectory, 0, stop_point);
    wall.pose = motion_utils::calcInterpolatedPose(trajectory, length);
    virtual_walls.push_back(wall);
  }
  for (const auto & slowdown : result.slowdown_intervals) {
    wall.style = motion_utils::VirtualWallType::slowdown;
    const auto length = motion_utils::calcSignedArcLength(trajectory, 0, slowdown.from);
    wall.pose = motion_utils::calcInterpolatedPose(trajectory, length);
    wall.pose.position = slowdown.from;
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}

inline MarkerArray make_debug_min_stop_marker(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const double time_to_stop)
{
  MarkerArray markers;
  Marker m;
  m.header.frame_id = "map";
  m.ns = "min_stop";
  m.type = Marker::SPHERE;
  m.color = universe_utils::createMarkerColor(1.0, 0.0, 0.0, 0.75);
  m.scale = universe_utils::createMarkerScale(0.5, 0.5, 0.5);

  const auto it_after_time = std::find_if(
    trajectory.begin(), trajectory.end(),
    [&](const autoware_planning_msgs::msg::TrajectoryPoint & p) {
      return rclcpp::Duration(p.time_from_start).seconds() > time_to_stop;
    });
  if (it_after_time == trajectory.begin()) {
    m.pose = it_after_time->pose;
  } else {
    const auto it_before_time = std::prev(it_after_time);
    const auto delta = (rclcpp::Duration(it_after_time->time_from_start) -
                        rclcpp::Duration(it_before_time->time_from_start))
                         .seconds();
    const auto diff = time_to_stop - rclcpp::Duration(it_before_time->time_from_start).seconds();
    const auto ratio = diff / delta;
    m.pose = universe_utils::calcInterpolatedPose(it_before_time->pose, it_after_time->pose, ratio);
  }
  markers.markers.push_back(m);
  return markers;
}
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // DEBUG_HPP_
