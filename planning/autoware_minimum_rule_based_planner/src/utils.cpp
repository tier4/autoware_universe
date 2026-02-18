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

#include "utils.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/trajectory/interpolator/linear.hpp>
#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/trajectory/utils/crop.hpp>
#include <autoware/trajectory/utils/find_intervals.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <limits>
#include <set>
#include <vector>

namespace autoware::minimum_rule_based_planner::utils
{
namespace
{
template <typename T>
bool exists(const std::vector<T> & vec, const T & item)
{
  return std::find(vec.begin(), vec.end(), item) != vec.end();
}

template <typename const_iterator>
std::vector<geometry_msgs::msg::Point> to_geometry_msgs_points(
  const const_iterator begin, const const_iterator end)
{
  std::vector<geometry_msgs::msg::Point> geometry_msgs_points{};
  geometry_msgs_points.reserve(std::distance(begin, end));
  std::transform(begin, end, std::back_inserter(geometry_msgs_points), [](const auto & point) {
    return lanelet::utils::conversion::toGeomMsgPt(point);
  });
  return geometry_msgs_points;
}

lanelet::BasicPoints3d to_lanelet_points(
  const std::vector<geometry_msgs::msg::Point> & geometry_msgs_points)
{
  lanelet::BasicPoints3d lanelet_points{};
  lanelet_points.reserve(geometry_msgs_points.size());
  std::transform(
    geometry_msgs_points.begin(), geometry_msgs_points.end(), std::back_inserter(lanelet_points),
    [](const auto & point) { return lanelet::utils::conversion::toLaneletPoint(point); });
  return lanelet_points;
}
}  // namespace

std::optional<lanelet::ConstLanelets> get_lanelets_within_route_up_to(
  const lanelet::ConstLanelet & lanelet, const RouteContext & planner_data, const double distance)
{
  if (!exists(planner_data.route_lanelets, lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelets lanelets{};
  auto current_lanelet = lanelet;
  auto length = 0.;

  while (length < distance) {
    const auto prev_lanelet = get_previous_lanelet_within_route(current_lanelet, planner_data);
    if (!prev_lanelet) {
      break;
    }

    lanelets.push_back(*prev_lanelet);
    current_lanelet = *prev_lanelet;
    length += lanelet::utils::getLaneletLength2d(*prev_lanelet);
  }

  std::reverse(lanelets.begin(), lanelets.end());
  return lanelets;
}

std::optional<lanelet::ConstLanelets> get_lanelets_within_route_after(
  const lanelet::ConstLanelet & lanelet, const RouteContext & planner_data, const double distance)
{
  if (!exists(planner_data.route_lanelets, lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelets lanelets{};
  auto current_lanelet = lanelet;
  auto length = 0.;

  while (length < distance) {
    const auto next_lanelet = get_next_lanelet_within_route(current_lanelet, planner_data);
    if (!next_lanelet) {
      break;
    }

    lanelets.push_back(*next_lanelet);
    current_lanelet = *next_lanelet;
    length += lanelet::utils::getLaneletLength2d(*next_lanelet);
  }

  return lanelets;
}

std::optional<lanelet::ConstLanelet> get_previous_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const RouteContext & planner_data)
{
  if (exists(planner_data.start_lanelets, lanelet)) {
    return std::nullopt;
  }

  const auto prev_lanelets = planner_data.routing_graph_ptr->previous(lanelet);
  if (prev_lanelets.empty()) {
    return std::nullopt;
  }

  const auto prev_lanelet_itr = std::find_if(
    prev_lanelets.cbegin(), prev_lanelets.cend(),
    [&](const lanelet::ConstLanelet & l) { return exists(planner_data.route_lanelets, l); });
  if (prev_lanelet_itr == prev_lanelets.cend()) {
    return std::nullopt;
  }
  return *prev_lanelet_itr;
}

std::optional<lanelet::ConstLanelet> get_next_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const RouteContext & planner_data)
{
  if (planner_data.preferred_lanelets.empty()) {
    return std::nullopt;
  }

  if (exists(planner_data.goal_lanelets, lanelet)) {
    return std::nullopt;
  }

  const auto next_lanelets = planner_data.routing_graph_ptr->following(lanelet);
  if (
    next_lanelets.empty() ||
    next_lanelets.front().id() == planner_data.preferred_lanelets.front().id()) {
    return std::nullopt;
  }

  // // 1. Check if any following lanelet is preferred
  // for (const auto & next : next_lanelets) {
  //   if (exists(planner_data.preferred_lanelets, next)) {
  //     RCLCPP_INFO(rclcpp::get_logger(""), "next: %ld", next.id());
  //     return next;
  //   }
  // }

  // // 2. Check if a preferred lanelet is adjacent to any following route lanelet
  // for (const auto & next : next_lanelets) {
  //   if (!exists(planner_data.route_lanelets, next)) {
  //     continue;
  //   }
  //   const auto & rg = planner_data.routing_graph_ptr;
  //   for (const auto & beside : rg->besides(next)) {
  //     if (beside.id() != next.id() && exists(planner_data.preferred_lanelets, beside)) {
  //       RCLCPP_INFO(rclcpp::get_logger(""), "beside: %ld", next.id());
  //       return beside;
  //     }
  //   }
  // }

  // 3. Fallback: any route lanelet
  const auto next_lanelet_itr = std::find_if(
    next_lanelets.cbegin(), next_lanelets.cend(),
    [&](const lanelet::ConstLanelet & l) { return exists(planner_data.route_lanelets, l); });
  if (next_lanelet_itr == next_lanelets.cend()) {
    return std::nullopt;
  }
  return *next_lanelet_itr;
}

PathWithLaneId generate_centerline_path(
  const lanelet::ConstLanelets & lanelets, const RouteContext & planner_data,
  const geometry_msgs::msg::Pose & current_pose, const double s_start, const double s_end)
{
  PathWithLaneId path;
  path.header.frame_id = planner_data.route_frame_id;

  if (lanelets.empty()) {
    return path;
  }

  const lanelet::LaneletSequence lanelet_sequence(lanelets);
  const auto & centerline = lanelet_sequence.centerline();

  // Calculate arc length along centerline
  double accumulated_length = 0.0;
  std::vector<std::pair<double, lanelet::ConstPoint3d>> points_with_arc_length;

  for (size_t i = 0; i < centerline.size(); ++i) {
    if (i > 0) {
      accumulated_length += lanelet::geometry::distance2d(centerline[i - 1], centerline[i]);
    }
    points_with_arc_length.emplace_back(accumulated_length, centerline[i]);
  }

  // Find lanelet id for each point
  auto get_lanelet_id = [&](const lanelet::ConstPoint3d & point) -> lanelet::Id {
    for (const auto & lanelet : lanelets) {
      if (lanelet::geometry::inside(lanelet, lanelet::utils::to2D(point).basicPoint())) {
        return lanelet.id();
      }
    }
    return lanelets.front().id();
  };

  // Generate path points within s_start to s_end
  for (const auto & [arc_length, point] : points_with_arc_length) {
    if (arc_length < s_start) {
      continue;
    }
    if (arc_length > s_end) {
      break;
    }

    PathPointWithLaneId path_point;
    path_point.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(point);

    // Get speed limit from lanelet
    const auto lanelet_id = get_lanelet_id(point);
    path_point.lane_ids.push_back(lanelet_id);

    const auto & lanelet = planner_data.lanelet_map_ptr->laneletLayer.get(lanelet_id);
    const auto speed_limit = planner_data.traffic_rules_ptr->speedLimit(lanelet);
    path_point.point.longitudinal_velocity_mps = speed_limit.speedLimit.value();

    path.points.push_back(path_point);
  }

  // Set orientation based on path direction
  for (size_t i = 0; i < path.points.size(); ++i) {
    if (i + 1 < path.points.size()) {
      const auto & p1 = path.points[i].point.pose.position;
      const auto & p2 = path.points[i + 1].point.pose.position;
      const double yaw = std::atan2(p2.y - p1.y, p2.x - p1.x);
      path.points[i].point.pose.orientation.x = 0.0;
      path.points[i].point.pose.orientation.y = 0.0;
      path.points[i].point.pose.orientation.z = std::sin(yaw / 2.0);
      path.points[i].point.pose.orientation.w = std::cos(yaw / 2.0);
    } else if (i > 0) {
      // Last point: use same orientation as previous
      path.points[i].point.pose.orientation = path.points[i - 1].point.pose.orientation;
    } else {
      // Single point: use current pose orientation
      path.points[i].point.pose.orientation = current_pose.orientation;
    }
  }

  return path;
}

std::vector<WaypointGroup> get_waypoint_groups(
  const lanelet::LaneletSequence & lanelet_sequence, const lanelet::LaneletMap & lanelet_map,
  const double group_separation_threshold, const double interval_margin_ratio)
{
  std::vector<WaypointGroup> waypoint_groups{};

  const auto get_interval_bound =
    [&](const lanelet::ConstPoint3d & point, const double lateral_distance_factor) {
      const auto arc_coordinates = lanelet::geometry::toArcCoordinates(
        lanelet_sequence.centerline2d(), lanelet::utils::to2D(point));
      return arc_coordinates.length + lateral_distance_factor * std::abs(arc_coordinates.distance);
    };

  for (const auto & lanelet : lanelet_sequence) {
    if (!lanelet.hasAttribute("waypoints")) {
      continue;
    }

    const auto waypoints_id = lanelet.attribute("waypoints").asId().value();
    const auto & waypoints = lanelet_map.lineStringLayer.get(waypoints_id);

    if (
      waypoint_groups.empty() || lanelet::geometry::distance2d(
                                   waypoint_groups.back().waypoints.back().point,
                                   waypoints.front()) > group_separation_threshold) {
      waypoint_groups.emplace_back().interval.start =
        get_interval_bound(waypoints.front(), -interval_margin_ratio);
    }
    waypoint_groups.back().interval.end =
      get_interval_bound(waypoints.back(), interval_margin_ratio);

    std::transform(
      waypoints.begin(), waypoints.end(), std::back_inserter(waypoint_groups.back().waypoints),
      [&](const lanelet::ConstPoint3d & waypoint) {
        return WaypointGroup::Waypoint{waypoint, lanelet.id()};
      });
  }

  return waypoint_groups;
}

std::optional<double> get_first_intersection_arc_length(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end,
  const double vehicle_length)
{
  if (lanelet_sequence.empty()) {
    return std::nullopt;
  }

  std::optional<double> s_intersection;

  const auto s_start_on_bounds = get_arc_length_on_bounds(lanelet_sequence, s_start);
  const auto s_end_on_bounds = get_arc_length_on_bounds(lanelet_sequence, s_end);

  const auto crop_and_convert = [](const auto & line, double s0, double s1) {
    return lanelet::utils::to2D(to_lanelet_points(
      crop_line_string(to_geometry_msgs_points(line.begin(), line.end()), s0, s1)));
  };

  const auto cropped_centerline = crop_and_convert(lanelet_sequence.centerline2d(), s_start, s_end);
  const auto cropped_left_bound =
    crop_and_convert(lanelet_sequence.leftBound2d(), s_start_on_bounds.left, s_end_on_bounds.left);
  const auto cropped_right_bound = crop_and_convert(
    lanelet_sequence.rightBound2d(), s_start_on_bounds.right, s_end_on_bounds.right);

  if (cropped_centerline.empty() || cropped_left_bound.empty() || cropped_right_bound.empty()) {
    return std::nullopt;
  }

  const lanelet::BasicLineString2d start_edge{
    cropped_left_bound.front(), cropped_right_bound.front()};

  // Helper: offset an optional arc length by a base value
  auto offset_optional = [](std::optional<double> s, double base) -> std::optional<double> {
    if (s) *s += base;
    return s;
  };

  // Self intersection of bounds
  {
    const auto s_left_bound = offset_optional(
      get_first_self_intersection_arc_length(cropped_left_bound), s_start_on_bounds.left);
    const auto s_right_bound = offset_optional(
      get_first_self_intersection_arc_length(cropped_right_bound), s_start_on_bounds.right);

    const auto [s_left, s_right] =
      get_arc_length_on_centerline(lanelet_sequence, s_left_bound, s_right_bound);

    if (s_left && s_right) {
      s_intersection = std::min(s_left, s_right);
    } else {
      s_intersection = s_left ? s_left : s_right;
    }
  }

  // intersection between left and right bounds
  {
    lanelet::BasicPoints2d intersections;
    boost::geometry::intersection(cropped_left_bound, cropped_right_bound, intersections);
    for (const auto & intersection : intersections) {
      const auto s_on_centerline = get_arc_length_on_centerline(
        lanelet_sequence,
        s_start_on_bounds.left +
          lanelet::geometry::toArcCoordinates(cropped_left_bound, intersection).length,
        s_start_on_bounds.right +
          lanelet::geometry::toArcCoordinates(cropped_right_bound, intersection).length);
      const auto s_mutual = [&]() -> std::optional<double> {
        if (s_on_centerline.left && s_on_centerline.right) {
          return std::max(s_on_centerline.left, s_on_centerline.right);
        }
        return s_on_centerline.left ? s_on_centerline.left : s_on_centerline.right;
      }();
      if (s_intersection && s_mutual) {
        s_intersection = std::min(s_intersection, s_mutual);
      } else if (s_mutual) {
        s_intersection = s_mutual;
      }
    }
  }

  // intersection between start edge of drivable area and left / right bound
  {
    const auto get_start_edge_intersection_arc_length =
      [&](const lanelet::BasicLineString2d & bound) {
        std::optional<double> s_start_edge = std::nullopt;
        if (bound.size() <= 2) {
          return s_start_edge;
        }
        lanelet::BasicPoints2d start_edge_intersections;
        boost::geometry::intersection(start_edge, bound, start_edge_intersections);
        for (const auto & intersection : start_edge_intersections) {
          if (boost::geometry::equals(intersection, bound.front())) {
            continue;
          }
          const auto s = lanelet::geometry::toArcCoordinates(bound, intersection).length;
          s_start_edge = s_start_edge ? std::min(*s_start_edge, s) : s;
        }
        return s_start_edge;
      };
    const auto s_left_bound = [&]() {
      auto s = get_start_edge_intersection_arc_length(cropped_left_bound);
      if (s) {
        *s += s_start_on_bounds.left;
      }
      return s;
    }();
    const auto s_right_bound = [&]() {
      auto s = get_start_edge_intersection_arc_length(cropped_right_bound);
      if (s) {
        *s += s_start_on_bounds.right;
      }
      return s;
    }();

    const auto s_on_centerline =
      get_arc_length_on_centerline(lanelet_sequence, s_left_bound, s_right_bound);

    const auto s_start_edge = [&]() {
      if (s_on_centerline.left && s_on_centerline.right) {
        return std::min(s_on_centerline.left, s_on_centerline.right);
      }
      return s_on_centerline.left ? s_on_centerline.left : s_on_centerline.right;
    }();
    if (s_intersection && s_start_edge) {
      s_intersection = std::min(s_intersection, s_start_edge);
    } else if (s_start_edge) {
      s_intersection = s_start_edge;
    }
  }

  // intersection between start edge of drivable area and center line
  {
    std::optional<double> s_start_edge;
    lanelet::BasicPoints2d start_edge_intersections;
    boost::geometry::intersection(start_edge, cropped_centerline, start_edge_intersections);
    for (const auto & intersection : start_edge_intersections) {
      auto s = lanelet::geometry::toArcCoordinates(cropped_centerline, intersection).length;
      // Ignore intersections near the beginning of the centerline.
      // It is impossible to make a turn shorter than the vehicle_length, so use it as a threshold.
      if (s < vehicle_length) continue;
      s += s_start;
      s_start_edge = s_start_edge ? std::min(*s_start_edge, s) : s;
    }
    if (s_intersection && s_start_edge) {
      s_intersection = std::min(s_intersection, s_start_edge);
    } else if (s_start_edge) {
      s_intersection = s_start_edge;
    }
  }

  return s_intersection;
}

std::optional<double> get_first_self_intersection_arc_length(
  const lanelet::BasicLineString2d & line_string)
{
  if (line_string.size() < 3) {
    return std::nullopt;
  }

  std::optional<size_t> first_self_intersection_index;
  std::optional<double> intersection_arc_length_on_latter_segment;
  double s = 0.;

  for (size_t i = 0; i < line_string.size() - 1; ++i) {
    if (
      first_self_intersection_index && i == first_self_intersection_index &&
      intersection_arc_length_on_latter_segment) {
      return s + *intersection_arc_length_on_latter_segment;
    }

    const auto current_segment = lanelet::BasicSegment2d{line_string.at(i), line_string.at(i + 1)};
    s += lanelet::geometry::length(current_segment);

    lanelet::BasicPoints2d self_intersections{};
    for (size_t j = i + 1; j < line_string.size() - 1; ++j) {
      const auto segment = lanelet::BasicSegment2d{line_string.at(j), line_string.at(j + 1)};
      if (
        segment.first == current_segment.second || segment.second == current_segment.first ||
        segment.first == current_segment.first) {
        continue;
      }
      boost::geometry::intersection(current_segment, segment, self_intersections);
      if (self_intersections.empty()) {
        continue;
      }
      first_self_intersection_index = j;
      intersection_arc_length_on_latter_segment =
        (self_intersections.front() - segment.first).norm();
      break;
    }
  }

  return std::nullopt;
}

double get_arc_length_on_path(
  const lanelet::LaneletSequence & lanelet_sequence, const std::vector<PathPointWithLaneId> & path,
  const double s_centerline)
{
  std::optional<lanelet::Id> target_lanelet_id;
  std::optional<lanelet::BasicPoint2d> point_on_centerline;

  if (lanelet_sequence.empty() || path.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("minimum_rule_based_planner").get_child("utils").get_child(__func__),
      "Input lanelet sequence or path is empty, returning 0.");
    return 0.;
  }

  if (s_centerline < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("minimum_rule_based_planner").get_child("utils").get_child(__func__),
      "Input arc length is negative, returning 0.");
    return 0.;
  }

  for (auto [it, s] = std::make_tuple(lanelet_sequence.begin(), 0.); it != lanelet_sequence.end();
       ++it) {
    const double centerline_length = lanelet::geometry::length(it->centerline2d());
    if (s + centerline_length < s_centerline) {
      s += centerline_length;
      continue;
    }

    target_lanelet_id = it->id();
    point_on_centerline =
      lanelet::geometry::interpolatedPointAtDistance(it->centerline2d(), s_centerline - s);
    break;
  }

  if (!target_lanelet_id || !point_on_centerline) {
    // lanelet_sequence is too short, thus we return input arc length as is.
    return s_centerline;
  }

  auto s_path = 0.;
  lanelet::BasicLineString2d target_path_segment;

  for (auto it = path.begin(); it != path.end(); ++it) {
    if (
      std::find(it->lane_ids.begin(), it->lane_ids.end(), *target_lanelet_id) ==
      it->lane_ids.end()) {
      if (target_path_segment.empty() && it != std::prev(path.end())) {
        s_path += autoware_utils::calc_distance2d(*it, *std::next(it));
        continue;
      }
      break;
    }
    target_path_segment.push_back(
      lanelet::utils::conversion::toLaneletPoint(it->point.pose.position).basicPoint2d());
  }

  // guard: toArcCoordinates requires at least 2 points in the line string,
  // otherwise closestSegment() dereferences a null pointer.
  if (target_path_segment.size() < 2) {
    RCLCPP_WARN(
      rclcpp::get_logger("path_generator").get_child("utils").get_child(__func__),
      "target_path_segment has insufficient points (%zu), "
      "falling back to closest point search on path",
      target_path_segment.size());
    // fallback: find the closest point on the entire path to point_on_centerline
    double min_dist_sq = std::numeric_limits<double>::max();
    double s_closest = 0.;
    double s_acc = 0.;
    for (size_t i = 0; i < path.size(); ++i) {
      const auto & pos = path[i].point.pose.position;
      const double dx = pos.x - point_on_centerline->x();
      const double dy = pos.y - point_on_centerline->y();
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        s_closest = s_acc;
      }
      if (i + 1 < path.size()) {
        s_acc += autoware_utils::calc_distance2d(path[i], path[i + 1]);
      }
    }
    return s_closest;
  }

  s_path += lanelet::geometry::toArcCoordinates(target_path_segment, *point_on_centerline).length;

  return s_path;
}

PathRange<std::vector<geometry_msgs::msg::Point>> get_path_bounds(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end)
{
  if (lanelet_sequence.empty()) {
    return {};
  }

  const auto [s_left_start, s_right_start] = get_arc_length_on_bounds(lanelet_sequence, s_start);
  const auto [s_left_end, s_right_end] = get_arc_length_on_bounds(lanelet_sequence, s_end);

  return {
    crop_line_string(
      to_geometry_msgs_points(
        lanelet_sequence.leftBound().begin(), lanelet_sequence.leftBound().end()),
      s_left_start, s_left_end),
    crop_line_string(
      to_geometry_msgs_points(
        lanelet_sequence.rightBound().begin(), lanelet_sequence.rightBound().end()),
      s_right_start, s_right_end)};
}

std::vector<geometry_msgs::msg::Point> crop_line_string(
  const std::vector<geometry_msgs::msg::Point> & line_string, const double s_start,
  const double s_end)
{
  if (s_start < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("minimum_rule_based_planner").get_child("utils").get_child(__func__),
      "Start of crop range is negative, returning input as is");
    return line_string;
  }

  if (s_start > s_end) {
    RCLCPP_WARN(
      rclcpp::get_logger("minimum_rule_based_planner").get_child("utils").get_child(__func__),
      "Start of crop range is larger than end, returning input as is");
    return line_string;
  }

  auto trajectory =
    autoware::experimental::trajectory::Trajectory<geometry_msgs::msg::Point>::Builder()
      .set_xy_interpolator<autoware::experimental::trajectory::interpolator::Linear>()
      .build(line_string);
  if (!trajectory) {
    return {};
  }

  trajectory->crop(s_start, s_end - s_start);
  return trajectory->restore();
}

PathRange<double> get_arc_length_on_bounds(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_centerline)
{
  if (s_centerline < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("minimum_rule_based_planner").get_child("utils").get_child(__func__),
      "Input arc length is negative, returning 0.");
    return {0., 0.};
  }

  auto s = 0.;
  auto s_left = 0.;
  auto s_right = 0.;

  for (auto it = lanelet_sequence.begin(); it != lanelet_sequence.end(); ++it) {
    const double centerline_length = lanelet::geometry::length(it->centerline2d());
    const double left_bound_length = lanelet::geometry::length(it->leftBound2d());
    const double right_bound_length = lanelet::geometry::length(it->rightBound2d());

    if (s + centerline_length < s_centerline) {
      s += centerline_length;
      s_left += left_bound_length;
      s_right += right_bound_length;
      continue;
    }

    const auto point_on_centerline =
      lanelet::geometry::interpolatedPointAtDistance(it->centerline2d(), s_centerline - s);
    s_left += lanelet::geometry::toArcCoordinates(it->leftBound2d(), point_on_centerline).length;
    s_right += lanelet::geometry::toArcCoordinates(it->rightBound2d(), point_on_centerline).length;

    return {s_left, s_right};
  }

  // If the loop ends without returning, it means that the lanelet_sequence is too short.
  // In this case, we return the original arc length on the centerline.
  return {s_centerline, s_centerline};
}

PathRange<std::optional<double>> get_arc_length_on_centerline(
  const lanelet::LaneletSequence & lanelet_sequence, const std::optional<double> & s_left_bound,
  const std::optional<double> & s_right_bound)
{
  std::optional<double> s_left_centerline;
  std::optional<double> s_right_centerline;

  if (s_left_bound && *s_left_bound < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("minimum_rule_based_planner").get_child("utils").get_child(__func__),
      "Input left arc length is negative, returning 0.");
    s_left_centerline = 0.;
  }
  if (s_right_bound && *s_right_bound < 0.) {
    RCLCPP_WARN(
      rclcpp::get_logger("minimum_rule_based_planner").get_child("utils").get_child(__func__),
      "Input right arc length is negative, returning 0.");
    s_right_centerline = 0.;
  }

  auto s = 0.;
  auto s_left = 0.;
  auto s_right = 0.;

  for (auto it = lanelet_sequence.begin(); it != lanelet_sequence.end(); ++it) {
    const auto is_left_done = !s_left_bound || s_left_centerline;
    const auto is_right_done = !s_right_bound || s_right_centerline;
    if (is_left_done && is_right_done) {
      break;
    }

    const double centerline_length = lanelet::utils::getLaneletLength2d(*it);
    const double left_bound_length = lanelet::geometry::length(it->leftBound2d());
    const double right_bound_length = lanelet::geometry::length(it->rightBound2d());

    if (!is_left_done && s_left + left_bound_length > s_left_bound) {
      s_left_centerline = s + lanelet::geometry::toArcCoordinates(
                                it->centerline2d(), lanelet::geometry::interpolatedPointAtDistance(
                                                      it->leftBound2d(), *s_left_bound - s_left))
                                .length;
    }
    if (!is_right_done && s_right + right_bound_length > s_right_bound) {
      s_right_centerline =
        s + lanelet::geometry::toArcCoordinates(
              it->centerline2d(), lanelet::geometry::interpolatedPointAtDistance(
                                    it->rightBound2d(), *s_right_bound - s_right))
              .length;
    }

    s += centerline_length;
    s_left += left_bound_length;
    s_right += right_bound_length;
  }

  return {
    s_left_centerline ? s_left_centerline : s_left_bound,
    s_right_centerline ? s_right_centerline : s_right_bound};
}

PathPointTrajectory refine_path_for_goal(
  const PathPointTrajectory & input, const geometry_msgs::msg::Pose & goal_pose,
  const lanelet::Id goal_lane_id, const double search_radius_range, const double pre_goal_offset)
{
  auto contain_goal_lane_id = [&](const PathPointWithLaneId & point) {
    const auto & ids = point.lane_ids;
    return std::find(ids.begin(), ids.end(), goal_lane_id) != ids.end();
  };

  auto outside_circle = [&](const PathPointWithLaneId & point) {
    return autoware_utils::calc_distance2d(point.point.pose, goal_pose) > search_radius_range;
  };

  auto closest_to_goal = autoware::experimental::trajectory::closest_with_constraint(
    input, goal_pose, contain_goal_lane_id);

  if (!closest_to_goal) {
    return input;
  }

  auto cropped_path = autoware::experimental::trajectory::crop(input, 0, *closest_to_goal);

  auto intervals =
    autoware::experimental::trajectory::find_intervals(cropped_path, outside_circle, 10);

  std::vector<PathPointWithLaneId> goal_connected_trajectory_points;

  if (!intervals.empty()) {
    auto cropped = autoware::experimental::trajectory::crop(cropped_path, 0, intervals.back().end);
    goal_connected_trajectory_points = cropped.restore(2);
  } else if (cropped_path.length() > pre_goal_offset) {
    // If distance from start to goal is smaller than refine_goal_search_radius_range and start is
    // farther from goal than pre_goal, we just connect start, pre_goal, and goal.
    goal_connected_trajectory_points = {cropped_path.compute(0)};
  }

  auto goal = input.compute(autoware::experimental::trajectory::closest(input, goal_pose));
  goal.point.pose = goal_pose;
  goal.point.longitudinal_velocity_mps = 0.0;

  const auto pre_goal_pose =
    autoware_utils::calc_offset_pose(goal_pose, -pre_goal_offset, 0.0, 0.0);
  auto pre_goal = input.compute(autoware::experimental::trajectory::closest(input, pre_goal_pose));
  pre_goal.point.pose = pre_goal_pose;

  goal_connected_trajectory_points.push_back(pre_goal);
  goal_connected_trajectory_points.push_back(goal);

  if (
    const auto output =
      autoware::experimental::trajectory::pretty_build(goal_connected_trajectory_points)) {
    return *output;
  }
  return input;
}

lanelet::ConstLanelets extract_lanelets_from_trajectory(
  const PathPointTrajectory & trajectory, const RouteContext & planner_data)
{
  lanelet::ConstLanelets lanelets{};
  const auto lane_ids = trajectory.get_contained_lane_ids();
  const auto lane_ids_set = std::set(lane_ids.begin(), lane_ids.end());
  for (const auto & lane_id : lane_ids_set) {
    const auto lanelet = planner_data.lanelet_map_ptr->laneletLayer.get(lane_id);
    lanelets.push_back(lanelet);
  }
  return lanelets;
}

bool is_in_lanelets(const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & lanes)
{
  return std::any_of(lanes.begin(), lanes.end(), [&](const auto & lane) {
    return lanelet::utils::isInLanelet(pose, lane);
  });
}

bool is_trajectory_inside_lanelets(
  const PathPointTrajectory & refined_path, const lanelet::ConstLanelets & lanelets)
{
  for (double s = 0.0; s < refined_path.length(); s += 0.1) {
    const auto point = refined_path.compute(s);
    if (!is_in_lanelets(point.point.pose, lanelets)) {
      return false;
    }
  }
  return true;
}

std::optional<PathPointTrajectory> modify_path_for_smooth_goal_connection(
  const PathPointTrajectory & trajectory, const RouteContext & planner_data,
  const double search_radius_range, const double pre_goal_offset)
{
  if (planner_data.preferred_lanelets.empty()) {
    return std::nullopt;
  }
  const auto lanelets = extract_lanelets_from_trajectory(trajectory, planner_data);

  // This process is to fit the trajectory inside the lanelets. By reducing
  // refine_goal_search_radius_range, we can fit the trajectory inside lanelets even if the
  // trajectory has a high curvature.
  for (double s = search_radius_range; s > 0; s -= 0.1) {
    const auto refined_trajectory = refine_path_for_goal(
      trajectory, planner_data.goal_pose, planner_data.preferred_lanelets.back().id(),
      search_radius_range, pre_goal_offset);
    const bool is_inside = is_trajectory_inside_lanelets(refined_trajectory, lanelets);
    if (is_inside) {
      return refined_trajectory;
    }
  }
  return std::nullopt;
}

autoware_planning_msgs::msg::Trajectory convert_path_to_trajectory(
  const PathWithLaneId & path, double resample_interval)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> traj_points;
  traj_points.reserve(path.points.size());
  for (const auto & path_point : path.points) {
    autoware_planning_msgs::msg::TrajectoryPoint traj_point;
    traj_point.pose = path_point.point.pose;
    traj_point.longitudinal_velocity_mps = path_point.point.longitudinal_velocity_mps;
    traj_point.lateral_velocity_mps = path_point.point.lateral_velocity_mps;
    traj_point.heading_rate_rps = path_point.point.heading_rate_rps;
    traj_points.push_back(traj_point);
  }

  const auto traj_msg = autoware::motion_utils::convertToTrajectory(traj_points, path.header);
  return autoware::motion_utils::resampleTrajectory(
    traj_msg, resample_interval, false, true, true, true);
}

std::optional<LaneTransitionInfo> detect_lane_transition(
  const lanelet::ConstLanelet & current_lanelet, const RouteContext & route_context,
  const double forward_distance)
{
  // Helper lambda: BFS laterally through adjacent lanelets to find a preferred lanelet.
  // Supports multi-lane transitions (e.g. 3-4 lanes across).
  auto try_build_transition =
    [&](const lanelet::ConstLanelet & lanelet) -> std::optional<LaneTransitionInfo> {
    std::vector<lanelet::Id> visited = {lanelet.id()};
    std::vector<lanelet::ConstLanelet> queue;

    // Seed BFS with direct adjacents
    for (const auto & beside : route_context.routing_graph_ptr->besides(lanelet)) {
      if (beside.id() != lanelet.id()) {
        queue.push_back(beside);
        visited.push_back(beside.id());
      }
    }

    while (!queue.empty()) {
      std::vector<lanelet::ConstLanelet> next_queue;
      for (const auto & candidate : queue) {
        if (exists(route_context.preferred_lanelets, candidate)) {
          // Found a preferred lanelet (possibly multiple lanes away)
          LaneTransitionInfo info;
          info.pre_transition_lanelets.push_back(current_lanelet);
          info.transition_from = lanelet;
          info.transition_to = candidate;

          // Build post-transition lanelets from the preferred lanelet forward
          info.post_transition_lanelets.push_back(candidate);
          auto post_current = candidate;
          double post_length = 0.0;
          while (post_length < forward_distance) {
            const auto post_next = get_next_lanelet_within_route(post_current, route_context);
            if (!post_next) {
              break;
            }
            info.post_transition_lanelets.push_back(*post_next);
            post_current = *post_next;
            post_length += lanelet::geometry::length2d(*post_next);
          }
          return info;
        }

        // Continue BFS: expand to adjacents of candidate
        for (const auto & beside : route_context.routing_graph_ptr->besides(candidate)) {
          if (std::find(visited.begin(), visited.end(), beside.id()) == visited.end()) {
            next_queue.push_back(beside);
            visited.push_back(beside.id());
          }
        }
      }
      queue = next_queue;
    }
    return std::nullopt;
  };

  // First check if current lanelet itself has an adjacent preferred lanelet
  // (ego is already on the transition lanelet)
  if (!exists(route_context.preferred_lanelets, current_lanelet)) {
    if (auto info = try_build_transition(current_lanelet)) {
      // pre_transition_lanelets already contains current_lanelet from the lambda
      return info;
    }
  }

  // Walk forward to find a transition point ahead
  auto current = current_lanelet;
  double accumulated_length = 0.0;
  lanelet::ConstLanelets intermediate_lanelets;

  while (accumulated_length < forward_distance) {
    const auto following = route_context.routing_graph_ptr->following(current);
    if (following.empty()) {
      break;
    }

    // Find next route lanelet
    const auto next_it = std::find_if(
      following.cbegin(), following.cend(),
      [&](const lanelet::ConstLanelet & l) { return exists(route_context.route_lanelets, l); });
    if (next_it == following.cend()) {
      break;
    }
    const auto & next = *next_it;

    // If next is directly a preferred lanelet, the path continues on the preferred lane
    // without needing a lateral transition → no blend needed
    if (exists(route_context.preferred_lanelets, next)) {
      return std::nullopt;
    }

    // Check if the next lanelet has an adjacent preferred lanelet
    if (auto info = try_build_transition(next)) {
      // Insert intermediate lanelets into pre_transition_lanelets
      // (lambda already has current_lanelet as first element)
      for (const auto & ll : intermediate_lanelets) {
        info->pre_transition_lanelets.push_back(ll);
      }
      info->pre_transition_lanelets.push_back(next);
      return info;
    }

    // Continue forward on current lane
    intermediate_lanelets.push_back(next);
    current = next;
    accumulated_length += lanelet::geometry::length2d(next);
  }

  return std::nullopt;
}

std::vector<PathPointWithLaneId> blend_centerlines(
  const std::vector<PathPointWithLaneId> & source_points,
  const std::vector<PathPointWithLaneId> & target_points, const double blend_start_s,
  const double blend_length)
{
  if (source_points.empty() || target_points.empty() || blend_length <= 0.0) {
    return source_points;
  }

  // Compute arc lengths for source points
  std::vector<double> source_arc_lengths(source_points.size(), 0.0);
  for (size_t i = 1; i < source_points.size(); ++i) {
    source_arc_lengths[i] = source_arc_lengths[i - 1] + autoware_utils::calc_distance2d(
                                                          source_points[i - 1].point.pose.position,
                                                          source_points[i].point.pose.position);
  }

  // Compute arc lengths for target points
  std::vector<double> target_arc_lengths(target_points.size(), 0.0);
  for (size_t i = 1; i < target_points.size(); ++i) {
    target_arc_lengths[i] = target_arc_lengths[i - 1] + autoware_utils::calc_distance2d(
                                                          target_points[i - 1].point.pose.position,
                                                          target_points[i].point.pose.position);
  }

  const double blend_end_s = blend_start_s + blend_length;

  std::vector<PathPointWithLaneId> blended_points;
  blended_points.reserve(source_points.size() + target_points.size());

  // Phase 1: Source points before blend zone
  for (size_t i = 0; i < source_points.size(); ++i) {
    if (source_arc_lengths[i] >= blend_start_s) {
      break;
    }
    blended_points.push_back(source_points[i]);
  }

  // Phase 2: Blend zone - interpolate between source and target
  // Use source points in the blend zone and interpolate target position at same arc length
  for (size_t i = 0; i < source_points.size(); ++i) {
    const double s = source_arc_lengths[i];
    if (s < blend_start_s || s > blend_end_s) {
      continue;
    }

    // Smoothstep weight: w = 3t^2 - 2t^3
    const double t = std::clamp((s - blend_start_s) / blend_length, 0.0, 1.0);
    const double w = t * t * (3.0 - 2.0 * t);

    // Find corresponding position on target centerline at same relative progress within blend zone
    // Map blend progress [0,1] to target arc length [0, blend_length]
    const double target_s = t * std::min(blend_length, target_arc_lengths.back());

    // Find target point by linear interpolation at target_s
    size_t target_idx = 0;
    for (size_t j = 1; j < target_points.size(); ++j) {
      if (target_arc_lengths[j] >= target_s) {
        target_idx = j - 1;
        break;
      }
      target_idx = j;
    }

    geometry_msgs::msg::Point target_pos;
    if (target_idx + 1 < target_points.size()) {
      const double seg_len = target_arc_lengths[target_idx + 1] - target_arc_lengths[target_idx];
      const double ratio =
        (seg_len > 1e-6) ? (target_s - target_arc_lengths[target_idx]) / seg_len : 0.0;
      const auto & p0 = target_points[target_idx].point.pose.position;
      const auto & p1 = target_points[target_idx + 1].point.pose.position;
      target_pos.x = p0.x + ratio * (p1.x - p0.x);
      target_pos.y = p0.y + ratio * (p1.y - p0.y);
      target_pos.z = p0.z + ratio * (p1.z - p0.z);
    } else {
      target_pos = target_points.back().point.pose.position;
    }

    // Interpolate position
    PathPointWithLaneId blended_point = source_points[i];
    blended_point.point.pose.position.x =
      (1.0 - w) * source_points[i].point.pose.position.x + w * target_pos.x;
    blended_point.point.pose.position.y =
      (1.0 - w) * source_points[i].point.pose.position.y + w * target_pos.y;
    blended_point.point.pose.position.z =
      (1.0 - w) * source_points[i].point.pose.position.z + w * target_pos.z;

    // Use target lane_id in the latter half of the blend
    if (w > 0.5 && !target_points.empty()) {
      blended_point.lane_ids =
        target_points[std::min(target_idx, target_points.size() - 1)].lane_ids;
    }

    blended_points.push_back(blended_point);
  }

  // Phase 3: Target points after blend zone
  // Append target points whose arc length exceeds the blend portion (blend_length)
  const double target_s_at_blend_end = std::min(blend_length, target_arc_lengths.back());
  for (size_t i = 0; i < target_points.size(); ++i) {
    if (target_arc_lengths[i] > target_s_at_blend_end) {
      blended_points.push_back(target_points[i]);
    }
  }

  return blended_points;
}

}  // namespace autoware::minimum_rule_based_planner::utils
