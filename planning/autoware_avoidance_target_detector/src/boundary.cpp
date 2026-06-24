// Copyright 2026 Autoware Foundation
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

#include "autoware/avoidance_target_detector/boundary.hpp"

#include "autoware/avoidance_target_detector/map_construction.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/path_point.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <algorithm>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::avoidance_target_detector
{

namespace
{

using autoware_planning_msgs::msg::PathPoint;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using traffic_rules::get_left_lanelet;
using traffic_rules::get_right_lanelet;

struct DrivableLanes
{
  lanelet::ConstLanelet left_lane;
  lanelet::ConstLanelet right_lane;
};

using FootprintsByLaneletId =
  std::unordered_map<lanelet::Id, std::vector<autoware_utils::LinearRing2d>>;

DrivableLanes make_not_expanded_drivable_lanes(const lanelet::ConstLanelet & lanelet)
{
  return DrivableLanes{lanelet, lanelet};
}

autoware_utils::LinearRing2d create_vehicle_footprint(
  const Pose & ego_pose, const VehicleInfo & vehicle_info)
{
  const auto local_footprint = vehicle_info.createFootprint();
  return autoware_utils_geometry::transform_vector(
    local_footprint, autoware_utils_geometry::pose2transform(ego_pose));
}

// Indices of rear-right and rear-left corners in VehicleInfo::createFootprint().
constexpr size_t k_rear_right_footprint_index = 3;
constexpr size_t k_rear_left_footprint_index = 4;

Pose create_rear_midpoint_pose(
  const autoware_utils::LinearRing2d & footprint, const Pose & base_pose)
{
  if (footprint.size() <= k_rear_left_footprint_index) {
    return base_pose;
  }

  const auto & rear_right = footprint[k_rear_right_footprint_index];
  const auto & rear_left = footprint[k_rear_left_footprint_index];

  Pose rear_midpoint_pose = base_pose;
  rear_midpoint_pose.position.x = (rear_right.x() + rear_left.x()) * 0.5;
  rear_midpoint_pose.position.y = (rear_right.y() + rear_left.y()) * 0.5;
  return rear_midpoint_pose;
}

autoware_utils_geometry::Polygon2d to_polygon2d(const autoware_utils::LinearRing2d & footprint)
{
  autoware_utils_geometry::Polygon2d polygon;
  for (const auto & point : footprint) {
    polygon.outer().emplace_back(point.x(), point.y());
  }
  boost::geometry::correct(polygon);
  return polygon;
}

autoware_utils_geometry::Polygon2d to_polygon2d(const lanelet::ConstLanelet & lanelet)
{
  autoware_utils_geometry::Polygon2d polygon;
  for (const auto & point : lanelet.polygon2d().basicPolygon()) {
    polygon.outer().emplace_back(point.x(), point.y());
  }
  boost::geometry::correct(polygon);
  return polygon;
}

bool footprint_intersects_lanelet(
  const autoware_utils::LinearRing2d & footprint, const lanelet::ConstLanelet & lanelet)
{
  return boost::geometry::intersects(to_polygon2d(footprint), to_polygon2d(lanelet));
}

bool is_same_or_left_of(
  const lanelet::ConstLanelet & base, const lanelet::ConstLanelet & candidate,
  const lanelet::routing::RoutingGraph & routing_graph)
{
  auto current = base;
  if (current.id() == candidate.id()) {
    return true;
  }
  while (const auto left = get_left_lanelet(routing_graph, current)) {
    if (left->id() == candidate.id()) {
      return true;
    }
    current = *left;
  }
  return false;
}

bool is_same_or_right_of(
  const lanelet::ConstLanelet & base, const lanelet::ConstLanelet & candidate,
  const lanelet::routing::RoutingGraph & routing_graph)
{
  auto current = base;
  if (current.id() == candidate.id()) {
    return true;
  }
  while (const auto right = get_right_lanelet(routing_graph, current)) {
    if (right->id() == candidate.id()) {
      return true;
    }
    current = *right;
  }
  return false;
}

lanelet::ConstLanelet pick_leftmost_lane(
  const lanelet::ConstLanelet & base, const lanelet::ConstLanelet & a,
  const lanelet::ConstLanelet & b, const lanelet::routing::RoutingGraph & routing_graph)
{
  if (is_same_or_left_of(base, a, routing_graph) && is_same_or_left_of(a, b, routing_graph)) {
    return b;
  }
  if (is_same_or_left_of(base, b, routing_graph) && is_same_or_left_of(b, a, routing_graph)) {
    return a;
  }
  return a;
}

lanelet::ConstLanelet pick_rightmost_lane(
  const lanelet::ConstLanelet & base, const lanelet::ConstLanelet & a,
  const lanelet::ConstLanelet & b, const lanelet::routing::RoutingGraph & routing_graph)
{
  if (is_same_or_right_of(base, a, routing_graph) && is_same_or_right_of(a, b, routing_graph)) {
    return b;
  }
  if (is_same_or_right_of(base, b, routing_graph) && is_same_or_right_of(b, a, routing_graph)) {
    return a;
  }
  return a;
}

lanelet::ConstLanelet expand_lane_to_left(
  const lanelet::ConstLanelet & lanelet, const RouteHandler & route_handler,
  const lanelet::routing::RoutingGraph & routing_graph,
  const autoware_utils::LinearRing2d & footprint, const bool allow_non_route_neighbors)
{
  auto left_lane = lanelet;
  auto current_lane = lanelet;

  while (const auto neighbor = get_left_lanelet(routing_graph, current_lane)) {
    if (!allow_non_route_neighbors && !route_handler.isRouteLanelet(*neighbor)) {
      break;
    }
    if (!footprint_intersects_lanelet(footprint, *neighbor)) {
      break;
    }
    left_lane = *neighbor;
    current_lane = *neighbor;
  }

  return left_lane;
}

lanelet::ConstLanelet expand_lane_to_right(
  const lanelet::ConstLanelet & lanelet, const RouteHandler & route_handler,
  const lanelet::routing::RoutingGraph & routing_graph,
  const autoware_utils::LinearRing2d & footprint, const bool allow_non_route_neighbors)
{
  auto right_lane = lanelet;
  auto current_lane = lanelet;

  while (const auto neighbor = get_right_lanelet(routing_graph, current_lane)) {
    if (!allow_non_route_neighbors && !route_handler.isRouteLanelet(*neighbor)) {
      break;
    }
    if (!footprint_intersects_lanelet(footprint, *neighbor)) {
      break;
    }
    right_lane = *neighbor;
    current_lane = *neighbor;
  }

  return right_lane;
}

DrivableLanes expand_drivable_lanes_for_footprint(
  const lanelet::ConstLanelet & lanelet, const RouteHandler & route_handler,
  const lanelet::routing::RoutingGraph & routing_graph,
  const autoware_utils::LinearRing2d & footprint, const bool allow_non_route_neighbors)
{
  DrivableLanes drivable_lanes;
  drivable_lanes.left_lane = expand_lane_to_left(
    lanelet, route_handler, routing_graph, footprint, allow_non_route_neighbors);
  drivable_lanes.right_lane = expand_lane_to_right(
    lanelet, route_handler, routing_graph, footprint, allow_non_route_neighbors);
  return drivable_lanes;
}

DrivableLanes expand_drivable_lanes_for_footprints(
  const lanelet::ConstLanelet & lanelet, const RouteHandler & route_handler,
  const lanelet::routing::RoutingGraph & routing_graph,
  const std::vector<autoware_utils::LinearRing2d> & footprints,
  const bool allow_non_route_neighbors)
{
  auto drivable_lanes = make_not_expanded_drivable_lanes(lanelet);
  for (const auto & footprint : footprints) {
    const auto expanded = expand_drivable_lanes_for_footprint(
      lanelet, route_handler, routing_graph, footprint, allow_non_route_neighbors);
    drivable_lanes.left_lane =
      pick_leftmost_lane(lanelet, drivable_lanes.left_lane, expanded.left_lane, routing_graph);
    drivable_lanes.right_lane =
      pick_rightmost_lane(lanelet, drivable_lanes.right_lane, expanded.right_lane, routing_graph);
  }
  return drivable_lanes;
}

bool is_lanelet_in_first_n_route_sections(
  const LaneletRoute & route, const lanelet::ConstLanelet & lanelet, const size_t n)
{
  if (route.segments.empty() || n == 0) {
    return false;
  }

  const size_t end_idx = std::min(n, route.segments.size());
  for (size_t i = 0; i < end_idx; ++i) {
    for (const auto & primitive : route.segments[i].primitives) {
      if (primitive.id == lanelet.id()) {
        return true;
      }
    }
  }
  return false;
}

bool is_lanelet_in_last_n_route_sections(
  const LaneletRoute & route, const lanelet::ConstLanelet & lanelet, const size_t n)
{
  if (route.segments.empty() || n == 0) {
    return false;
  }

  const size_t start_idx = route.segments.size() > n ? route.segments.size() - n : 0;
  for (size_t i = start_idx; i < route.segments.size(); ++i) {
    for (const auto & primitive : route.segments[i].primitives) {
      if (primitive.id == lanelet.id()) {
        return true;
      }
    }
  }
  return false;
}

bool is_lanelet_in_relaxed_route_sections(
  const LaneletRoute * route, const lanelet::ConstLanelet & lanelet)
{
  if (route == nullptr) {
    return false;
  }

  constexpr size_t route_section_relax_count = 5;
  return is_lanelet_in_first_n_route_sections(*route, lanelet, route_section_relax_count) ||
         is_lanelet_in_last_n_route_sections(*route, lanelet, route_section_relax_count);
}

std::vector<lanelet::Id> collect_overlapping_route_lanelets(
  const lanelet::ConstLanelet & seed_lanelet, const autoware_utils::LinearRing2d & footprint,
  const RouteHandler & route_handler, const lanelet::routing::RoutingGraph & routing_graph,
  FootprintsByLaneletId & footprints_by_lanelet_id, const LaneletRoute * route)
{
  const bool allow_non_route_neighbors = is_lanelet_in_relaxed_route_sections(route, seed_lanelet);

  std::vector<lanelet::ConstLanelet> queue{seed_lanelet};
  std::unordered_set<lanelet::Id> visited;
  std::unordered_set<lanelet::Id> newly_found_lanelet_ids;

  const auto register_route_lanelet = [&](const lanelet::ConstLanelet & route_lanelet) {
    footprints_by_lanelet_id[route_lanelet.id()].push_back(footprint);
    newly_found_lanelet_ids.insert(route_lanelet.id());
  };

  while (!queue.empty()) {
    const auto current_lanelet = queue.back();
    queue.pop_back();

    if (!visited.insert(current_lanelet.id()).second) {
      continue;
    }

    const bool is_route_lanelet = route_handler.isRouteLanelet(current_lanelet);
    if (!is_route_lanelet && !allow_non_route_neighbors) {
      continue;
    }
    if (!footprint_intersects_lanelet(footprint, current_lanelet)) {
      if (const auto left_lanelet = get_left_lanelet(routing_graph, current_lanelet)) {
        queue.push_back(*left_lanelet);
      }
      if (const auto right_lanelet = get_right_lanelet(routing_graph, current_lanelet)) {
        queue.push_back(*right_lanelet);
      }
      continue;
    }

    if (is_route_lanelet) {
      register_route_lanelet(current_lanelet);
    } else if (route_handler.isRouteLanelet(seed_lanelet)) {
      register_route_lanelet(seed_lanelet);
    }

    if (const auto left_lanelet = get_left_lanelet(routing_graph, current_lanelet)) {
      queue.push_back(*left_lanelet);
    }
    if (const auto right_lanelet = get_right_lanelet(routing_graph, current_lanelet)) {
      queue.push_back(*right_lanelet);
    }
  }

  return {newly_found_lanelet_ids.begin(), newly_found_lanelet_ids.end()};
}

std::vector<lanelet::ConstLanelet> collect_trajectory_lanelets(
  const RouteHandler & route_handler, const lanelet::routing::RoutingGraph & routing_graph,
  const Trajectory & trajectory, const VehicleInfo & vehicle_info,
  FootprintsByLaneletId & footprints_by_lanelet_id, const LaneletRoute * route)
{
  std::vector<lanelet::ConstLanelet> ordered_lanelets;
  std::unordered_set<lanelet::Id> added_lanelet_ids;
  const auto lanelet_map_ptr = route_handler.getLaneletMapPtr();

  for (const auto & trajectory_point : trajectory.points) {
    const auto footprint = create_vehicle_footprint(trajectory_point.pose, vehicle_info);
    const auto rear_midpoint_pose = create_rear_midpoint_pose(footprint, trajectory_point.pose);

    lanelet::ConstLanelet closest_lanelet;
    if (!route_handler.getClosestLaneletWithinRoute(rear_midpoint_pose, &closest_lanelet)) {
      continue;
    }

    const auto newly_found_lanelet_ids = collect_overlapping_route_lanelets(
      closest_lanelet, footprint, route_handler, routing_graph, footprints_by_lanelet_id, route);

    for (const auto lanelet_id : newly_found_lanelet_ids) {
      if (!added_lanelet_ids.insert(lanelet_id).second) {
        continue;
      }
      ordered_lanelets.push_back(lanelet_map_ptr->laneletLayer.get(lanelet_id));
    }
  }

  return ordered_lanelets;
}

std::vector<DrivableLanes> build_drivable_lanes(
  const std::vector<lanelet::ConstLanelet> & trajectory_lanelets,
  const RouteHandler & route_handler, const lanelet::routing::RoutingGraph & routing_graph,
  const FootprintsByLaneletId & footprints_by_lanelet_id, const LaneletRoute * route)
{
  std::vector<DrivableLanes> drivable_lanes;
  drivable_lanes.reserve(trajectory_lanelets.size());

  for (const auto & lanelet : trajectory_lanelets) {
    const auto footprints_it = footprints_by_lanelet_id.find(lanelet.id());
    if (footprints_it == footprints_by_lanelet_id.end() || footprints_it->second.empty()) {
      continue;
    }
    const bool allow_non_route_neighbors = is_lanelet_in_relaxed_route_sections(route, lanelet);
    drivable_lanes.push_back(expand_drivable_lanes_for_footprints(
      lanelet, route_handler, routing_graph, footprints_it->second, allow_non_route_neighbors));
  }

  return drivable_lanes;
}

std::vector<Point> generate_bound(
  const std::vector<DrivableLanes> & drivable_lanes, const bool is_left)
{
  constexpr double overlap_threshold = 0.01;

  std::vector<lanelet::ConstPoint3d> points;
  lanelet::Id previous_bound_id = lanelet::InvalId;

  for (const auto & drivable_lane : drivable_lanes) {
    const auto bound =
      is_left ? drivable_lane.left_lane.leftBound3d() : drivable_lane.right_lane.rightBound3d();

    if (bound.id() == previous_bound_id) {
      continue;
    }
    previous_bound_id = bound.id();

    for (const auto & point : bound) {
      if (
        points.empty() ||
        overlap_threshold < (points.back().basicPoint2d() - point.basicPoint2d()).norm()) {
        points.push_back(point);
      }
    }
  }

  std::vector<Point> bound_points;
  bound_points.reserve(points.size());
  for (const auto & point : points) {
    bound_points.push_back(lanelet::utils::conversion::toGeomMsgPt(point));
  }
  return bound_points;
}

}  // namespace

std::optional<DrivableAreaResult> create_drivable_area(
  const Trajectory & trajectory, const VehicleInfo & vehicle_info, const LaneletRoute & route,
  const RouteHandler & route_handler, const lanelet::routing::RoutingGraph & routing_graph)
{
  if (!route_handler.isHandlerReady() || trajectory.points.empty()) {
    return std::nullopt;
  }

  FootprintsByLaneletId footprints_by_lanelet_id;
  const auto trajectory_lanelets = collect_trajectory_lanelets(
    route_handler, routing_graph, trajectory, vehicle_info, footprints_by_lanelet_id, &route);
  if (trajectory_lanelets.empty()) {
    return std::nullopt;
  }

  const auto drivable_lanes = build_drivable_lanes(
    trajectory_lanelets, route_handler, routing_graph, footprints_by_lanelet_id, &route);

  auto left_bound = generate_bound(drivable_lanes, true);
  auto right_bound = generate_bound(drivable_lanes, false);
  if (left_bound.size() < 2 || right_bound.size() < 2) {
    return std::nullopt;
  }

  DrivableAreaResult result;
  result.header = trajectory.header;
  result.left_bound = std::move(left_bound);
  result.right_bound = std::move(right_bound);
  return result;
}

DrivableAreaResult to_drivable_area_result(
  const std::pair<lanelet::LineString2d, lanelet::LineString2d> & bounds,
  const std_msgs::msg::Header & header)
{
  auto to_geometry_points = [](const lanelet::LineString2d & linestring) {
    std::vector<Point> points;
    points.reserve(linestring.size());
    for (const auto & point : linestring) {
      Point geometry_point;
      geometry_point.x = point.x();
      geometry_point.y = point.y();
      geometry_point.z = 0.0;
      points.push_back(geometry_point);
    }
    return points;
  };

  DrivableAreaResult result;
  result.header = header;
  result.left_bound = to_geometry_points(bounds.first);
  result.right_bound = to_geometry_points(bounds.second);
  return result;
}

Path to_path_msg(const DrivableAreaResult & area, const Trajectory & trajectory)
{
  Path path;
  path.header = area.header;
  path.left_bound = area.left_bound;
  path.right_bound = area.right_bound;

  path.points.reserve(trajectory.points.size());
  for (const auto & trajectory_point : trajectory.points) {
    PathPoint path_point;
    path_point.pose = trajectory_point.pose;
    path_point.longitudinal_velocity_mps = trajectory_point.longitudinal_velocity_mps;
    path_point.lateral_velocity_mps = trajectory_point.lateral_velocity_mps;
    path_point.heading_rate_rps = trajectory_point.heading_rate_rps;
    path.points.push_back(path_point);
  }

  return path;
}

}  // namespace autoware::avoidance_target_detector
