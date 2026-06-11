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

#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware/lanelet2_utils/kind.hpp>
#include <autoware/lanelet2_utils/topology.hpp>
#include <autoware_lane_event_classifier/geometry_utils.hpp>
#include <autoware_lane_event_classifier/lane_following/checker.hpp>
#include <magic_enum.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BoundingBox.h>

#include <cmath>
#include <limits>
#include <string_view>
#include <unordered_set>

namespace autoware::lane_event_classifier
{

namespace
{
namespace lanelet2_utils = autoware::experimental::lanelet2_utils;

// The reference lane plus every longitudinally connected lane reachable forward/backward within
// `length`.
std::unordered_set<lanelet::Id> build_connected_lane_sequence_ids(
  const lanelet::ConstLanelet & reference_lane,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph, double length)
{
  std::unordered_set<lanelet::Id> ids;
  ids.insert(reference_lane.id());
  if (!routing_graph) {
    return ids;
  }
  for (const auto & sequence :
       lanelet2_utils::get_succeeding_lanelet_sequences(reference_lane, routing_graph, length)) {
    for (const auto & lane : sequence) {
      ids.insert(lane.id());
    }
  }
  for (const auto & sequence :
       lanelet2_utils::get_preceding_lanelet_sequences(reference_lane, routing_graph, length)) {
    for (const auto & lane : sequence) {
      ids.insert(lane.id());
    }
  }
  return ids;
}

bool is_point_inside_any(
  const lanelet::LaneletMapPtr & map, const std::unordered_set<lanelet::Id> & ids,
  const lanelet::BasicPoint2d & point)
{
  return std::any_of(ids.cbegin(), ids.cend(), [&](const auto id) {
    return map->laneletLayer.exists(id) &&
           lanelet::geometry::inside(map->laneletLayer.get(id), point);
  });
}

bool is_point_within_tolerance_of_any(
  const lanelet::LaneletMapPtr & map, const std::unordered_set<lanelet::Id> & ids,
  const lanelet::BasicPoint2d & point, double tolerance)
{
  return std::any_of(ids.cbegin(), ids.cend(), [&](const auto id) {
    return map->laneletLayer.exists(id) &&
           lanelet::geometry::distance2d(map->laneletLayer.get(id), point) <= tolerance;
  });
}

// road_shoulder_exempt: ego point overlaps a road shoulder (shoulders are excluded from the routing
// graph).
bool is_point_in_road_shoulder(
  const lanelet::LaneletMapPtr & map, const lanelet::BasicPoint2d & point)
{
  const lanelet::BoundingBox2d query_box(point, point);
  for (const auto & lane : map->laneletLayer.search(query_box)) {
    if (lanelet2_utils::is_shoulder_lane(lane) && lanelet::geometry::inside(lane, point)) {
      return true;
    }
  }
  return false;
}

// turn_lane_exempt: ego is in a turn / intersection lane (the reference lane, or a sequence lane it
// sits in).
bool is_in_turn_lane(
  const lanelet::LaneletMapPtr & map, const lanelet::ConstLanelet & reference_lane,
  const std::unordered_set<lanelet::Id> & sequence_ids, const lanelet::BasicPoint2d & point)
{
  if (lanelet2_utils::is_intersection_lanelet(reference_lane)) {
    return true;
  }
  for (const auto id : sequence_ids) {
    if (!map->laneletLayer.exists(id)) {
      continue;
    }
    const auto lane = map->laneletLayer.get(id);
    if (lanelet2_utils::is_intersection_lanelet(lane) && lanelet::geometry::inside(lane, point)) {
      return true;
    }
  }
  return false;
}

// virtual_boundary_exempt: the boundary the ego crossed is virtual (approximated by the nearest
// sequence boundary).
bool nearest_sequence_boundary_is_virtual(
  const lanelet::LaneletMapPtr & map, const std::unordered_set<lanelet::Id> & ids,
  const lanelet::BasicPoint2d & point)
{
  double nearest_distance = std::numeric_limits<double>::max();
  bool nearest_is_virtual = false;
  for (const auto id : ids) {
    if (!map->laneletLayer.exists(id)) {
      continue;
    }
    const auto lane = map->laneletLayer.get(id);
    const double left_distance =
      std::abs(lanelet::geometry::toArcCoordinates(lane.leftBound2d(), point).distance);
    const double right_distance =
      std::abs(lanelet::geometry::toArcCoordinates(lane.rightBound2d(), point).distance);
    if (left_distance < nearest_distance) {
      nearest_distance = left_distance;
      nearest_is_virtual = is_virtual_linestring(lane.leftBound());
    }
    if (right_distance < nearest_distance) {
      nearest_distance = right_distance;
      nearest_is_virtual = is_virtual_linestring(lane.rightBound());
    }
  }
  return nearest_is_virtual;
}

}  // namespace

std::string_view to_string(LaneFollowingReason reason)
{
  return magic_enum::enum_name(reason);
}

LaneFollowingChecker::LaneFollowingChecker(LaneFollowingConfig config) : config_{config}
{
}

const std::unordered_set<lanelet::Id> & LaneFollowingChecker::connected_sequence_ids(
  const lanelet::ConstLanelet & reference_lane,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph_ptr) const
{
  if (
    routing_graph_ptr.get() == cached_graph_ptr_ &&
    reference_lane.id() == cached_reference_lane_id_) {
    return cached_sequence_ids_;
  }
  cached_graph_ptr_ = routing_graph_ptr.get();
  cached_reference_lane_id_ = reference_lane.id();
  cached_sequence_ids_ = build_connected_lane_sequence_ids(
    reference_lane, routing_graph_ptr, config_.connected_sequence_length_m);
  return cached_sequence_ids_;
}

LaneFollowingResult LaneFollowingChecker::evaluate(
  const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph_ptr, lanelet::Id reference_lane_id,
  const lanelet::BasicPoint2d & ego_point) const
{
  if (
    !lanelet_map_ptr || reference_lane_id == lanelet::InvalId ||
    !lanelet_map_ptr->laneletLayer.exists(reference_lane_id)) {
    return {true, LaneFollowingReason::no_reference_lane};
  }
  const auto reference_lane = lanelet_map_ptr->laneletLayer.get(reference_lane_id);
  const auto & sequence_ids = connected_sequence_ids(reference_lane, routing_graph_ptr);

  // inside_connected_sequence (docs/lane_following.md, "Inside the connected sequence").
  if (is_point_inside_any(lanelet_map_ptr, sequence_ids, ego_point)) {
    return {true, LaneFollowingReason::inside_connected_sequence};
  }

  // within_lateral_tolerance (docs/lane_following.md, "Within lateral tolerance").
  if (is_point_within_tolerance_of_any(
        lanelet_map_ptr, sequence_ids, ego_point, config_.lateral_tolerance_m)) {
    return {true, LaneFollowingReason::within_lateral_tolerance};
  }

  // road_shoulder_exempt (docs/lane_following.md, "Road-shoulder exemption").
  if (
    config_.enable_road_shoulder_exemption &&
    is_point_in_road_shoulder(lanelet_map_ptr, ego_point)) {
    return {true, LaneFollowingReason::road_shoulder_exempt};
  }

  // turn_lane_exempt (docs/lane_following.md, "Turn / intersection-lane exemption").
  if (
    config_.enable_turn_lane_exemption &&
    is_in_turn_lane(lanelet_map_ptr, reference_lane, sequence_ids, ego_point)) {
    return {true, LaneFollowingReason::turn_lane_exempt};
  }

  // virtual_boundary_exempt (docs/lane_following.md, "Virtual-boundary exemption").
  if (
    config_.enable_virtual_boundary_exemption &&
    nearest_sequence_boundary_is_virtual(lanelet_map_ptr, sequence_ids, ego_point)) {
    return {true, LaneFollowingReason::virtual_boundary_exempt};
  }

  return {false, LaneFollowingReason::departed};
}

}  // namespace autoware::lane_event_classifier
