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

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/topology.hpp>
#include <autoware_lane_event_classifier/geometry_utils.hpp>
#include <autoware_lane_event_classifier/lane_tracker.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BoundingBox.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

namespace autoware::lane_event_classifier
{

namespace lanelet2_utils = autoware::experimental::lanelet2_utils;

tl::expected<void, std::string> LaneTracker::set_lanelet_map(
  const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  if (!lanelet_map_ptr) {
    return tl::make_unexpected("LaneTracker: lanelet map is null");
  }
  if (lanelet_map_ptr->laneletLayer.empty()) {
    return tl::make_unexpected("LaneTracker: lanelet map has no lanelets");
  }
  lanelet_map_ptr_ = lanelet_map_ptr;
  std::tie(routing_graph_ptr_, traffic_rules_ptr_) =
    autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
      lanelet_map_ptr_);
  // The routing graph changed, so any memoized straight lane sequence is stale.
  cached_lane_sequence_source_id_ = lanelet::InvalId;
  cached_lane_sequence_look_ahead_m_ = -1.0;
  cached_lane_sequence_ids_.clear();
  return {};
}

void LaneTracker::release_reference_lane()
{
  is_reference_lane_held_ = false;
  reference_lane_ = {};
}

bool LaneTracker::lanelet_exists(lanelet::Id lane_id) const
{
  return lanelet_map_ptr_ && lane_id != lanelet::InvalId &&
         lanelet_map_ptr_->laneletLayer.exists(lane_id);
}

std::optional<lanelet::ConstLanelet> LaneTracker::get_lanelet(lanelet::Id lane_id) const
{
  if (!lanelet_exists(lane_id)) {
    return std::nullopt;
  }
  return lanelet_map_ptr_->laneletLayer.get(lane_id);
}

std::optional<double> LaneTracker::distance_to_lane(
  lanelet::Id lane_id, const lanelet::BasicPoint2d & point) const
{
  if (!lanelet_exists(lane_id)) {
    return std::nullopt;
  }
  const auto lane = lanelet_map_ptr_->laneletLayer.get(lane_id);
  return lanelet::geometry::distance2d(lane, point);
}

std::vector<lanelet::Id> LaneTracker::lanelet_ids_at(const lanelet::BasicPoint2d & point) const
{
  std::vector<lanelet::Id> ids;
  if (!lanelet_map_ptr_) {
    return ids;
  }
  // R-tree query then precise inside test — avoids scanning the whole map.
  const lanelet::BoundingBox2d query_box(point, point);
  for (const auto & lane : lanelet_map_ptr_->laneletLayer.search(query_box)) {
    if (lanelet::geometry::inside(lane, point)) {
      ids.push_back(lane.id());
    }
  }

  return ids;
}

std::vector<lanelet::Id> LaneTracker::footprint_lane_ids(
  const std::vector<lanelet::BasicPoint2d> & footprint) const
{
  std::vector<lanelet::Id> ids;
  if (!lanelet_map_ptr_ || footprint.empty()) {
    return ids;
  }
  // R-tree query on the footprint bbox then per-corner inside test — avoids a whole-map scan.
  lanelet::BasicPoint2d min_corner = footprint.front();
  lanelet::BasicPoint2d max_corner = footprint.front();
  for (const auto & corner : footprint) {
    min_corner.x() = std::min(min_corner.x(), corner.x());
    min_corner.y() = std::min(min_corner.y(), corner.y());
    max_corner.x() = std::max(max_corner.x(), corner.x());
    max_corner.y() = std::max(max_corner.y(), corner.y());
  }
  const lanelet::BoundingBox2d query_box(min_corner, max_corner);
  for (const auto & lane : lanelet_map_ptr_->laneletLayer.search(query_box)) {
    const bool any_corner_inside = std::any_of(
      footprint.cbegin(), footprint.cend(), [&lane](const lanelet::BasicPoint2d & corner) {
        return lanelet::geometry::inside(lane, corner);
      });
    if (any_corner_inside) {
      ids.push_back(lane.id());
    }
  }
  return ids;
}

std::vector<lanelet::Id> LaneTracker::next_lane_ids(lanelet::Id lane_id) const
{
  std::vector<lanelet::Id> ids;
  const auto lane = get_lanelet(lane_id);
  if (!routing_graph_ptr_ || !lane) {
    return ids;
  }
  for (const auto & next_lane : routing_graph_ptr_->following(*lane)) {
    ids.push_back(next_lane.id());
  }
  return ids;
}

const std::unordered_set<lanelet::Id> & LaneTracker::straight_lane_sequence_ids(
  const lanelet::ConstLanelet & lane, double look_ahead_m) const
{
  // Reuse the memoized lane sequence while the queried lane and look-ahead are unchanged;
  // look_ahead_m is a fixed configuration value, so the exact comparison always holds on a cache
  // hit.
  if (
    lane.id() == cached_lane_sequence_source_id_ &&
    look_ahead_m == cached_lane_sequence_look_ahead_m_) {
    return cached_lane_sequence_ids_;
  }

  cached_lane_sequence_ids_.clear();
  cached_lane_sequence_ids_.insert(lane.id());
  if (routing_graph_ptr_) {
    const auto insert_sequence_ids = [this](const auto & sequences) {
      for (const auto & sequence : sequences) {
        for (const auto & sequence_lane : sequence) {
          cached_lane_sequence_ids_.insert(sequence_lane.id());
        }
      }
    };
    std::invoke(
      insert_sequence_ids,
      lanelet2_utils::get_succeeding_lanelet_sequences(lane, routing_graph_ptr_, look_ahead_m));
    std::invoke(
      insert_sequence_ids,
      lanelet2_utils::get_preceding_lanelet_sequences(lane, routing_graph_ptr_, look_ahead_m));
  }
  cached_lane_sequence_source_id_ = lane.id();
  cached_lane_sequence_look_ahead_m_ = look_ahead_m;
  return cached_lane_sequence_ids_;
}

bool LaneTracker::is_footprint_fully_inside_lane(
  lanelet::Id lane_id, const std::vector<lanelet::BasicPoint2d> & footprint) const
{
  const auto lane = get_lanelet(lane_id);
  if (!lane) {
    return false;
  }
  return autoware::lane_event_classifier::is_footprint_fully_inside_lane(*lane, footprint);
}

bool LaneTracker::is_route_primitive(lanelet::Id lane_id) const
{
  return route_primitive_ids_.count(lane_id) != 0;
}

bool LaneTracker::is_lane_directly_connected(
  lanelet::Id from_lane_id, lanelet::Id candidate_lane_id) const
{
  if (!routing_graph_ptr_ || !lanelet_exists(from_lane_id)) {
    return false;
  }
  const auto from_lane = lanelet_map_ptr_->laneletLayer.get(from_lane_id);
  const auto next_lanes = routing_graph_ptr_->following(from_lane);
  return std::any_of(
    next_lanes.cbegin(), next_lanes.cend(),
    [candidate_lane_id](const lanelet::ConstLanelet & next_lane) {
      return next_lane.id() == candidate_lane_id;
    });
}

std::optional<lanelet::Id> LaneTracker::select_current_lane_id(const LaneEventInput & input) const
{
  const auto & odom_pos = input.odometry_ptr->pose.pose.position;
  const lanelet::BasicPoint2d ego_pt{odom_pos.x, odom_pos.y};

  // Prefer a route primitive the ego is inside; R-tree query avoids a whole-map scan each cycle.
  const lanelet::BoundingBox2d query_box(ego_pt, ego_pt);
  std::optional<lanelet::Id> off_route_lane_id;
  for (const auto & candidate_lane : lanelet_map_ptr_->laneletLayer.search(query_box)) {
    if (!lanelet::geometry::inside(candidate_lane, ego_pt)) {
      continue;
    }
    if (is_route_primitive(candidate_lane.id())) {
      return candidate_lane.id();
    }
    if (!off_route_lane_id) {
      off_route_lane_id = candidate_lane.id();  // off-route lane the ego sits in
    }
  }
  if (off_route_lane_id) {
    return off_route_lane_id;
  }

  // Fall back to the nearest route primitive (ego between lanelets, e.g. at a boundary).
  std::optional<lanelet::Id> nearest_lane_id;
  double min_dist_to_nearest_lane = std::numeric_limits<double>::max();
  for (const auto & segment : input.route_ptr->segments) {
    const auto candidate_lane_id = static_cast<lanelet::Id>(segment.preferred_primitive.id);
    if (!lanelet_exists(candidate_lane_id)) {
      continue;
    }
    const auto candidate_lane = lanelet_map_ptr_->laneletLayer.get(candidate_lane_id);
    const double dist_to_candidate = lanelet::geometry::distance2d(candidate_lane, ego_pt);
    if (dist_to_candidate < min_dist_to_nearest_lane) {
      min_dist_to_nearest_lane = dist_to_candidate;
      nearest_lane_id = candidate_lane_id;
    }
  }
  return nearest_lane_id;
}

void LaneTracker::update(const LaneEventInput & input)
{
  refresh_route_primitive_cache(input);
  is_last_reanchor_blocked_ = false;
  if (!is_reference_lane_held_) {
    refresh_reference_lane(input);
  }
}

void LaneTracker::refresh_route_primitive_cache(const LaneEventInput & input)
{
  // The node only replaces route_ptr on a new route (uuid change), so pointer identity is a stable
  // cache key: rebuild the primitive-id set only when it flips.
  if (input.route_ptr == cached_route_ptr_) {
    return;
  }
  cached_route_ptr_ = input.route_ptr;
  route_primitive_ids_.clear();
  if (!input.route_ptr) {
    return;
  }
  route_primitive_ids_.reserve(input.route_ptr->segments.size());
  for (const auto & segment : input.route_ptr->segments) {
    route_primitive_ids_.insert(static_cast<lanelet::Id>(segment.preferred_primitive.id));
  }
}

void LaneTracker::refresh_reference_lane(const LaneEventInput & input)
{
  const auto selected_lane_id = select_current_lane_id(input);
  last_selected_lane_id_ = selected_lane_id.value_or(lanelet::InvalId);
  if (!selected_lane_id) {
    return;  // keep the previous reference lane if we cannot resolve a lane this cycle
  }
  const lanelet::Id current_lane_id = *selected_lane_id;

  // Re-anchor on the first selection, or when the ego advances into a next lane.
  // A lateral move keeps the reference lane, so a lane change is not misread as forward progress.
  const bool is_reference_lane_unset = reference_lane_.reference_lane_id == lanelet::InvalId;
  const bool is_advancing_to_next_lane =
    !is_reference_lane_unset && current_lane_id != reference_lane_.reference_lane_id &&
    is_lane_directly_connected(reference_lane_.reference_lane_id, current_lane_id);
  if (!is_reference_lane_unset && !is_advancing_to_next_lane) {
    // Ego is in a lane that is neither the reference lane nor a next lane of it: the reference lane
    // cannot advance.
    is_last_reanchor_blocked_ = current_lane_id != reference_lane_.reference_lane_id;
    return;
  }

  const bool is_current_lane_on_route = is_route_primitive(current_lane_id);
  reference_lane_.reference_lane_id = current_lane_id;
  reference_lane_.is_reference_lane_on_route = is_current_lane_on_route;
}

}  // namespace autoware::lane_event_classifier
