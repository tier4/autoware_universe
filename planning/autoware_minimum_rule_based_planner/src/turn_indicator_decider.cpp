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

#include "turn_indicator_decider.hpp"

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware/lanelet2_utils/topology.hpp>
#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <magic_enum.hpp>
#include <rclcpp/duration.hpp>

#include <std_msgs/msg/color_rgba.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
namespace
{
namespace lanelet2_utils = autoware::experimental::lanelet2_utils;
using turn_indicator::ManeuverKind;
using turn_indicator::Signal;
using turn_indicator::TurnDirection;

uint8_t to_command(const TurnDirection direction)
{
  switch (direction) {
    case TurnDirection::LEFT:
      return TurnIndicatorsCommand::ENABLE_LEFT;
    case TurnDirection::RIGHT:
      return TurnIndicatorsCommand::ENABLE_RIGHT;
    case TurnDirection::NONE:
    default:
      return TurnIndicatorsCommand::DISABLE;
  }
}

std::string to_string(const Signal & signal)
{
  return std::string(magic_enum::enum_name(signal.direction)) + " (" +
         std::string(magic_enum::enum_name(signal.kind)) + ")";
}

bool is_private(const lanelet::ConstLanelet & lanelet)
{
  return lanelet.attributeOr(lanelet::AttributeNamesString::Location, std::string("")) ==
         lanelet::AttributeValueString::Private;
}

//! The lanelet with `id`, or nothing when the map does not hold it.
std::optional<lanelet::ConstLanelet> find_lanelet(const lanelet::LaneletMap & map, const int64_t id)
{
  try {
    return map.laneletLayer.get(id);
  } catch (const lanelet::NoSuchPrimitiveError &) {
    return std::nullopt;
  }
}

TurnDirection tagged_turn_direction(const lanelet::ConstLanelet & lanelet)
{
  const auto direction = lanelet2_utils::get_turn_direction(lanelet);
  if (direction == lanelet2_utils::TurnDirection::Left) {
    return TurnDirection::LEFT;
  }
  if (direction == lanelet2_utils::TurnDirection::Right) {
    return TurnDirection::RIGHT;
  }
  return TurnDirection::NONE;
}

//! Path heading `offset` metres along the path from `index` (signed), falling back to the path end
//! (or start) when the path is too short to reach it.
double path_yaw_at(const PathWithLaneId & path, const std::size_t index, const double offset)
{
  const auto pose = motion_utils::calcLongitudinalOffsetPose(path.points, index, offset);
  if (pose) {
    return tf2::getYaw(pose->orientation);
  }
  const auto & fallback = offset >= 0.0 ? path.points.back() : path.points.front();
  return tf2::getYaw(fallback.point.pose.orientation);
}

//! An upcoming (or currently occupied) maneuver along the path.
struct Maneuver
{
  Signal signal;
  double dist_to_start{0.0};  //!< [m] arc length from ego to its start (negative once inside)
  double exit_yaw{0.0};       //!< [rad] path heading the maneuver ends on
  std::size_t start_index{0};
  std::size_t end_index{0};
};

//! Distinct lane ids in the order they first appear along the path.
std::vector<int64_t> ordered_lane_ids(const PathWithLaneId & path)
{
  std::vector<int64_t> ids;
  for (const auto & point : path.points) {
    for (const auto id : point.lane_ids) {
      if (std::find(ids.begin(), ids.end(), id) == ids.end()) {
        ids.push_back(id);
      }
    }
  }
  return ids;
}

//! Every maneuver this module signals for, ordered from ego. Two cases, both keyed on the lanelet
//! run the path traverses:
//!  - an intersection turn: a lanelet tagged `turn_direction=left/right`;
//!  - a private-area exit: the boundary where a `location=private` lanelet rejoins a public one.
//! The private case is handled separately because such a lanelet may be tagged `straight`, or not
//! tagged at all, so the turn tag alone would miss it.
//!
//! A maneuver whose exit lies more than `k_exit_lookahead` behind ego is dropped; one ego is still
//! completing is kept, so its signal survives until the heading check clears it.
std::vector<Maneuver> find_maneuvers(
  const PathWithLaneId & path, const std::size_t ego_index, const RouteContext & route_context,
  const turn_indicator::TurnSignalParams & params)
{
  std::vector<Maneuver> maneuvers;
  const auto & map = route_context.lanelet_map_ptr;
  if (!map) {
    return maneuvers;
  }

  const auto lane_ids = ordered_lane_ids(path);
  for (std::size_t i = 0; i < lane_ids.size(); ++i) {
    const auto range = motion_utils::getPathIndexRangeWithLaneId(path, lane_ids[i]);
    if (!range) {
      continue;
    }
    const auto lanelet = find_lanelet(*map, lane_ids[i]);
    if (!lanelet) {
      continue;
    }

    // The private run ends here only if the next lanelet along the path is public (or the path
    // ends). A run of several private lanelets therefore yields exactly one exit. An id the map
    // does not hold counts as public, so a missing lanelet cannot swallow the exit.
    const auto next = i + 1 < lane_ids.size() ? find_lanelet(*map, lane_ids[i + 1]) : std::nullopt;
    const bool leaves_private_area = is_private(*lanelet) && (!next || !is_private(*next));
    const auto turn_direction = tagged_turn_direction(*lanelet);
    if (turn_direction == TurnDirection::NONE && !leaves_private_area) {
      continue;
    }

    Maneuver maneuver;
    // A turn is signalled from the entry of its lanelet; a private exit from the boundary itself,
    // which is where the maneuver actually starts (the private run leading up to it may be long).
    maneuver.start_index = turn_direction != TurnDirection::NONE ? range->first : range->second;
    maneuver.end_index = range->second;
    maneuver.exit_yaw = path_yaw_at(path, range->second, turn_indicator::k_exit_lookahead);
    maneuver.dist_to_start =
      motion_utils::calcSignedArcLength(path.points, ego_index, maneuver.start_index);

    if (turn_direction != TurnDirection::NONE) {
      maneuver.signal = {turn_direction, ManeuverKind::INTERSECTION};
    } else {
      // Untagged private exit: take the side from the yaw change across the merge, measured from a
      // look-ahead BEFORE the boundary - right at it the path is already turning, so the local yaw
      // change is near zero and every merge would read as straight.
      const double approach_yaw =
        path_yaw_at(path, range->second, -turn_indicator::k_exit_lookahead);
      const double delta = autoware_utils_math::normalize_radian(maneuver.exit_yaw - approach_yaw);
      if (std::abs(delta) <= params.heading_align_threshold) {
        continue;  // a geometrically straight merge has no side to signal
      }
      maneuver.signal = {
        delta > 0.0 ? TurnDirection::LEFT : TurnDirection::RIGHT, ManeuverKind::PRIVATE_EXIT};
    }

    const double dist_to_end =
      motion_utils::calcSignedArcLength(path.points, ego_index, maneuver.end_index);
    if (dist_to_end + turn_indicator::k_exit_lookahead <= 0.0) {
      continue;  // fully behind ego
    }
    maneuvers.push_back(maneuver);
  }
  return maneuvers;
}

//! The lane ego is expected to track plus its lateral neighbours, so that a lane change (whose
//! target lane is not the path's lane) still reports a near-zero offset and cannot raise a signal.
lanelet::ConstLanelets ego_lanes(
  const PathWithLaneId & path, const std::size_t ego_index, const RouteContext & route_context)
{
  const auto & lane_ids = path.points.at(ego_index).lane_ids;
  if (lane_ids.empty() || !route_context.lanelet_map_ptr) {
    return {};
  }
  try {
    const auto lanelet = route_context.lanelet_map_ptr->laneletLayer.get(lane_ids.front());
    if (!route_context.routing_graph_ptr) {
      return {lanelet};
    }
    return lanelet2_utils::all_neighbor_lanelets(lanelet, route_context.routing_graph_ptr);
  } catch (const lanelet::NoSuchPrimitiveError &) {
    return {};
  }
}

}  // namespace

TurnIndicatorDecider::TurnIndicatorDecider(const turn_indicator::TurnSignalParams & params)
: params_(params), blink_hold_(params.min_blink_duration)
{
}

void TurnIndicatorDecider::update_params(const turn_indicator::TurnSignalParams & params)
{
  params_ = params;
  blink_hold_.set_min_duration(params.min_blink_duration);
}

TurnIndicatorsCommand TurnIndicatorDecider::decide(
  const PathWithLaneId & path, const RouteContext & route_context,
  const geometry_msgs::msg::Pose & ego_pose, const double ego_velocity, const rclcpp::Time & stamp)
{
  debug_ = TurnIndicatorDebug();
  TurnIndicatorsCommand cmd;
  cmd.stamp = stamp;

  if (path.points.empty()) {
    cmd.command = to_command(blink_hold_.update(TurnDirection::NONE, stamp.seconds()));
    debug_.signal.direction = blink_hold_.current();
    return cmd;
  }

  // Re-arm the latched pull-out / pull-over states whenever the route changes.
  if (
    !latched_goal_pose_ ||
    autoware_utils_geometry::calc_distance2d(*latched_goal_pose_, route_context.goal_pose) > 1e-3) {
    latched_goal_pose_ = route_context.goal_pose;
    pull_out_latch_ = TurnDirection::NONE;
    arrived_at_goal_ = false;
  }

  // Reject path points the path traverses in (roughly) the opposite direction to ego, so a route
  // folding back on itself (U-turn, loop) cannot snap ego onto the wrong leg.
  const double ego_yaw = tf2::getYaw(ego_pose.orientation);
  const auto aligned_index = motion_utils::findNearestIndex(
    path.points, ego_pose, std::numeric_limits<double>::max(), M_PI_2);
  const std::size_t ego_index =
    aligned_index.value_or(motion_utils::findNearestIndex(path.points, ego_pose.position));

  // 1./2. Intersection turns and private-area exits, both keyed on the path's lanelets. The
  //       nearest lit maneuver of each kind competes; an intersection outranks a private exit.
  const Maneuver * lit_intersection = nullptr;
  const Maneuver * lit_private_exit = nullptr;
  const auto maneuvers = find_maneuvers(path, ego_index, route_context, params_);
  for (const auto & maneuver : maneuvers) {
    const auto direction = turn_indicator::decide_maneuver_signal(
      maneuver.signal.direction, maneuver.dist_to_start, ego_yaw, maneuver.exit_yaw, ego_velocity,
      params_);
    if (direction == TurnDirection::NONE) {
      continue;
    }
    const Maneuver *& slot =
      maneuver.signal.kind == ManeuverKind::INTERSECTION ? lit_intersection : lit_private_exit;
    if (slot == nullptr) {
      slot = &maneuver;
    }
  }

  // 3. Arrival (pull-over): signal toward the side an off-centerline goal sits on.
  const double dist_to_goal =
    autoware_utils_geometry::calc_distance2d(ego_pose, route_context.goal_pose);
  const double goal_offset = route_context.goal_lanelets.empty()
                               ? 0.0
                               : lanelet2_utils::get_lateral_distance_to_centerline(
                                   route_context.goal_lanelets, route_context.goal_pose);
  const auto pull_over = turn_indicator::decide_pull_over(
    dist_to_goal, goal_offset, ego_velocity, params_, arrived_at_goal_);

  // 4. Departure (pull-out). Suppressed inside the pull-over range so the two cannot fight over
  //    the direction while ego pulls into an offset goal.
  const auto lanes = ego_lanes(path, ego_index, route_context);
  const auto pull_out =
    lanes.empty()
      // No usable reference lane this cycle: keep whatever the latch already decided.
      ? pull_out_latch_
      : turn_indicator::decide_pull_out(
          lanelet2_utils::get_lateral_distance_to_centerline(lanes, ego_pose), ego_velocity,
          dist_to_goal <= params_.search_distance, params_, pull_out_latch_);

  const auto signal = turn_indicator::resolve_priority(
    {lit_intersection != nullptr ? lit_intersection->signal : Signal{},
     lit_private_exit != nullptr ? lit_private_exit->signal : Signal{},
     Signal{pull_out, ManeuverKind::PULL_OUT}, Signal{pull_over, ManeuverKind::PULL_OVER}});
  cmd.command = to_command(blink_hold_.update(signal.direction, stamp.seconds()));

  // Record what drove the decision, for visualization.
  debug_.signal = {blink_hold_.current(), signal.kind};
  const Maneuver * active = signal.kind == ManeuverKind::INTERSECTION   ? lit_intersection
                            : signal.kind == ManeuverKind::PRIVATE_EXIT ? lit_private_exit
                                                                        : nullptr;
  if (active != nullptr) {
    debug_.has_segment = true;
    debug_.start_point = path.points.at(active->start_index).point.pose.position;
    debug_.end_point = path.points.at(active->end_index).point.pose.position;
  }

  return cmd;
}

visualization_msgs::msg::MarkerArray create_turn_indicator_markers(
  const TurnIndicatorDebug & debug, const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray markers;

  const bool lit = debug.signal.direction != turn_indicator::TurnDirection::NONE;
  std_msgs::msg::ColorRGBA colour;  // amber when lit, grey when dark
  colour.a = 0.999;
  colour.r = lit ? 1.0 : 0.6;
  colour.g = lit ? 0.7 : 0.6;
  colour.b = lit ? 0.0 : 0.6;

  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  marker.pose.orientation.w = 1.0;
  marker.color = colour;

  if (debug.has_segment) {
    marker.ns = "turn_indicator_segment";
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.scale.x = 0.4;  // shaft diameter
    marker.scale.y = 0.8;  // head diameter
    marker.points = {debug.start_point, debug.end_point};
    markers.markers.push_back(marker);
    marker.points.clear();
  }

  marker.ns = "turn_indicator_state";
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.pose.position = debug.start_point;
  marker.pose.position.z += 2.0;
  marker.scale = geometry_msgs::msg::Vector3();
  marker.scale.z = 1.0;
  marker.text = to_string(debug.signal);
  markers.markers.push_back(marker);

  return markers;
}

}  // namespace autoware::minimum_rule_based_planner
