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

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

namespace turn_indicator
{

double activation_distance(const double ego_velocity, const TurnSignalParams & params)
{
  return std::max(ego_velocity * k_search_time, params.search_distance);
}

TurnDirection direction_from_lateral_offset(const double signed_offset, const double deadzone)
{
  if (signed_offset > deadzone) {
    return TurnDirection::LEFT;
  }
  if (signed_offset < -deadzone) {
    return TurnDirection::RIGHT;
  }
  return TurnDirection::NONE;
}

Signal resolve_priority(const std::initializer_list<Signal> candidates)
{
  for (const auto & candidate : candidates) {
    if (candidate.direction != TurnDirection::NONE) {
      return candidate;
    }
  }
  return {};
}

TurnDirection decide_maneuver_signal(
  const TurnDirection direction, const double dist_to_start, const double ego_yaw,
  const double exit_yaw, const double ego_velocity, const TurnSignalParams & params)
{
  if (direction == TurnDirection::NONE) {
    return TurnDirection::NONE;
  }

  constexpr double entered_tolerance = 1e-3;
  const double yaw_gap = std::abs(autoware_utils_math::normalize_radian(ego_yaw - exit_yaw));
  if (dist_to_start <= entered_tolerance && yaw_gap <= params.heading_align_threshold) {
    return TurnDirection::NONE;
  }
  if (dist_to_start <= activation_distance(ego_velocity, params)) {
    return direction;
  }
  return TurnDirection::NONE;
}

TurnDirection decide_pull_out(
  const double signed_offset, const double ego_velocity, const bool suppressed,
  const TurnSignalParams & params, TurnDirection & latched)
{
  const double offset_magnitude = std::abs(signed_offset);

  if (suppressed || offset_magnitude <= k_lateral_shift_threshold) {
    latched = TurnDirection::NONE;
    return TurnDirection::NONE;
  }

  if (latched == TurnDirection::NONE) {
    const bool stopped = std::abs(ego_velocity) <= params.stopped_velocity_threshold;
    if (!stopped || offset_magnitude <= k_departure_lateral_threshold) {
      return TurnDirection::NONE;
    }
    latched = direction_from_lateral_offset(-signed_offset, k_lateral_shift_threshold);
  }

  return latched;
}

TurnDirection decide_pull_over(
  const double dist_to_goal, const double goal_offset, const double ego_velocity,
  const TurnSignalParams & params, bool & arrived)
{
  if (dist_to_goal > params.search_distance) {
    arrived = false;
    return TurnDirection::NONE;
  }

  if (
    dist_to_goal <= k_goal_arrival_distance &&
    std::abs(ego_velocity) <= params.stopped_velocity_threshold) {
    arrived = true;
  }
  if (arrived) {
    return TurnDirection::NONE;
  }

  return direction_from_lateral_offset(goal_offset, k_lateral_shift_threshold);
}

TurnDirection BlinkHold::update(const TurnDirection desired, const double now)
{
  if (desired == held_) {
    return held_;
  }

  const bool turning_off = desired == TurnDirection::NONE;
  if (turning_off && held_ != TurnDirection::NONE && (now - held_since_) < min_duration_) {
    return held_;
  }

  held_ = desired;
  held_since_ = now;
  return held_;
}

}  // namespace turn_indicator

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

bool is_private(const lanelet::ConstLanelet & lanelet)
{
  return lanelet.attributeOr(lanelet::AttributeNamesString::Location, std::string("")) ==
         lanelet::AttributeValueString::Private;
}

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

//! Path heading `offset` metres (signed) along the path from `index`, clamped to the path ends.
double path_yaw_at(const PathWithLaneId & path, const std::size_t index, const double offset)
{
  const auto pose = motion_utils::calcLongitudinalOffsetPose(path.points, index, offset);
  if (pose) {
    return tf2::getYaw(pose->orientation);
  }
  const auto & fallback = offset >= 0.0 ? path.points.back() : path.points.front();
  return tf2::getYaw(fallback.point.pose.orientation);
}

struct Maneuver
{
  Signal signal;
  double dist_to_start{0.0};  //!< [m] arc length from ego to its start (negative once inside)
  double exit_yaw{0.0};       //!< [rad] path heading the maneuver ends on
};

//! Distinct lane ids in first-appearance order.
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

//! Maneuvers along the path, ordered from ego. Two cases:
//!  - an intersection turn: a lanelet tagged `turn_direction=left/right`;
//!  - a private-area exit: the boundary where a `location=private` lanelet rejoins a public one.
//! The private case needs its own detection because such a lanelet is usually untagged in practice
//! (or tagged `straight`), so the turn tag alone would miss the merge.
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

    // A run of several private lanelets yields exactly one exit: the last one before a public
    // lanelet (an id the map does not hold counts as public, so it cannot swallow the exit).
    const auto next = i + 1 < lane_ids.size() ? find_lanelet(*map, lane_ids[i + 1]) : std::nullopt;
    const bool leaves_private_area = is_private(*lanelet) && (!next || !is_private(*next));
    const auto turn_direction = tagged_turn_direction(*lanelet);
    if (turn_direction == TurnDirection::NONE && !leaves_private_area) {
      continue;
    }

    // A turn starts at the entry of its lanelet; a private exit at the boundary itself (the
    // private run leading up to it may be long).
    const std::size_t start_index =
      turn_direction != TurnDirection::NONE ? range->first : range->second;

    Maneuver maneuver;
    maneuver.exit_yaw = path_yaw_at(path, range->second, turn_indicator::k_exit_lookahead);
    maneuver.dist_to_start = motion_utils::calcSignedArcLength(path.points, ego_index, start_index);

    if (turn_direction != TurnDirection::NONE) {
      maneuver.signal = {turn_direction, ManeuverKind::TURN};
    } else {
      // Untagged private exit: the side comes from the yaw change across the merge, measured
      // from before the boundary - at it the path is already turning, so it reads as straight.
      const double approach_yaw =
        path_yaw_at(path, range->second, -turn_indicator::k_exit_lookahead);
      const double delta = autoware_utils_math::normalize_radian(maneuver.exit_yaw - approach_yaw);
      if (std::abs(delta) <= params.heading_align_threshold) {
        continue;  // a geometrically straight merge has no side to signal
      }
      maneuver.signal = {
        delta > 0.0 ? TurnDirection::LEFT : TurnDirection::RIGHT, ManeuverKind::TURN};
    }

    const double dist_to_end =
      motion_utils::calcSignedArcLength(path.points, ego_index, range->second);
    if (dist_to_end + turn_indicator::k_exit_lookahead <= 0.0) {
      continue;  // fully behind ego
    }
    maneuvers.push_back(maneuver);
  }
  return maneuvers;
}

//! The lane ego tracks plus its lateral neighbours, so a lane change (whose target lane is not the
//! path's lane) still reports a near-zero offset and cannot raise a signal.
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
  TurnIndicatorsCommand cmd;
  cmd.stamp = stamp;

  if (path.points.empty()) {
    cmd.command = to_command(blink_hold_.update(TurnDirection::NONE, stamp.seconds()));
    return cmd;
  }

  // Re-arm the latched pull-out / pull-over states on a route change.
  if (
    !latched_goal_pose_ ||
    autoware_utils_geometry::calc_distance2d(*latched_goal_pose_, route_context.goal_pose) > 1e-3) {
    latched_goal_pose_ = route_context.goal_pose;
    pull_out_latch_ = TurnDirection::NONE;
    arrived_at_goal_ = false;
  }

  // The yaw limit rejects points facing away from ego, so a route folding back on itself (U-turn,
  // loop) cannot snap ego onto the wrong leg.
  const double ego_yaw = tf2::getYaw(ego_pose.orientation);
  const auto aligned_index = motion_utils::findNearestIndex(
    path.points, ego_pose, std::numeric_limits<double>::max(), M_PI_2);
  const std::size_t ego_index =
    aligned_index.value_or(motion_utils::findNearestIndex(path.points, ego_pose.position));

  // Maneuvers come back in path order, so the first one asking for a light is the one ego reaches
  // first - or is still completing, which keeps its signal from being stolen by the next one.
  Signal maneuver_signal;
  for (const auto & maneuver : find_maneuvers(path, ego_index, route_context, params_)) {
    const auto direction = turn_indicator::decide_maneuver_signal(
      maneuver.signal.direction, maneuver.dist_to_start, ego_yaw, maneuver.exit_yaw, ego_velocity,
      params_);
    if (direction != TurnDirection::NONE) {
      maneuver_signal = maneuver.signal;
      break;
    }
  }

  const double dist_to_goal =
    autoware_utils_geometry::calc_distance2d(ego_pose, route_context.goal_pose);
  const double goal_offset = route_context.goal_lanelets.empty()
                               ? 0.0
                               : lanelet2_utils::get_lateral_distance_to_centerline(
                                   route_context.goal_lanelets, route_context.goal_pose);
  const auto pull_over = turn_indicator::decide_pull_over(
    dist_to_goal, goal_offset, ego_velocity, params_, arrived_at_goal_);

  // Pull-out is suppressed inside the pull-over range so the two cannot fight over the direction.
  const auto lanes = ego_lanes(path, ego_index, route_context);
  const auto pull_out =
    lanes.empty()
      // No usable reference lane this cycle: keep whatever the latch already decided.
      ? pull_out_latch_
      : turn_indicator::decide_pull_out(
          lanelet2_utils::get_lateral_distance_to_centerline(lanes, ego_pose), ego_velocity,
          dist_to_goal <= params_.search_distance, params_, pull_out_latch_);

  const auto signal = turn_indicator::resolve_priority(
    {maneuver_signal, Signal{pull_out, ManeuverKind::PULL_OUT},
     Signal{pull_over, ManeuverKind::PULL_OVER}});
  cmd.command = to_command(blink_hold_.update(signal.direction, stamp.seconds()));

  return cmd;
}

}  // namespace autoware::minimum_rule_based_planner
