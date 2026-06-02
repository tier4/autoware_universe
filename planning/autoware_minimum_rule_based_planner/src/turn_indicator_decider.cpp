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

#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
namespace
{
using turn_indicator::PathPointLite;
using turn_indicator::TurnDirection;

TurnDirection parse_turn_direction(const std::string & value)
{
  if (value == "left") {
    return TurnDirection::LEFT;
  }
  if (value == "right") {
    return TurnDirection::RIGHT;
  }
  return TurnDirection::NONE;
}

uint8_t to_command(TurnDirection dir)
{
  switch (dir) {
    case TurnDirection::LEFT:
      return TurnIndicatorsCommand::ENABLE_LEFT;
    case TurnDirection::RIGHT:
      return TurnIndicatorsCommand::ENABLE_RIGHT;
    case TurnDirection::NONE:
    default:
      return TurnIndicatorsCommand::DISABLE;
  }
}

std::vector<PathPointLite> to_lite_points(const PathWithLaneId & path)
{
  std::vector<PathPointLite> points;
  points.reserve(path.points.size());
  for (const auto & p : path.points) {
    PathPointLite lite;
    lite.x = p.point.pose.position.x;
    lite.y = p.point.pose.position.y;
    lite.lane_ids.assign(p.lane_ids.begin(), p.lane_ids.end());
    points.push_back(std::move(lite));
  }
  return points;
}

//! Build lane_id -> turn direction map for every lane id referenced by the path.
// TODO(odashima): rebuilt from scratch every cycle; cache by lane_id (or per route) to speed up.
std::unordered_map<int64_t, TurnDirection> build_direction_map(
  const std::vector<PathPointLite> & points, const lanelet::LaneletMapPtr & map)
{
  std::unordered_map<int64_t, TurnDirection> direction_of;
  if (!map) {
    return direction_of;
  }
  for (const auto & point : points) {
    for (const auto id : point.lane_ids) {
      if (direction_of.count(id) != 0U) {
        continue;
      }
      try {
        const auto lanelet = map->laneletLayer.get(id);
        direction_of[id] = parse_turn_direction(lanelet.attributeOr("turn_direction", "none"));
      } catch (const lanelet::NoSuchPrimitiveError &) {
        direction_of[id] = TurnDirection::NONE;
      }
    }
  }
  return direction_of;
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
  const geometry_msgs::msg::Pose & ego_pose, double ego_velocity, const rclcpp::Time & stamp)
{
  TurnIndicatorsCommand cmd;
  cmd.stamp = stamp;

  const auto points = to_lite_points(path);
  if (points.empty()) {
    cmd.command = to_command(blink_hold_.update(TurnDirection::NONE, stamp.seconds()));
    return cmd;
  }

  const auto & map = route_context.lanelet_map_ptr;
  const std::size_t ego_index =
    turn_indicator::nearest_index(points, ego_pose.position.x, ego_pose.position.y);

  // 1. Intersection turn (lanelet turn_direction attribute).
  const auto direction_of = build_direction_map(points, map);
  const auto segment = turn_indicator::find_next_turn_segment(points, ego_index, direction_of);
  const TurnDirection intersection_dir =
    turn_indicator::decide_intersection_signal(segment, ego_velocity, params_);

  // Record the detected turn-segment geometry for debug visualization.
  debug_.has_segment = false;
  debug_.direction = TurnDirection::NONE;
  if (
    segment && segment->start_index < path.points.size() &&
    segment->end_index < path.points.size()) {
    debug_.has_segment = true;
    debug_.direction = segment->direction;
    debug_.start_point = path.points[segment->start_index].point.pose.position;
    debug_.end_point = path.points[segment->end_index].point.pose.position;
    // Heading at the start point (for the debug arrow); fall back to the previous point at the
    // path end so two distinct points are used (avoids atan2(0, 0)).
    if (segment->start_index + 1 < path.points.size()) {
      const auto & p0 = path.points[segment->start_index].point.pose.position;
      const auto & p1 = path.points[segment->start_index + 1].point.pose.position;
      debug_.start_yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
    } else if (segment->start_index > 0) {
      const auto & p0 = path.points[segment->start_index - 1].point.pose.position;
      const auto & p1 = path.points[segment->start_index].point.pose.position;
      debug_.start_yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
    } else {
      debug_.start_yaw = 0.0;  // single-point path: heading is undefined
    }
  }

  // 2./3. Pull-out and pull-over direction from the signed lateral offset (getArcCoordinates:
  // +distance = left of centerline) of the maneuver endpoint vs the main-lane centerline. The
  // offset magnitude is the trigger, so no road_shoulder attribute is needed; a lane-tracking
  // ego has offset ~0, so normal driving does not blink.
  TurnDirection pull_out_dir = TurnDirection::NONE;
  TurnDirection pull_over_dir = TurnDirection::NONE;
  const auto & reference_lanelets = route_context.preferred_lanelets;
  if (!reference_lanelets.empty()) {
    // Pull-out: ego shifts from its offset back onto the centre, so direction = -sign(ego_offset).
    const double ego_offset =
      lanelet::utils::getArcCoordinates(reference_lanelets, ego_pose).distance;
    pull_out_dir =
      turn_indicator::direction_from_lateral_offset(-ego_offset, params_.lateral_shift_threshold);

    // Pull-over: near the goal, signal toward the side the goal sits on.
    const double dist_to_goal = std::hypot(
      route_context.goal_pose.position.x - ego_pose.position.x,
      route_context.goal_pose.position.y - ego_pose.position.y);
    if (dist_to_goal <= params_.pull_over_search_distance) {
      const double goal_offset =
        lanelet::utils::getArcCoordinates(reference_lanelets, route_context.goal_pose).distance;
      pull_over_dir =
        turn_indicator::direction_from_lateral_offset(goal_offset, params_.lateral_shift_threshold);
    }
  }

  const TurnDirection desired =
    turn_indicator::resolve_priority(intersection_dir, pull_out_dir, pull_over_dir);
  cmd.command = to_command(blink_hold_.update(desired, stamp.seconds()));
  return cmd;
}

}  // namespace autoware::minimum_rule_based_planner
