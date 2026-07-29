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
#include <rclcpp/duration.hpp>

#include <std_msgs/msg/color_rgba.hpp>

#include <tf2/utils.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
namespace
{
using turn_indicator::LaneAttribute;
using turn_indicator::ManeuverKind;
using turn_indicator::PathPointLite;
using turn_indicator::TurnDirection;

turn_indicator::TurnDirection parse_turn_direction(const std::string & value)
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

std::string to_string(ManeuverKind kind)
{
  switch (kind) {
    case ManeuverKind::INTERSECTION:
      return "intersection";
    case ManeuverKind::PRIVATE_EXIT:
      return "private_exit";
    case ManeuverKind::PULL_OUT:
      return "pull_out";
    case ManeuverKind::PULL_OVER:
      return "pull_over";
    case ManeuverKind::NONE:
    default:
      return "none";
  }
}

std::string to_string(TurnDirection dir)
{
  switch (dir) {
    case TurnDirection::LEFT:
      return "left";
    case TurnDirection::RIGHT:
      return "right";
    case TurnDirection::NONE:
    default:
      return "off";
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

bool is_same_pose(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  constexpr double kEpsilon = 1e-3;
  return std::abs(a.position.x - b.position.x) < kEpsilon &&
         std::abs(a.position.y - b.position.y) < kEpsilon &&
         std::abs(a.orientation.z - b.orientation.z) < kEpsilon &&
         std::abs(a.orientation.w - b.orientation.w) < kEpsilon;
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

const turn_indicator::LaneAttributeMap & TurnIndicatorDecider::lane_attributes(
  const std::vector<PathPointLite> & points, const RouteContext & route_context)
{
  const auto * map = route_context.lanelet_map_ptr.get();
  if (map != cached_map_) {
    cached_map_ = map;
    lane_attribute_cache_.clear();
    neighbour_cache_.clear();
  }
  if (map == nullptr) {
    return lane_attribute_cache_;
  }

  // Only ids not seen before are looked up, so a steady route costs nothing after the first cycle.
  for (const auto & point : points) {
    for (const auto id : point.lane_ids) {
      if (lane_attribute_cache_.count(id) != 0U) {
        continue;
      }
      LaneAttribute attribute;
      try {
        const auto lanelet = map->laneletLayer.get(id);
        attribute.turn_direction = parse_turn_direction(lanelet.attributeOr("turn_direction", ""));
        attribute.is_private = std::string(lanelet.attributeOr("location", "")) == "private";
      } catch (const lanelet::NoSuchPrimitiveError &) {
        // Leave the default (no turn direction, public) for ids missing from the map.
      }
      lane_attribute_cache_[id] = attribute;
    }
  }
  return lane_attribute_cache_;
}

const lanelet::ConstLanelets & TurnIndicatorDecider::lanes_with_neighbours(
  const int64_t lane_id, const RouteContext & route_context)
{
  const auto cached = neighbour_cache_.find(lane_id);
  if (cached != neighbour_cache_.end()) {
    return cached->second;
  }

  lanelet::ConstLanelets lanes;
  const auto & map = route_context.lanelet_map_ptr;
  if (map) {
    try {
      const auto lanelet = map->laneletLayer.get(lane_id);
      lanes.push_back(lanelet);
      if (route_context.routing_graph_ptr) {
        const auto & graph = *route_context.routing_graph_ptr;
        for (const auto & neighbour :
             {graph.left(lanelet), graph.right(lanelet), graph.adjacentLeft(lanelet),
              graph.adjacentRight(lanelet)}) {
          if (neighbour) {
            lanes.push_back(*neighbour);
          }
        }
      }
    } catch (const lanelet::NoSuchPrimitiveError &) {
      // no lanes for this id
    }
  }
  return neighbour_cache_.emplace(lane_id, std::move(lanes)).first->second;
}

TurnIndicatorsCommand TurnIndicatorDecider::decide(
  const PathWithLaneId & path, const RouteContext & route_context,
  const geometry_msgs::msg::Pose & ego_pose, const double ego_velocity, const rclcpp::Time & stamp)
{
  TurnIndicatorsCommand cmd;
  cmd.stamp = stamp;

  debug_ = TurnIndicatorDebug();

  const auto points = to_lite_points(path);
  if (points.empty()) {
    cmd.command = to_command(blink_hold_.update(TurnDirection::NONE, stamp.seconds()));
    debug_.command = blink_hold_.current();
    return cmd;
  }

  const double ego_yaw = tf2::getYaw(ego_pose.orientation);
  const std::size_t ego_index =
    turn_indicator::nearest_index(points, ego_pose.position.x, ego_pose.position.y, ego_yaw);
  const auto & attributes = lane_attributes(points, route_context);

  // 1. Intersection turn (lanelet `turn_direction` tag).
  const auto turn_segments =
    turn_indicator::find_turn_segments(points, ego_index, attributes, params_);
  const auto intersection =
    turn_indicator::decide_maneuver_signal(turn_segments, ego_velocity, ego_yaw, params_);

  // 2. Private-area exit (`location=private` run rejoining a public lane). Handled separately from
  //    the turn tag because a private exit may be tagged `straight`, or not tagged at all.
  const auto private_segments =
    turn_indicator::find_private_exit_segments(points, ego_index, attributes, params_);
  const auto private_exit =
    turn_indicator::decide_maneuver_signal(private_segments, ego_velocity, ego_yaw, params_);

  // 3. Arrival (pull-over): signal toward the side the goal sits on while approaching an
  //    off-centerline goal, and clear once stopped at it.
  const double distance_to_goal = std::hypot(
    route_context.goal_pose.position.x - ego_pose.position.x,
    route_context.goal_pose.position.y - ego_pose.position.y);
  debug_.distance_to_goal = distance_to_goal;

  if (!has_cached_goal_ || !is_same_pose(cached_goal_pose_, route_context.goal_pose)) {
    // New route: recompute the (static) goal offset once and re-arm the departure/arrival states.
    cached_goal_pose_ = route_context.goal_pose;
    has_cached_goal_ = true;
    cached_goal_offset_ =
      route_context.goal_lanelets.empty()
        ? 0.0
        : lanelet::utils::getArcCoordinates(route_context.goal_lanelets, route_context.goal_pose)
            .distance;
    departure_latch_.reset();
    arrival_state_.reset();
  }
  debug_.goal_lateral_offset = cached_goal_offset_;

  const TurnDirection pull_over =
    arrival_state_.update(distance_to_goal, cached_goal_offset_, ego_velocity, params_);

  // 4. Departure (pull-out): only latches while ego stands still clear of the lane, so a lane
  //    change or avoidance (offset but moving) never lights it. Suppressed inside the pull-over
  //    range so the two cannot fight over the direction while ego pulls into an offset goal.
  const bool in_pull_over_range = distance_to_goal <= params_.pull_over_search_distance;
  static const lanelet::ConstLanelets kNoLanes{};
  const auto & ego_lanes =
    points[ego_index].lane_ids.empty()
      ? kNoLanes
      : lanes_with_neighbours(points[ego_index].lane_ids.front(), route_context);
  TurnDirection pull_out = TurnDirection::NONE;
  if (ego_lanes.empty()) {
    // No usable reference lane this cycle: keep whatever the latch already decided.
    pull_out = departure_latch_.direction();
  } else {
    const double ego_offset = lanelet::utils::getArcCoordinates(ego_lanes, ego_pose).distance;
    debug_.ego_lateral_offset = ego_offset;
    pull_out = departure_latch_.update(ego_offset, ego_velocity, in_pull_over_range, params_);
  }

  const auto decision = turn_indicator::resolve_priority(
    intersection.direction, private_exit.direction, pull_out, pull_over);
  cmd.command = to_command(blink_hold_.update(decision.direction, stamp.seconds()));

  // Record what drove the decision, for visualization.
  debug_.command = blink_hold_.current();
  debug_.kind = decision.kind;
  const auto & active_segment = decision.kind == ManeuverKind::INTERSECTION
                                  ? intersection.segment
                                  : (decision.kind == ManeuverKind::PRIVATE_EXIT
                                       ? private_exit.segment
                                       : std::optional<turn_indicator::ManeuverSegment>{});
  if (
    active_segment && active_segment->start_index < path.points.size() &&
    active_segment->end_index < path.points.size()) {
    debug_.has_segment = true;
    debug_.start_point = path.points[active_segment->start_index].point.pose.position;
    debug_.end_point = path.points[active_segment->end_index].point.pose.position;
    debug_.start_yaw = turn_indicator::path_yaw_at(points, active_segment->start_index);
    debug_.exit_yaw = active_segment->exit_yaw;
    debug_.dist_to_start = active_segment->dist_to_start;
  }

  return cmd;
}

visualization_msgs::msg::MarkerArray create_turn_indicator_markers(
  const TurnIndicatorDebug & debug, const std_msgs::msg::Header & header)
{
  using turn_indicator::TurnDirection;

  visualization_msgs::msg::MarkerArray markers;

  const auto lifetime = rclcpp::Duration::from_seconds(0.5);
  const auto colour = [&debug]() {
    std_msgs::msg::ColorRGBA c;
    c.a = 0.999;
    // amber when lit, grey when dark
    c.r = debug.command == TurnDirection::NONE ? 0.6 : 1.0;
    c.g = debug.command == TurnDirection::NONE ? 0.6 : 0.7;
    c.b = debug.command == TurnDirection::NONE ? 0.6 : 0.0;
    return c;
  }();

  if (debug.has_segment) {
    visualization_msgs::msg::Marker arrow;
    arrow.header = header;
    arrow.ns = "turn_indicator_segment";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.lifetime = lifetime;
    arrow.pose.orientation.w = 1.0;
    arrow.scale.x = 0.4;  // shaft diameter
    arrow.scale.y = 0.8;  // head diameter
    arrow.color = colour;
    arrow.points.push_back(debug.start_point);
    arrow.points.push_back(debug.end_point);
    markers.markers.push_back(arrow);
  }

  visualization_msgs::msg::Marker text;
  text.header = header;
  text.ns = "turn_indicator_state";
  text.id = 0;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::msg::Marker::ADD;
  text.lifetime = lifetime;
  text.pose.position = debug.has_segment ? debug.start_point : geometry_msgs::msg::Point{};
  text.pose.position.z += 2.0;
  text.pose.orientation.w = 1.0;
  text.scale.z = 1.0;
  text.color = colour;
  text.text = to_string(debug.command) + " (" + to_string(debug.kind) + ")";
  markers.markers.push_back(text);

  return markers;
}

}  // namespace autoware::minimum_rule_based_planner
