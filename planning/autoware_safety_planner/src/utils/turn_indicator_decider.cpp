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

#include "sl_view_utils.hpp"

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware/lanelet2_utils/topology.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <rclcpp/time.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autoware::safety_planner
{

namespace
{
namespace lanelet2_utils = autoware::experimental::lanelet2_utils;

constexpr uint8_t DISABLE = TurnIndicatorsCommand::DISABLE;
constexpr uint8_t LEFT = TurnIndicatorsCommand::ENABLE_LEFT;
constexpr uint8_t RIGHT = TurnIndicatorsCommand::ENABLE_RIGHT;

constexpr double SEARCH_TIME_S = 3.0;
constexpr double EXIT_LOOKAHEAD_M = 15.0;
constexpr double GOAL_ARRIVAL_DISTANCE_M = 1.0;
constexpr double LATERAL_SHIFT_THRESHOLD_M = 0.5;
// Must exceed the lateral deviation a lane change or avoidance produces, or those blink too
constexpr double DEPARTURE_LATERAL_THRESHOLD_M = 1.5;

double activation_distance(const double ego_velocity, const TurnSignalParams & params)
{
  return std::max(ego_velocity * SEARCH_TIME_S, params.search_distance);
}

uint8_t direction_from_lateral_offset(const double signed_offset, const double deadzone)
{
  if (signed_offset > deadzone) {
    return LEFT;
  }
  if (signed_offset < -deadzone) {
    return RIGHT;
  }
  return DISABLE;
}

uint8_t decide_maneuver_signal(
  const uint8_t direction, const double dist_to_start, const double ego_yaw, const double exit_yaw,
  const double ego_velocity, const TurnSignalParams & params)
{
  constexpr double entered_tolerance = 1e-3;
  const double yaw_gap = std::abs(autoware_utils_math::normalize_radian(ego_yaw - exit_yaw));
  if (dist_to_start <= entered_tolerance && yaw_gap <= params.heading_align_threshold) {
    return DISABLE;
  }
  if (dist_to_start <= activation_distance(ego_velocity, params)) {
    return direction;
  }
  return DISABLE;
}

// Latches only while ego stands still, so a lane change or avoidance (offset, but moving) never
// raises a signal
uint8_t decide_pull_out(
  const double signed_offset, const double ego_velocity, const bool suppressed,
  const TurnSignalParams & params, uint8_t & latched)
{
  const double offset_magnitude = std::abs(signed_offset);

  if (suppressed || offset_magnitude <= LATERAL_SHIFT_THRESHOLD_M) {
    latched = DISABLE;
    return DISABLE;
  }

  if (latched == DISABLE) {
    const bool stopped = std::abs(ego_velocity) <= params.stopped_velocity_threshold;
    if (!stopped || offset_magnitude <= DEPARTURE_LATERAL_THRESHOLD_M) {
      return DISABLE;
    }
    latched = direction_from_lateral_offset(-signed_offset, LATERAL_SHIFT_THRESHOLD_M);
  }

  return latched;
}

uint8_t decide_pull_over(
  const double dist_to_goal, const double goal_offset, const double ego_velocity,
  const TurnSignalParams & params, bool & arrived)
{
  if (dist_to_goal > params.search_distance) {
    arrived = false;
    return DISABLE;
  }

  if (
    dist_to_goal <= GOAL_ARRIVAL_DISTANCE_M &&
    std::abs(ego_velocity) <= params.stopped_velocity_threshold) {
    arrived = true;
  }
  if (arrived) {
    return DISABLE;
  }

  return direction_from_lateral_offset(goal_offset, LATERAL_SHIFT_THRESHOLD_M);
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

uint8_t tagged_turn_direction(const lanelet::ConstLanelet & lanelet)
{
  const auto direction = lanelet2_utils::get_turn_direction(lanelet);
  if (direction == lanelet2_utils::TurnDirection::Left) {
    return LEFT;
  }
  if (direction == lanelet2_utils::TurnDirection::Right) {
    return RIGHT;
  }
  return DISABLE;
}

double path_yaw_at(const PathPointTrajectory & path, const double s)
{
  return path.azimuth(std::clamp(s, 0.0, path.length()));
}

struct Maneuver
{
  uint8_t direction{DISABLE};
  double dist_to_start{0.0};
  double exit_yaw{0.0};
};

struct LaneRange
{
  int64_t id{0};
  double s_first{0.0};
  double s_last{0.0};
};

std::vector<LaneRange> lane_ranges(const PathPointTrajectory & path)
{
  std::vector<LaneRange> ranges;
  const auto points = path.restore();
  const auto bases = path.get_underlying_bases();
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (const auto id : points[i].lane_ids) {
      const auto it = std::find_if(
        ranges.begin(), ranges.end(), [id](const LaneRange & range) { return range.id == id; });
      if (it == ranges.end()) {
        ranges.push_back({id, bases[i], bases[i]});
      } else {
        it->s_last = bases[i];
      }
    }
  }
  return ranges;
}

// A private-area exit is detected on its own because such a lanelet is usually untagged (or tagged
// straight), so the turn_direction tag alone would miss the merge
std::vector<Maneuver> find_maneuvers(
  const PathPointTrajectory & path, const double s_ego, const lanelet::LaneletMap & map,
  const TurnSignalParams & params)
{
  std::vector<Maneuver> maneuvers;
  const auto ranges = lane_ranges(path);
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const auto lanelet = find_lanelet(map, ranges[i].id);
    if (!lanelet) {
      continue;
    }

    // An id the map does not hold counts as public, so it cannot swallow the exit
    const auto next = i + 1 < ranges.size() ? find_lanelet(map, ranges[i + 1].id) : std::nullopt;
    const bool leaves_private_area = is_private(*lanelet) && (!next || !is_private(*next));
    const auto turn_direction = tagged_turn_direction(*lanelet);
    if (turn_direction == DISABLE && !leaves_private_area) {
      continue;
    }

    // A private exit starts at the boundary, not at the lanelet entry: the private run may be long
    const double s_start = turn_direction != DISABLE ? ranges[i].s_first : ranges[i].s_last;
    const double s_end = ranges[i].s_last;

    Maneuver maneuver;
    maneuver.exit_yaw = path_yaw_at(path, s_end + EXIT_LOOKAHEAD_M);
    maneuver.dist_to_start = s_start - s_ego;

    if (turn_direction != DISABLE) {
      maneuver.direction = turn_direction;
    } else {
      // Measured from before the boundary: at it the path is already turning and reads as straight
      const double approach_yaw = path_yaw_at(path, s_end - EXIT_LOOKAHEAD_M);
      const double delta = autoware_utils_math::normalize_radian(maneuver.exit_yaw - approach_yaw);
      if (std::abs(delta) <= params.heading_align_threshold) {
        continue;  // a geometrically straight merge has no side to signal
      }
      maneuver.direction = delta > 0.0 ? LEFT : RIGHT;
    }

    if (s_end - s_ego + EXIT_LOOKAHEAD_M <= 0.0) {
      continue;  // fully behind ego
    }
    maneuvers.push_back(maneuver);
  }
  return maneuvers;
}

}  // namespace

TurnIndicatorsCommand TurnIndicatorDecider::decide(const PlannerContext & context)
{
  // Not guarded: SafetyPlanner::plan builds the reference_path from it before any planner runs
  const auto & route_manager = *context.route_manager;
  const auto & ego_pose = context.odometry.pose.pose;
  const double ego_velocity = context.odometry.twist.twist.linear.x;
  const rclcpp::Time stamp(context.odometry.header.stamp);

  if (
    !latched_goal_pose_ ||
    autoware_utils_geometry::calc_distance2d(*latched_goal_pose_, context.goal_pose) > 1e-3) {
    latched_goal_pose_ = context.goal_pose;
    pull_out_latch_ = DISABLE;
    arrived_at_goal_ = false;
  }

  const double ego_yaw = autoware_utils_geometry::get_rpy(ego_pose).z;
  const double s_ego = compute_ego_frenet_state(context).s;

  // The first lit maneuver wins, so a turn still being completed is not stolen by the next one
  uint8_t maneuver_signal = DISABLE;
  for (const auto & maneuver :
       find_maneuvers(context.reference_path, s_ego, *route_manager.lanelet_map_ptr(), params_)) {
    maneuver_signal = decide_maneuver_signal(
      maneuver.direction, maneuver.dist_to_start, ego_yaw, maneuver.exit_yaw, ego_velocity,
      params_);
    if (maneuver_signal != DISABLE) {
      break;
    }
  }

  // Not the lanelet under the goal: a shoulder goal must read as offset from the road lane
  const double dist_to_goal = autoware_utils_geometry::calc_distance2d(ego_pose, context.goal_pose);
  const auto goal_lanelet = route_manager.get_closest_preferred_route_lanelet(context.goal_pose);
  const double goal_offset = goal_lanelet ? lanelet2_utils::get_lateral_distance_to_centerline(
                                              *goal_lanelet, context.goal_pose)
                                          : 0.0;
  const auto pull_over =
    decide_pull_over(dist_to_goal, goal_offset, ego_velocity, params_, arrived_at_goal_);

  // Neighbours included so a lane change still reads as near-zero offset and cannot raise a signal.
  // Suppressed inside the pull-over range so the two cannot fight over the direction
  const auto lanes = lanelet2_utils::all_neighbor_lanelets(
    route_manager.current_lanelet(), route_manager.routing_graph_ptr());
  const auto pull_out = decide_pull_out(
    lanelet2_utils::get_lateral_distance_to_centerline(lanes, ego_pose), ego_velocity,
    dist_to_goal <= params_.search_distance, params_, pull_out_latch_);

  uint8_t desired = DISABLE;
  for (const uint8_t candidate : {maneuver_signal, pull_out, pull_over}) {
    if (candidate != DISABLE) {
      desired = candidate;
      break;
    }
  }

  // Switching directly between left and right is not held back
  const double now = stamp.seconds();
  const bool turning_off = desired == DISABLE && held_command_ != DISABLE;
  if (
    desired != held_command_ && !(turning_off && now - held_since_ < params_.min_blink_duration)) {
    held_command_ = desired;
    held_since_ = now;
  }

  TurnIndicatorsCommand cmd;
  cmd.stamp = stamp;
  cmd.command = held_command_;
  return cmd;
}

}  // namespace autoware::safety_planner
