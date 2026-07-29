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

#include "turn_indicator_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace autoware::minimum_rule_based_planner::turn_indicator
{

namespace
{
//! Arc length below which ego counts as having entered a segment (it may be slightly negative
//! once ego is inside, because the segment start is then behind ego).
constexpr double kEnteredTolerance = 1e-3;

double point_distance(const PathPointLite & a, const PathPointLite & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

//! First index at most `distance` metres behind `ego_index`.
std::size_t scan_back_index(
  const std::vector<PathPointLite> & points, std::size_t ego_index, double distance)
{
  std::size_t index = ego_index;
  double travelled = 0.0;
  while (index > 0) {
    const double step = point_distance(points[index - 1], points[index]);
    if (travelled + step > distance) {
      break;
    }
    travelled += step;
    --index;
  }
  return index;
}

//! Walk `lookahead` metres forward from `index`, writing the reached index and its arc length.
void extend_forward(
  const std::vector<PathPointLite> & points, double lookahead, std::size_t & index, double & arc)
{
  double travelled = 0.0;
  for (std::size_t i = index + 1; i < points.size() && travelled < lookahead; ++i) {
    const double step = point_distance(points[i - 1], points[i]);
    travelled += step;
    arc += step;
    index = i;
  }
}

}  // namespace

double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle <= -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double path_yaw_at(const std::vector<PathPointLite> & points, std::size_t index)
{
  if (points.size() < 2 || index >= points.size()) {
    return 0.0;  // heading is undefined for a degenerate path
  }
  const std::size_t i0 = index + 1 < points.size() ? index : index - 1;
  const std::size_t i1 = i0 + 1;
  return std::atan2(points[i1].y - points[i0].y, points[i1].x - points[i0].x);
}

TurnDirection point_turn_direction(const PathPointLite & point, const LaneAttributeMap & attrs)
{
  for (const auto id : point.lane_ids) {
    const auto it = attrs.find(id);
    if (it != attrs.end() && it->second.turn_direction != TurnDirection::NONE) {
      return it->second.turn_direction;
    }
  }
  return TurnDirection::NONE;
}

bool point_is_private(const PathPointLite & point, const LaneAttributeMap & attrs)
{
  for (const auto id : point.lane_ids) {
    const auto it = attrs.find(id);
    if (it != attrs.end() && it->second.is_private) {
      return true;
    }
  }
  return false;
}

std::size_t nearest_index(const std::vector<PathPointLite> & points, double x, double y, double yaw)
{
  std::size_t best_aligned = 0;
  double best_aligned_dist = std::numeric_limits<double>::max();
  std::size_t best_any = 0;
  double best_any_dist = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < points.size(); ++i) {
    const double d = std::hypot(points[i].x - x, points[i].y - y);
    if (d < best_any_dist) {
      best_any_dist = d;
      best_any = i;
    }
    // Reject points the path traverses in (roughly) the opposite direction to ego.
    if (std::abs(normalize_angle(path_yaw_at(points, i) - yaw)) > M_PI_2) {
      continue;
    }
    if (d < best_aligned_dist) {
      best_aligned_dist = d;
      best_aligned = i;
    }
  }

  return best_aligned_dist == std::numeric_limits<double>::max() ? best_any : best_aligned;
}

std::vector<ManeuverSegment> find_turn_segments(
  const std::vector<PathPointLite> & points, std::size_t ego_index, const LaneAttributeMap & attrs,
  const TurnSignalParams & params)
{
  std::vector<ManeuverSegment> segments;
  if (points.empty() || ego_index >= points.size()) {
    return segments;
  }

  // Include a short window behind ego so that a turn ego is still completing (its lanelet already
  // behind) keeps its signal until the heading check clears it.
  const std::size_t begin = scan_back_index(points, ego_index, params.exit_lookahead);
  double arc = -0.0;
  {
    // Arc lengths are measured from ego, so points behind ego get a negative arc.
    double behind = 0.0;
    for (std::size_t i = begin; i < ego_index; ++i) {
      behind += point_distance(points[i], points[i + 1]);
    }
    arc = -behind;
  }

  std::optional<std::size_t> start_idx;
  std::size_t end_idx = 0;
  TurnDirection seg_dir = TurnDirection::NONE;
  double start_arc = 0.0;
  double end_arc = 0.0;

  const auto flush = [&]() {
    if (!start_idx) {
      return;
    }
    std::size_t index = end_idx;
    double exit_arc = end_arc;
    // Carry the segment `exit_lookahead` past the turn lanelet so the maneuver can be declared
    // finished by the heading check rather than by the lanelet's extent.
    extend_forward(points, params.exit_lookahead, index, exit_arc);
    segments.push_back(
      ManeuverSegment{
        seg_dir, ManeuverKind::INTERSECTION, start_arc, exit_arc, *start_idx, index,
        path_yaw_at(points, index)});
    start_idx.reset();
    seg_dir = TurnDirection::NONE;
  };

  for (std::size_t i = begin; i < points.size(); ++i) {
    if (i > begin) {
      arc += point_distance(points[i - 1], points[i]);
    }
    const auto dir = point_turn_direction(points[i], attrs);

    if (!start_idx) {
      if (dir != TurnDirection::NONE) {
        start_idx = i;
        end_idx = i;
        seg_dir = dir;
        start_arc = arc;
        end_arc = arc;
      }
    } else if (dir == seg_dir) {
      end_idx = i;
      end_arc = arc;
    } else {
      flush();
      // The very same point may open the next segment (e.g. right lanelet -> left lanelet).
      if (dir != TurnDirection::NONE) {
        start_idx = i;
        end_idx = i;
        seg_dir = dir;
        start_arc = arc;
        end_arc = arc;
      }
    }
  }
  flush();

  return segments;
}

std::vector<ManeuverSegment> find_private_exit_segments(
  const std::vector<PathPointLite> & points, std::size_t ego_index, const LaneAttributeMap & attrs,
  const TurnSignalParams & params)
{
  std::vector<ManeuverSegment> segments;
  if (points.empty() || ego_index >= points.size()) {
    return segments;
  }

  const std::size_t begin = scan_back_index(points, ego_index, params.exit_lookahead);
  double arc = 0.0;
  {
    double behind = 0.0;
    for (std::size_t i = begin; i < ego_index; ++i) {
      behind += point_distance(points[i], points[i + 1]);
    }
    arc = -behind;
  }

  bool in_private_run = false;
  std::size_t run_start = 0;
  double start_arc = 0.0;

  for (std::size_t i = begin; i < points.size(); ++i) {
    if (i > begin) {
      arc += point_distance(points[i - 1], points[i]);
    }
    const bool is_private = point_is_private(points[i], attrs);

    if (!in_private_run) {
      if (is_private) {
        in_private_run = true;
        run_start = i;
        start_arc = arc;
      }
      continue;
    }
    if (is_private) {
      continue;
    }

    // `i` is the first public point: the private area is left here.
    std::size_t end_index = i;
    double end_arc = arc;
    extend_forward(points, params.exit_lookahead, end_index, end_arc);
    const double exit_yaw = path_yaw_at(points, end_index);

    // Prefer the exit lanelet's own tag; fall back to the geometric yaw change across the merge.
    TurnDirection dir = point_turn_direction(points[i - 1], attrs);
    if (dir == TurnDirection::NONE) {
      // Measure the approach heading a look-ahead BEFORE the boundary (but not before the private
      // run starts): right at the boundary the path is already turning, so the yaw change there
      // is near zero and every merge would read as straight.
      const std::size_t approach_index =
        std::max(run_start, scan_back_index(points, i - 1, params.exit_lookahead));
      const double delta = normalize_angle(exit_yaw - path_yaw_at(points, approach_index));
      if (std::abs(delta) > params.heading_align_threshold) {
        dir = delta > 0.0 ? TurnDirection::LEFT : TurnDirection::RIGHT;
      }
    }
    // A geometrically straight merge has no side to signal; skip it rather than guessing.
    if (dir != TurnDirection::NONE) {
      segments.push_back(
        ManeuverSegment{
          dir, ManeuverKind::PRIVATE_EXIT, start_arc, end_arc, run_start, end_index, exit_yaw});
    }
    in_private_run = false;
  }

  return segments;
}

TurnDirection decide_maneuver_signal(
  const ManeuverSegment & segment, double ego_velocity, double ego_yaw,
  const TurnSignalParams & params)
{
  if (segment.direction == TurnDirection::NONE) {
    return TurnDirection::NONE;
  }
  // Fully behind ego.
  if (segment.dist_to_end <= 0.0) {
    return TurnDirection::NONE;
  }
  // Spec end condition: once the maneuver has started, it is over when ego's heading matches the
  // heading it exits on (i.e. ego is aligned with the centerline again).
  const bool entered = segment.dist_to_start <= kEnteredTolerance;
  if (
    entered &&
    std::abs(normalize_angle(ego_yaw - segment.exit_yaw)) <= params.heading_align_threshold) {
    return TurnDirection::NONE;
  }
  const double activation_distance =
    std::max(ego_velocity * params.search_time, params.intersection_search_distance);
  if (segment.dist_to_start <= activation_distance) {
    return segment.direction;
  }
  return TurnDirection::NONE;
}

ActiveManeuver decide_maneuver_signal(
  const std::vector<ManeuverSegment> & segments, double ego_velocity, double ego_yaw,
  const TurnSignalParams & params)
{
  for (const auto & segment : segments) {
    const auto dir = decide_maneuver_signal(segment, ego_velocity, ego_yaw, params);
    if (dir != TurnDirection::NONE) {
      return ActiveManeuver{dir, segment};
    }
  }
  return ActiveManeuver{};
}

TurnDirection direction_from_lateral_offset(double signed_offset, double deadzone)
{
  if (signed_offset > deadzone) {
    return TurnDirection::LEFT;
  }
  if (signed_offset < -deadzone) {
    return TurnDirection::RIGHT;
  }
  return TurnDirection::NONE;
}

SignalDecision resolve_priority(
  TurnDirection intersection, TurnDirection private_exit, TurnDirection pull_out,
  TurnDirection pull_over)
{
  if (intersection != TurnDirection::NONE) {
    return {intersection, ManeuverKind::INTERSECTION};
  }
  if (private_exit != TurnDirection::NONE) {
    return {private_exit, ManeuverKind::PRIVATE_EXIT};
  }
  if (pull_out != TurnDirection::NONE) {
    return {pull_out, ManeuverKind::PULL_OUT};
  }
  if (pull_over != TurnDirection::NONE) {
    return {pull_over, ManeuverKind::PULL_OVER};
  }
  return {TurnDirection::NONE, ManeuverKind::NONE};
}

void DepartureLatch::reset()
{
  latched_ = false;
  direction_ = TurnDirection::NONE;
}

TurnDirection DepartureLatch::update(
  double signed_offset, double ego_velocity, bool suppressed, const TurnSignalParams & params)
{
  if (suppressed) {
    reset();
    return TurnDirection::NONE;
  }

  const double offset_magnitude = std::abs(signed_offset);

  // Back on the centerline: the departure is complete.
  if (offset_magnitude <= params.lateral_shift_threshold) {
    reset();
    return TurnDirection::NONE;
  }

  if (!latched_) {
    // Only a vehicle standing still, clear of the lane, is departing from a stop. A vehicle that
    // is offset while moving is being pushed off the centerline by the upstream planner (lane
    // change / avoidance), which is explicitly out of scope - stay dark.
    const bool stopped = std::abs(ego_velocity) <= params.stopped_velocity_threshold;
    const bool parked_off_lane = offset_magnitude > params.departure_lateral_threshold;
    if (!stopped || !parked_off_lane) {
      return TurnDirection::NONE;
    }
    latched_ = true;
    // Ego pulls out towards the centerline, i.e. against its own offset.
    direction_ = direction_from_lateral_offset(-signed_offset, params.lateral_shift_threshold);
  }

  return direction_;
}

void ArrivalState::reset()
{
  arrived_ = false;
}

TurnDirection ArrivalState::update(
  double dist_to_goal, double goal_offset, double ego_velocity, const TurnSignalParams & params)
{
  if (dist_to_goal > params.pull_over_search_distance) {
    arrived_ = false;  // re-arm for the next approach (e.g. a new route)
    return TurnDirection::NONE;
  }

  if (
    dist_to_goal <= params.goal_arrival_distance &&
    std::abs(ego_velocity) <= params.stopped_velocity_threshold) {
    arrived_ = true;
  }
  if (arrived_) {
    return TurnDirection::NONE;  // maneuver finished: turn the signal off
  }

  return direction_from_lateral_offset(goal_offset, params.lateral_shift_threshold);
}

TurnDirection BlinkHold::update(TurnDirection desired, double now)
{
  if (desired == held_) {
    return held_;
  }

  // Hold a lit signal off-transition until the minimum duration has elapsed.
  const bool turning_off = desired == TurnDirection::NONE;
  if (turning_off && held_ != TurnDirection::NONE && (now - held_since_) < min_duration_) {
    return held_;
  }

  held_ = desired;
  held_since_ = now;
  return held_;
}

}  // namespace autoware::minimum_rule_based_planner::turn_indicator
