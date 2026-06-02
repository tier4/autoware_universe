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
#include <unordered_map>
#include <vector>

namespace autoware::minimum_rule_based_planner::turn_indicator
{

namespace
{
double point_distance(const PathPointLite & a, const PathPointLite & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}
}  // namespace

TurnDirection point_turn_direction(
  const PathPointLite & point, const std::unordered_map<int64_t, TurnDirection> & direction_of)
{
  for (const auto id : point.lane_ids) {
    const auto it = direction_of.find(id);
    if (it != direction_of.end() && it->second != TurnDirection::NONE) {
      return it->second;
    }
  }
  return TurnDirection::NONE;
}

std::size_t nearest_index(const std::vector<PathPointLite> & points, double x, double y)
{
  std::size_t best = 0;
  double best_dist = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double d = std::hypot(points[i].x - x, points[i].y - y);
    if (d < best_dist) {
      best_dist = d;
      best = i;
    }
  }
  return best;
}

std::optional<TurnSegment> find_next_turn_segment(
  const std::vector<PathPointLite> & points, std::size_t ego_index,
  const std::unordered_map<int64_t, TurnDirection> & direction_of)
{
  if (points.empty() || ego_index >= points.size()) {
    return std::nullopt;
  }

  double arc = 0.0;
  std::optional<std::size_t> start_idx;
  std::size_t end_idx = 0;
  TurnDirection seg_dir = TurnDirection::NONE;
  double start_arc = 0.0;
  double end_arc = 0.0;

  for (std::size_t i = ego_index; i < points.size(); ++i) {
    if (i > ego_index) {
      arc += point_distance(points[i - 1], points[i]);
    }
    const auto dir = point_turn_direction(points[i], direction_of);

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
      break;  // the turn segment has ended
    }
  }

  if (!start_idx) {
    return std::nullopt;
  }
  return TurnSegment{seg_dir, start_arc, end_arc, *start_idx, end_idx};
}

TurnDirection decide_intersection_signal(
  const std::optional<TurnSegment> & segment, double ego_velocity, const TurnSignalParams & params)
{
  if (!segment || segment->direction == TurnDirection::NONE) {
    return TurnDirection::NONE;
  }
  const double activation_distance =
    std::max(ego_velocity * params.search_time, params.intersection_search_distance);
  if (segment->dist_to_start <= activation_distance && segment->dist_to_end > 0.0) {
    return segment->direction;
  }
  return TurnDirection::NONE;
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

TurnDirection resolve_priority(
  TurnDirection intersection, TurnDirection pull_out, TurnDirection pull_over)
{
  if (intersection != TurnDirection::NONE) {
    return intersection;
  }
  if (pull_out != TurnDirection::NONE) {
    return pull_out;
  }
  return pull_over;
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
