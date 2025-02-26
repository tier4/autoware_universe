// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include "parameters.hpp"

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <rclcpp/time.hpp>

#include <boost/geometry/index/rtree.hpp>

#include <deque>
#include <iomanip>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{

/// @brief indicates the position of an intersection with the ego footprint
enum IntersectionPosition {
  front_left,
  front_right,
  rear_left,
  rear_right,
  rear,
  front,
  inside_front_polygon,
  inside_rear_polygon,
  inside_both_polygons
};
/// @brief footprint represented by linestrings corresponding to the path of 4 footprint corners
struct CornerFootprint
{
  std::vector<universe_utils::LineString2d> corner_linestrings{4};

  [[nodiscard]] size_t size() const
  {
    return corner_linestrings[IntersectionPosition::front_left].size();
  }
};
struct ObjectCornerFootprint
{
  CornerFootprint corner_footprint;
  double time_step{};
};

struct FootprintIntersection
{
  double ego_time{};
  double object_time{};
  universe_utils::Point2d intersection;
  IntersectionPosition position;
};

struct FootprintIntersections
{
  std::vector<FootprintIntersection> intersections;
  std::string uuid;

  explicit FootprintIntersections(const std::string & id) { uuid = id; }
};

namespace bgi = boost::geometry::index;
using SegmentNode = std::pair<universe_utils::Segment2d, size_t>;
using SegmentRtree = bgi::rtree<SegmentNode, bgi::rstar<16>>;
using FootprintSegmentNode =
  std::pair<universe_utils::Segment2d, std::pair<IntersectionPosition, size_t>>;
using FootprintSegmentRtree = bgi::rtree<FootprintSegmentNode, bgi::rstar<16>>;

struct TrajectoryCornerFootprint
{
  CornerFootprint corner_footprint;
  universe_utils::Polygon2d front_polygon;  // polygon built from the front linestrings
  universe_utils::Polygon2d rear_polygon;   // polygon built from the rear linestrings
  FootprintSegmentRtree rtree;

  /// @brief get the segment from rear left corner to rear right corner
  [[nodiscard]] universe_utils::Segment2d get_rear_segment() const
  {
    return {
      corner_footprint.corner_linestrings[rear_left].front(),
      corner_footprint.corner_linestrings[rear_right].front()};
  }
  /// @brief get the segment from front left corner to front right corner
  [[nodiscard]] universe_utils::Segment2d get_front_segment() const
  {
    return {
      corner_footprint.corner_linestrings[front_left].front(),
      corner_footprint.corner_linestrings[front_right].front()};
  }
};

/// @brief represent the time interval where a vehicle overlaps the path of another vehicle
struct TimeCollisionInterval
{
  double from;
  double to;
  FootprintIntersection first_intersection;
  FootprintIntersection last_intersection;
  TimeCollisionInterval() = delete;
  TimeCollisionInterval(
    const double from_, const double to_, FootprintIntersection first_intersection_,
    FootprintIntersection last_intersection_)
  : from(from_),
    to(to_),
    first_intersection(std::move(first_intersection_)),
    last_intersection(std::move(last_intersection_))
  {
  }

  [[nodiscard]] bool precedes(const TimeCollisionInterval & o) const { return to < o.from; }
  [[nodiscard]] bool succeeds(const TimeCollisionInterval & o) const { return from > o.to; }
  [[nodiscard]] bool overlaps(const TimeCollisionInterval & o) const
  {
    // TODO(Maxime): double check
    return !precedes(o) && !succeeds(o) && !o.precedes(*this) && !o.succeeds(*this);
  };

  void expand(const TimeCollisionInterval & o)
  {
    if (o.from < from) {
      from = o.from;
      first_intersection = o.first_intersection;
    }
    if (o.to > to) {
      to = o.to;
      last_intersection = o.last_intersection;
    }
  }
};

inline std::ostream & operator<<(std::ostream & os, const TimeCollisionInterval & i)
{
  std::stringstream ss;
  ss << std::setprecision(3) << "[" << i.from << ", " << i.to << "]";
  os << ss.str();
  return os;
}

enum CollisionType { pass_first_no_collision, pass_first_collision, collision, no_collision };

struct Collision
{
  double enter_time_margin{};
  double exit_time_margin{};
  TimeCollisionInterval ego_time_interval;
  TimeCollisionInterval object_time_interval;
  CollisionType type;
  std::string explanation;

  Collision(
    const TimeCollisionInterval & ego, const TimeCollisionInterval & object,
    const Parameters & params)
  : ego_time_interval(ego), object_time_interval(object)
  {
    // TODO(Maxime): move to collision.cpp
    if (
      params.enable_passing_collisions && ego.from < object.from && ego.overlaps(object) &&
      ego.from + params.passing_collisions_time_margin < object.from &&
      ego.to - ego.from <= params.passing_max_overlap_duration) {
      type = pass_first_collision;
      enter_time_margin = object.from - ego.from;
      std::stringstream ss;
      ss << std::setprecision(2) << "pass first collision since ego arrives first (" << ego.from
         << " < " << object.from << "), including with margin ("
         << params.passing_collisions_time_margin << ") and ego overlap bellow max ("
         << ego.to - ego.from << " < " << params.passing_max_overlap_duration << ")";
      explanation = ss.str();
    } else if (ego.overlaps(object)) {
      type = collision;
      enter_time_margin = object.to - ego.from;
    } else if (ego.to < object.from) {
      type = pass_first_no_collision;
      enter_time_margin = ego.to - object.from;
    } else {
      type = no_collision;
    }
  }
};

enum DecisionType { stop, slowdown, nothing };
struct Decision
{
  std::optional<Collision> collision = std::nullopt;
  DecisionType type = nothing;
  std::optional<geometry_msgs::msg::Point> stop_point;
  std::string explanation;

  Decision() = default;
  Decision(Collision collision_, const DecisionType type_)
  : collision(std::move(collision_)), type(type_)
  {
  }
};

struct DecisionHistory
{
  std::deque<Decision> decisions;
  std::deque<double> times;

  /// @brief remove outdated history, keeping at most one item above the max_history_duration
  void remove_outdated(const rclcpp::Time & now, const double max_history_duration)
  {
    while (times.size() > 1UL && (now.seconds() - times[1]) >= max_history_duration) {
      times.pop_front();
      decisions.pop_front();
    }
  }

  void add_decision(const double now, const Decision & decision)
  {
    times.push_back(now);
    decisions.push_back(decision);
  }

  [[nodiscard]] bool has_data_before_time(const double t) const
  {
    return !times.empty() && times.front() <= t;
  }
  [[nodiscard]] std::optional<int> first_index_at_or_after_time(const double t) const
  {
    for (auto i = 0UL; i < times.size(); ++i) {
      if (times[i] > t) {
        return i;
      }
    }
    return std::nullopt;
  }
};
struct ObjectDecisionsTracker
{
  std::unordered_map<std::string, DecisionHistory> history_per_object;

  void remove_outdated(const rclcpp::Time & now, const double outdated_duration)
  {
    for (auto & [_, history] : history_per_object) {
      history.remove_outdated(now, outdated_duration);
    }
  }

  void update_objects_without_decisions(const rclcpp::Time & now)
  {
    for (auto & [_, history] : history_per_object) {
      if (!history.times.empty() && history.times.back() != now.seconds()) {
        history.times.push_back(now.seconds());
        history.decisions.emplace_back();
      }
    }
    // remove histories where all decisions are "nothing"
    constexpr auto nothing_no_collision = [](const Decision & decision) {
      return decision.type == nothing &&
             (!decision.collision.has_value() || decision.collision->type == no_collision);
    };
    for (auto it = history_per_object.begin(); it != history_per_object.end();) {
      const auto & history = it->second;
      if (std::all_of(history.decisions.begin(), history.decisions.end(), nothing_no_collision)) {
        it = history_per_object.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::optional<DecisionHistory> get(const std::string & object) const
  {
    if (history_per_object.count(object) < 1) {
      return std::nullopt;
    }
    return history_per_object.at(object);
  }
};

struct UnavoidableCollision
{
  double time_to_collision;
  double comfortable_time_to_stop;
};

struct Object
{
  std::string uuid;
  std::vector<ObjectCornerFootprint> corner_footprints;  // footprint of each predicted path
  universe_utils::Polygon2d current_footprint;
  universe_utils::Point2d position;
  bool is_pedestrian = false;
  bool is_road_object = false;
  bool is_on_ego_trajectory = false;
  bool has_parked_label = false;
  bool has_target_label = false;
  std::vector<Collision> collisions;  // collisions with the ego trajectory
};

/// @brief data to filter predicted paths and collisions
struct FilteringData
{
  std::vector<universe_utils::LinearRing2d> ignore_pedestrian_polygons;
  std::vector<universe_utils::LinearRing2d> ignore_road_object_polygons;
  std::vector<universe_utils::Segment2d> cut_predicted_paths_segments;
  SegmentRtree cut_predicted_paths_rtree;
};

}  // namespace autoware::motion_velocity_planner::run_out

#endif  // TYPES_HPP_
