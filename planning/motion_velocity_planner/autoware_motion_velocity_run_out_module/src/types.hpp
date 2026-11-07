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

#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <autoware/trajectory_time_overlap/types.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <rclcpp/time.hpp>

#include <autoware_internal_planning_msgs/msg/safety_factor_array.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/index/predicates.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{

using autoware::trajectory_time_overlap::CornerFootprint;
using autoware::trajectory_time_overlap::FootprintIntersection;
using autoware::trajectory_time_overlap::FootprintSegmentNode;
using autoware::trajectory_time_overlap::FootprintSegmentRtree;
using autoware::trajectory_time_overlap::front;
using autoware::trajectory_time_overlap::front_left;
using autoware::trajectory_time_overlap::front_right;
using autoware::trajectory_time_overlap::inside_both_polygons;
using autoware::trajectory_time_overlap::inside_front_polygon;
using autoware::trajectory_time_overlap::inside_rear_polygon;
using autoware::trajectory_time_overlap::IntersectionPosition;
using autoware::trajectory_time_overlap::ObjectPathFootprint;
using autoware::trajectory_time_overlap::PolygonRtree;
using autoware::trajectory_time_overlap::rear;
using autoware::trajectory_time_overlap::rear_left;
using autoware::trajectory_time_overlap::rear_right;
using autoware::trajectory_time_overlap::TimeOverlapInterval;
using autoware::trajectory_time_overlap::TimeOverlapIntervalPair;
using autoware::trajectory_time_overlap::TrajectoryCornerFootprint;
using ObjectPredictedPathFootprint = ObjectPathFootprint;

/// @brief a set of footprint intersections between ego and a specific object
struct FootprintIntersections
{
  std::vector<FootprintIntersection> intersections;
  std::string uuid;

  explicit FootprintIntersections(const std::string & id) { uuid = id; }
};

namespace bgi = boost::geometry::index;
using SegmentNode = std::pair<universe_utils::Segment2d, size_t>;
using SegmentRtree = bgi::rtree<SegmentNode, bgi::rstar<16>>;
/// @brief Type of collision between ego and an object
enum CollisionType { pass_first_no_collision, ignored_collision, collision, no_collision };
/// @brief Collision between ego and an object with the corresponding time intervals and type
struct Collision
{
  TimeOverlapInterval ego_time_interval;
  TimeOverlapInterval object_time_interval;
  CollisionType type = no_collision;
  double ego_collision_time{};  // [s] predicted time of the collision for ego (only used when type
                                // is 'collision')
  std::string explanation;

  Collision(TimeOverlapInterval ego, TimeOverlapInterval object)
  : ego_time_interval(std::move(ego)), object_time_interval(std::move(object))
  {
  }
};
/// @brief Decision type
enum DecisionType { stop, slowdown, nothing };
/// @brief Decision with the corresponding collision, type, and stop point
struct Decision
{
  std::optional<Collision> collision = std::nullopt;
  DecisionType type = nothing;
  std::optional<geometry_msgs::msg::Point> stop_point;  // stop point calculated from this decision
  std::optional<SlowdownInterval> slowdown_interval;    // slowdown calculated from this decision
  std::string explanation;

  Decision() = default;
  Decision(Collision collision_, const DecisionType type_)
  : collision(std::move(collision_)), type(type_)
  {
  }
};
/// @brief History of decision and the corresponding decision times
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
};
/// @brief Tracker for the decisions toward predicted objects
struct ObjectDecisionsTracker
{
  std::unordered_map<std::string, DecisionHistory> history_per_object;
  /// @brief get the decision history of the given object
  std::optional<DecisionHistory> get(const std::string & object) const
  {
    if (history_per_object.count(object) < 1) {
      return std::nullopt;
    }
    return history_per_object.at(object);
  }
};
/// @brief Object represented by its uuid, corner footprints, current footprint, position,
/// collisions with ego, classification label
struct Object
{
  std::shared_ptr<motion_velocity_planner::PlannerData::Object> object;
  std::string uuid;
  std::vector<ObjectPredictedPathFootprint>
    predicted_path_footprints;  // footprint of each predicted path
  universe_utils::Polygon2d current_footprint;
  universe_utils::Point2d position;
  bool is_stopped = false;
  uint8_t label;
  bool has_target_label = false;
  std::vector<Collision> collisions;  // collisions with the ego trajectory
  bool ignore_but_preserve_predicted_paths = false;
};
/// @brief data to filter predicted paths and collisions
struct FilteringData
{
  std::vector<universe_utils::LinearRing2d> ignore_objects_polygons;
  PolygonRtree ignore_objects_rtree;
  std::vector<universe_utils::LinearRing2d> ignore_collisions_polygons;
  PolygonRtree ignore_collisions_rtree;
  std::vector<universe_utils::Segment2d> cut_predicted_paths_segments;
  SegmentRtree cut_predicted_paths_rtree;
  std::vector<universe_utils::Segment2d> strict_cut_predicted_paths_segments;
  SegmentRtree strict_cut_predicted_paths_rtree;
};
using FilteringDataPerLabel = std::vector<FilteringData>;

/// @brief run out results including the factors for the stops and slowdowns
struct RunOutResult
{
  VelocityPlanningResult velocity_planning_result;
  std::vector<std::string> stop_objects;      // target object uuid for each stop result
  std::vector<std::string> slowdown_objects;  // target object uuid for each slowdown result
};

}  // namespace autoware::motion_velocity_planner::run_out

#endif  // TYPES_HPP_
