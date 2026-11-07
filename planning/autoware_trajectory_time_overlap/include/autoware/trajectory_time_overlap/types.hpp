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

#ifndef AUTOWARE__TRAJECTORY_TIME_OVERLAP__TYPES_HPP_
#define AUTOWARE__TRAJECTORY_TIME_OVERLAP__TYPES_HPP_

#include <autoware/universe_utils/geometry/boost_geometry.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/index/predicates.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <iomanip>
#include <iterator>
#include <sstream>
#include <utility>
#include <vector>

namespace autoware::trajectory_time_overlap
{

/// @brief indicates the position of an intersection with the ego footprint
enum IntersectionPosition {
  front_left,            // intersect with the front_left linestring
  front_right,           // intersect with the front_right linestring
  rear_left,             // intersect with the rear_left linestring
  rear_right,            // intersect with the rear_right linestring
  rear,                  // intersect with the rear segment
  front,                 // intersect with the front segment
  inside_front_polygon,  // inside the front polygon (polygon from front left and right linestrings)
  inside_rear_polygon,   // inside the rear polygon (polygon from rear left and right linestrings)
  inside_both_polygons   // inside both the front and rear polygons
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

/// @brief rigid outline swept along a predicted path, one linestring per outline vertex
struct ObjectPathFootprint
{
  std::vector<universe_utils::LineString2d> outline_linestrings;
  double time_step{};

  /// @brief number of poses along the path
  [[nodiscard]] size_t size() const
  {
    return outline_linestrings.empty() ? 0UL : outline_linestrings.front().size();
  }
};

/// @brief footprint intersection with the corresponding time (for ego and the object), position,
/// and angle
struct FootprintIntersection
{
  double ego_time{};     // [s] time when ego is predicted to reach the intersection point
  double object_time{};  // [s] time when the object is predicted to reach the intersection point
  universe_utils::Point2d intersection;  // intersection point
  IntersectionPosition position{};       // position relative to the ego trajectory footprint
  double arc_length{};  // [m] arc length of the intersection along the ego trajectory
  double yaw_diff{};    // [rad] yaw difference between ego and the object at the intersection
  double ego_vel{};     // [m/s] ego velocity at the intersection
  double vel_diff{};    // [m/s] velocity difference between ego and the object at the intersection
};

namespace bgi = boost::geometry::index;
using PolygonNode = std::pair<universe_utils::Box2d, size_t>;
class PolygonRtree : bgi::rtree<PolygonNode, bgi::rstar<16>>
{
  static std::vector<PolygonNode> prepare_nodes(
    const std::vector<universe_utils::LinearRing2d> & polygons)
  {
    std::vector<PolygonNode> nodes;
    nodes.reserve(polygons.size());
    for (auto i = 0UL; i < polygons.size(); ++i) {
      nodes.emplace_back(boost::geometry::return_envelope<universe_utils::Box2d>(polygons[i]), i);
    }
    return nodes;
  }

public:
  PolygonRtree() = default;
  explicit PolygonRtree(const std::vector<universe_utils::LinearRing2d> & polygons)
  : bgi::rtree<PolygonNode, bgi::rstar<16>>(PolygonRtree::prepare_nodes(polygons))
  {
  }

  /// @brief check if the given geometry is disjoint from the polygons contained in the rtree
  template <class T>
  bool is_geometry_disjoint_from_rtree_polygons(
    const T & geometry, const std::vector<universe_utils::LinearRing2d> & polygons) const
  {
    std::vector<PolygonNode> query_results;
    query(!bgi::disjoint(geometry), std::back_inserter(query_results));
    for (const auto & query_result : query_results) {
      const auto & polygon = polygons[query_result.second];
      if (!boost::geometry::disjoint(geometry, polygon)) {
        return false;
      }
    }
    return true;
  }
};
using FootprintSegmentNode =
  std::pair<universe_utils::Segment2d, std::pair<IntersectionPosition, size_t>>;
using FootprintSegmentRtree = bgi::rtree<FootprintSegmentNode, bgi::rstar<16>>;

/// @brief the corner footprint of the ego trajectory
struct TrajectoryCornerFootprint
{
  CornerFootprint predicted_path_footprint;
  FootprintSegmentRtree segments_rtree;
  std::vector<universe_utils::LinearRing2d>
    front_polygons;  // polygons built from the front linestrings
  std::vector<universe_utils::LinearRing2d>
    rear_polygons;  // polygons built from the rear linestrings
  PolygonRtree front_polygons_rtree;
  PolygonRtree rear_polygons_rtree;
  double max_longitudinal_offset{};  // [m] distance between baselink and the front of the vehicle
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> ego_trajectory;

  /// @brief get the rear footprint segment of the given index
  [[nodiscard]] universe_utils::Segment2d get_rear_segment(const size_t index) const
  {
    return {
      predicted_path_footprint.corner_linestrings[rear_left][index],
      predicted_path_footprint.corner_linestrings[rear_right][index]};
  }
  /// @brief get the front footprint segment of the given index
  [[nodiscard]] universe_utils::Segment2d get_front_segment(const size_t index) const
  {
    return {
      predicted_path_footprint.corner_linestrings[front_left][index],
      predicted_path_footprint.corner_linestrings[front_right][index]};
  }
};

/// @brief the time interval where a vehicle overlaps the path of another vehicle
struct TimeOverlapInterval
{
  double from{};
  double to{};
  FootprintIntersection first_intersection;
  FootprintIntersection last_intersection;
  TimeOverlapInterval(
    const double from_, const double to_, FootprintIntersection first_intersection_,
    FootprintIntersection last_intersection_)
  : from(from_),
    to(to_),
    first_intersection(std::move(first_intersection_)),
    last_intersection(std::move(last_intersection_))
  {
  }

  [[nodiscard]] bool precedes(const TimeOverlapInterval & o, const double tolerance = 0.0) const
  {
    return to + tolerance < o.from;
  }
  [[nodiscard]] bool succeeds(const TimeOverlapInterval & o, const double tolerance = 0.0) const
  {
    return from > o.to + tolerance;
  }
  [[nodiscard]] bool overlaps(const TimeOverlapInterval & o, const double tolerance = 0.0) const
  {
    return !precedes(o, tolerance) && !succeeds(o, tolerance) && !o.precedes(*this, tolerance) &&
           !o.succeeds(*this, tolerance);
  };

  void expand(const TimeOverlapInterval & o)
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

inline std::ostream & operator<<(std::ostream & os, const TimeOverlapInterval & i)
{
  std::stringstream ss;
  ss << std::setprecision(3) << "[" << i.from << ", " << i.to << "]";
  os << ss.str();
  return os;
}

/// @brief time intervals for ego and the object when their predicted paths overlap
struct TimeOverlapIntervalPair
{
  TimeOverlapInterval ego;
  TimeOverlapInterval object;
  /// @brief false when the object outline is still inside the ego footprint at the end of the path
  bool exited{true};

  TimeOverlapIntervalPair(TimeOverlapInterval e, TimeOverlapInterval o, const bool exited_ = true)
  : ego(std::move(e)), object(std::move(o)), exited(exited_)
  {
  }
};

}  // namespace autoware::trajectory_time_overlap

#endif  // AUTOWARE__TRAJECTORY_TIME_OVERLAP__TYPES_HPP_
