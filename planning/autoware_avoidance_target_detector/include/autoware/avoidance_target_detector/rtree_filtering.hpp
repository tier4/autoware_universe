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
#ifndef AUTOWARE__AVOIDANCE_TARGET_DETECTOR__RTREE_FILTERING_HPP_
#define AUTOWARE__AVOIDANCE_TARGET_DETECTOR__RTREE_FILTERING_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <boost/geometry/algorithms/for_each.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <lanelet2_core/primitives/LineString.h>

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::avoidance_target_detector
{
/** Left and right route boundary linestrings. */
using RouteBounds = std::pair<lanelet::LineString2d, lanelet::LineString2d>;
using Segment = autoware_utils_geometry::Segment2d;
using SegmentRtree = boost::geometry::index::rtree<Segment, boost::geometry::index::rstar<16>>;

namespace detail
{
template <class Segment>
void append_lanelet_linestring_segments(
  std::vector<Segment> & segments, const lanelet::LineString2d & line_string)
{
  if (line_string.size() < 2) {
    return;
  }

  segments.reserve(segments.size() + line_string.size() - 1);
  for (std::size_t i = 0; i + 1 < line_string.size(); ++i) {
    segments.emplace_back(
      autoware_utils_geometry::Point2d(line_string[i].x(), line_string[i].y()),
      autoware_utils_geometry::Point2d(line_string[i + 1].x(), line_string[i + 1].y()));
  }
}
}  // namespace detail

/**
 * @brief
 */
inline SegmentRtree prepare_trajectory_rtree(
  const autoware_planning_msgs::msg::Trajectory & trajectory)
{
  std::vector<Segment> segments;
  segments.reserve(trajectory.points.size());
  for (auto i = 0UL; i + 1 < trajectory.points.size(); ++i) {
    const auto & p1 = trajectory.points[i].pose.position;
    const auto & p2 = trajectory.points[i + 1].pose.position;
    segments.emplace_back(
      autoware_utils_geometry::Point2d(p1.x, p1.y), autoware_utils_geometry::Point2d(p2.x, p2.y));
  }
  return {segments.begin(), segments.end()};
}

/**
 * @brief Build a spatial index containing every segment of the supplied road borders.
 */
inline SegmentRtree prepare_road_border_rtree(
  const std::vector<lanelet::LineString2d> & road_borders)
{
  std::vector<Segment> segments;
  for (const auto & road_border : road_borders) {
    detail::append_lanelet_linestring_segments(segments, road_border);
  }
  return {segments.begin(), segments.end()};
}

/**
 * @brief Build a spatial index containing every segment of both drivable-area bounds.
 */
inline SegmentRtree prepare_drivable_area_rtree(const RouteBounds & drivable_area)
{
  std::vector<Segment> segments;
  detail::append_lanelet_linestring_segments(segments, drivable_area.first);
  detail::append_lanelet_linestring_segments(segments, drivable_area.second);
  return {segments.begin(), segments.end()};
}

/**
 * @brief Retrieve road-border segments intersecting the trajectory bounding box plus a margin.
 */
inline std::vector<Segment> get_road_border_subset(
  const SegmentRtree & road_border_rtree,
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double margin)
{
  if (trajectory.points.empty() || road_border_rtree.empty()) {
    return {};
  }

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto & trajectory_point : trajectory.points) {
    const auto & position = trajectory_point.pose.position;
    min_x = std::min(min_x, position.x);
    min_y = std::min(min_y, position.y);
    max_x = std::max(max_x, position.x);
    max_y = std::max(max_y, position.y);
  }

  const double nonnegative_margin = std::max(0.0, margin);
  const lanelet::BoundingBox2d query_box(
    lanelet::BasicPoint2d(min_x - nonnegative_margin, min_y - nonnegative_margin),
    lanelet::BasicPoint2d(max_x + nonnegative_margin, max_y + nonnegative_margin));

  std::vector<Segment> road_border_subset;
  road_border_rtree.query(
    boost::geometry::index::intersects(query_box), std::back_inserter(road_border_subset));
  return road_border_subset;
}

/**
 * @brief Retrieve drivable-area segments intersecting the trajectory bounding box plus a margin.
 */
inline std::vector<Segment> get_drivable_area_subset(
  const SegmentRtree & drivable_area_rtree,
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double margin)
{
  if (trajectory.points.empty() || drivable_area_rtree.empty()) {
    return {};
  }

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto & trajectory_point : trajectory.points) {
    const auto & position = trajectory_point.pose.position;
    min_x = std::min(min_x, position.x);
    min_y = std::min(min_y, position.y);
    max_x = std::max(max_x, position.x);
    max_y = std::max(max_y, position.y);
  }

  const double nonnegative_margin = std::max(0.0, margin);
  const lanelet::BoundingBox2d query_box(
    lanelet::BasicPoint2d(min_x - nonnegative_margin, min_y - nonnegative_margin),
    lanelet::BasicPoint2d(max_x + nonnegative_margin, max_y + nonnegative_margin));

  std::vector<Segment> drivable_area_subset;
  drivable_area_rtree.query(
    boost::geometry::index::intersects(query_box), std::back_inserter(drivable_area_subset));
  return drivable_area_subset;
}

}  // namespace autoware::avoidance_target_detector
#endif  // AUTOWARE__AVOIDANCE_TARGET_DETECTOR__RTREE_FILTERING_HPP_
