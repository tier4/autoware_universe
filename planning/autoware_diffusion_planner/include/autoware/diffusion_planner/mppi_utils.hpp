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

#ifndef AUTOWARE__DIFFUSION_PLANNER__MPPI_UTILS__HPP_
#define AUTOWARE__DIFFUSION_PLANNER__MPPI_UTILS__HPP_

#include <autoware/avoidance_target_detector/boundary.hpp>
#include <autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/geometry/algorithms/for_each.hpp>
#include <boost/geometry/index/predicates.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <lanelet2_core/primitives/LineString.h>

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{

using RoadBorderSegment = autoware_utils_geometry::Segment2d;
using RoadBorderRtree =
  boost::geometry::index::rtree<RoadBorderSegment, boost::geometry::index::rstar<16>>;
using DrivableAreaSegment = autoware_utils_geometry::Segment2d;
using DrivableAreaRtree =
  boost::geometry::index::rtree<DrivableAreaSegment, boost::geometry::index::rstar<16>>;

namespace detail
{

inline visualization_msgs::msg::Marker create_mppi_line_list_marker(
  const std::string & marker_namespace,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = marker_namespace;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.15;
  marker.color = color;
  return marker;
}

inline std_msgs::msg::ColorRGBA create_marker_color(
  const float red, const float green, const float blue)
{
  std_msgs::msg::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = 1.0F;
  return color;
}

template <class Segment>
void append_segment(
  visualization_msgs::msg::Marker & marker, const Segment & segment, const double z)
{
  geometry_msgs::msg::Point first;
  first.x = boost::geometry::get<0, 0>(segment);
  first.y = boost::geometry::get<0, 1>(segment);
  first.z = z;

  geometry_msgs::msg::Point second;
  second.x = boost::geometry::get<1, 0>(segment);
  second.y = boost::geometry::get<1, 1>(segment);
  second.z = z;

  marker.points.push_back(first);
  marker.points.push_back(second);
}

template <class Geometry>
void append_geometry_segments(
  visualization_msgs::msg::Marker & marker, const Geometry & geometry, const double z)
{
  boost::geometry::for_each_segment(
    geometry, [&](const auto & segment) { append_segment(marker, segment, z); });
}

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
 * @brief Build a spatial index containing every segment of the supplied road borders.
 */
inline RoadBorderRtree prepare_road_border_rtree(
  const std::vector<lanelet::LineString2d> & road_borders)
{
  std::vector<RoadBorderSegment> segments;
  for (const auto & road_border : road_borders) {
    detail::append_lanelet_linestring_segments(segments, road_border);
  }
  return {segments.begin(), segments.end()};
}

/**
 * @brief Build a spatial index containing every segment of both drivable-area bounds.
 */
inline DrivableAreaRtree prepare_drivable_area_rtree(
  const autoware::avoidance_target_detector::RouteBounds & drivable_area)
{
  std::vector<DrivableAreaSegment> segments;
  detail::append_lanelet_linestring_segments(segments, drivable_area.first);
  detail::append_lanelet_linestring_segments(segments, drivable_area.second);
  return {segments.begin(), segments.end()};
}

/**
 * @brief Retrieve road-border segments intersecting the trajectory bounding box plus a margin.
 */
inline std::vector<RoadBorderSegment> get_road_border_subset(
  const RoadBorderRtree & road_border_rtree,
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

  std::vector<RoadBorderSegment> road_border_subset;
  road_border_rtree.query(
    boost::geometry::index::intersects(query_box), std::back_inserter(road_border_subset));
  return road_border_subset;
}

/**
 * @brief Retrieve drivable-area segments intersecting the trajectory bounding box plus a margin.
 */
inline std::vector<DrivableAreaSegment> get_drivable_area_subset(
  const DrivableAreaRtree & drivable_area_rtree,
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

  std::vector<DrivableAreaSegment> drivable_area_subset;
  drivable_area_rtree.query(
    boost::geometry::index::intersects(query_box), std::back_inserter(drivable_area_subset));
  return drivable_area_subset;
}

template <class Segment>
inline std::vector<autoware::mppi_optimizer::FirstOrderDubinsMppiSegment> to_mppi_segments(
  const std::vector<Segment> & segments)
{
  std::vector<autoware::mppi_optimizer::FirstOrderDubinsMppiSegment> result;
  result.reserve(segments.size());
  for (const auto & segment : segments) {
    result.push_back({
      static_cast<float>(boost::geometry::get<0, 0>(segment)),
      static_cast<float>(boost::geometry::get<0, 1>(segment)),
      static_cast<float>(boost::geometry::get<1, 0>(segment)),
      static_cast<float>(boost::geometry::get<1, 1>(segment))});
  }
  return result;
}

/**
 * @brief Create LINE_LIST debug markers for the inputs supplied to the MPPI optimizer.
 */
inline visualization_msgs::msg::MarkerArray generate_mppi_debug_markers(
  const std::vector<RoadBorderSegment> & road_borders,
  const std::vector<DrivableAreaSegment> & drivable_area,
  const autoware_perception_msgs::msg::TrackedObjects & avoidance_targets,
  const autoware_perception_msgs::msg::TrackedObjects & driving_along_targets)
{
  constexpr double marker_z = 100.0;

  auto road_borders_marker = detail::create_mppi_line_list_marker(
    "mppi_road_borders",
    detail::create_marker_color(1.0F, 0.0F, 0.0F));
  for (const auto & road_border : road_borders) {
    detail::append_segment(road_borders_marker, road_border, marker_z);
  }

  auto drivable_area_marker = detail::create_mppi_line_list_marker(
    "mppi_drivable_area",
    detail::create_marker_color(0.0F, 1.0F, 0.0F));
  for (const auto & drivable_area_segment : drivable_area) {
    detail::append_segment(drivable_area_marker, drivable_area_segment, marker_z);
  }

  auto avoidance_targets_marker = detail::create_mppi_line_list_marker(
    "mppi_avoidance_targets",
    detail::create_marker_color(1.0F, 0.5F, 0.0F));
  for (const auto & object : avoidance_targets.objects) {
    const auto footprint = autoware_utils_geometry::to_polygon2d(object);
    detail::append_geometry_segments(
      avoidance_targets_marker, footprint,
      object.kinematics.pose_with_covariance.pose.position.z + marker_z);
  }

  auto driving_along_targets_marker = detail::create_mppi_line_list_marker(
    "mppi_driving_along_targets",
    detail::create_marker_color(0.0F, 0.5F, 1.0F));
  for (const auto & object : driving_along_targets.objects) {
    const auto footprint = autoware_utils_geometry::to_polygon2d(object);
    detail::append_geometry_segments(
      driving_along_targets_marker, footprint,
      object.kinematics.pose_with_covariance.pose.position.z + marker_z);
  }

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.reserve(4);
  marker_array.markers.push_back(std::move(road_borders_marker));
  marker_array.markers.push_back(std::move(drivable_area_marker));
  marker_array.markers.push_back(std::move(avoidance_targets_marker));
  marker_array.markers.push_back(std::move(driving_along_targets_marker));
  return marker_array;
}

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__MPPI_UTILS__HPP_
