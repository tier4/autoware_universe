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

#ifndef AUTOWARE__MPPI_OPTIMIZER__MPPI_DEBUG_MARKERS_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__MPPI_DEBUG_MARKERS_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"

#include <builtin_interfaces/msg/duration.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace autoware::mppi_optimizer
{

namespace detail
{

inline visualization_msgs::msg::Marker create_mppi_line_list_marker(
  const std::string & marker_namespace, const std_msgs::msg::ColorRGBA & color)
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

}  // namespace detail

using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

inline ColorRGBA makeColor(const float r, const float g, const float b, const float a)
{
  ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

inline ColorRGBA costGradientColor(const float cost, const float min_cost, const float max_cost)
{
  float t = 0.5F;
  if (max_cost > min_cost) {
    t = (cost - min_cost) / (max_cost - min_cost);
  }
  t = std::clamp(t, 0.0F, 1.0F);
  return makeColor(t, 1.0F - t, 0.0F, 0.35F);
}

inline Marker makeDeleteAllMarker(const std::string & ns)
{
  Marker marker;
  marker.header.frame_id = "map";
  marker.ns = ns;
  marker.id = 0;
  marker.action = Marker::DELETEALL;
  return marker;
}

inline Marker makeLineStripMarker(
  const std::string & ns, const int id, const ColorRGBA & color, const double line_width,
  const std::vector<std::pair<float, float>> & points, const double z)
{
  Marker marker;
  marker.header.frame_id = "map";
  marker.ns = ns;
  marker.id = id;
  marker.type = Marker::LINE_STRIP;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = line_width;
  marker.color = color;
  marker.lifetime = builtin_interfaces::msg::Duration{}.set__sec(0).set__nanosec(500000000);
  marker.points.reserve(points.size());
  for (const auto & [x, y] : points) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    marker.points.push_back(point);
  }
  return marker;
}

inline MarkerArray createMppiDebugMarkers(
  const FirstOrderDubinsMppiDebug & debug,
  const std::vector<autoware_utils_geometry::Segment2d> & road_borders,
  const std::vector<autoware_utils_geometry::Segment2d> & drivable_area,
  const autoware_perception_msgs::msg::TrackedObjects & avoidance_targets,
  const autoware_perception_msgs::msg::TrackedObjects & driving_along_targets, const double z)
{
  MarkerArray marker_array;

  if (!debug.reference_trajectory.points.empty()) {
    std::vector<std::pair<float, float>> reference_points;
    reference_points.reserve(debug.reference_trajectory.points.size());
    for (const auto & point : debug.reference_trajectory.points) {
      reference_points.emplace_back(
        static_cast<float>(point.pose.position.x), static_cast<float>(point.pose.position.y));
    }
    marker_array.markers.push_back(makeLineStripMarker(
      "mppi_reference", 0, makeColor(0.0F, 1.0F, 1.0F, 0.9F), 0.15, reference_points, z));
  }

  if (!debug.optimal_horizon.empty()) {
    marker_array.markers.push_back(makeLineStripMarker(
      "mppi_optimal", 0, makeColor(1.0F, 0.0F, 0.0F, 1.0F), 0.2, debug.optimal_horizon, z));
  }

  marker_array.markers.push_back(makeDeleteAllMarker("mppi_rollout"));

  if (!debug.rollouts.empty()) {
    float min_cost = debug.rollouts.front().cost;
    float max_cost = debug.rollouts.front().cost;
    for (const auto & rollout : debug.rollouts) {
      min_cost = std::min(min_cost, rollout.cost);
      max_cost = std::max(max_cost, rollout.cost);
    }

    int rollout_id = 1;
    for (const auto & rollout : debug.rollouts) {
      if (rollout.points.size() < 2U) {
        continue;
      }
      marker_array.markers.push_back(makeLineStripMarker(
        "mppi_rollout", rollout_id, costGradientColor(rollout.cost, min_cost, max_cost), 0.04,
        rollout.points, z));
      ++rollout_id;
    }
  }
  auto road_borders_marker = detail::create_mppi_line_list_marker(
    "mppi_road_borders", detail::create_marker_color(1.0F, 0.0F, 0.0F));
  for (const auto & road_border : road_borders) {
    detail::append_segment(road_borders_marker, road_border, z);
  }

  auto drivable_area_marker = detail::create_mppi_line_list_marker(
    "mppi_drivable_area", detail::create_marker_color(0.0F, 1.0F, 0.0F));
  for (const auto & drivable_area_segment : drivable_area) {
    detail::append_segment(drivable_area_marker, drivable_area_segment, z);
  }

  auto avoidance_targets_marker = detail::create_mppi_line_list_marker(
    "mppi_avoidance_targets", detail::create_marker_color(1.0F, 0.5F, 0.0F));
  for (const auto & object : avoidance_targets.objects) {
    const auto footprint = autoware_utils_geometry::to_polygon2d(object);
    detail::append_geometry_segments(
      avoidance_targets_marker, footprint,
      object.kinematics.pose_with_covariance.pose.position.z + z);
  }

  auto driving_along_targets_marker = detail::create_mppi_line_list_marker(
    "mppi_driving_along_targets", detail::create_marker_color(0.0F, 0.5F, 1.0F));
  for (const auto & object : driving_along_targets.objects) {
    const auto footprint = autoware_utils_geometry::to_polygon2d(object);
    detail::append_geometry_segments(
      driving_along_targets_marker, footprint,
      object.kinematics.pose_with_covariance.pose.position.z + z);
  }

  marker_array.markers.push_back(std::move(road_borders_marker));
  marker_array.markers.push_back(std::move(drivable_area_marker));
  marker_array.markers.push_back(std::move(avoidance_targets_marker));
  marker_array.markers.push_back(std::move(driving_along_targets_marker));
  return marker_array;
}

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__MPPI_DEBUG_MARKERS_HPP_
