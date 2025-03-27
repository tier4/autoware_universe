// Copyright 2025 TIER IV, Inc.
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

#include "debug.hpp"

#include <string>
#include <vector>

namespace color
{
using std_msgs::msg::ColorRGBA;

inline ColorRGBA blue(float a = 0.99)
{
  return autoware_utils::create_marker_color(0., 0., 1., a);
}

inline ColorRGBA yellow(float a = 0.99)
{
  return autoware_utils::create_marker_color(1., 1., 0., a);
}

inline ColorRGBA green(float a = 0.99)
{
  return autoware_utils::create_marker_color(0., 1., 0., a);
}
}  // namespace color

namespace autoware::motion_velocity_planner::debug
{
Marker create_ego_sides_marker(
  const EgoSides & ego_sides_from_footprints, Marker marker, std::string && ns,
  const double base_link_z)
{
  marker.ns = ns;
  marker.points.reserve(ego_sides_from_footprints.size() * 4);
  const auto to_geom = [base_link_z](const auto & pt) { return to_msg(pt.to_3d(base_link_z)); };
  for (const auto & ego_footprint_sides : ego_sides_from_footprints) {
    const auto & left = ego_footprint_sides.left;
    marker.points.push_back(to_geom(left.first));
    marker.points.push_back(to_geom(left.second));
    const auto & right = ego_footprint_sides.right;
    marker.points.push_back(to_geom(right.first));
    marker.points.push_back(to_geom(right.second));
  }

  return marker;
}

Marker create_side_to_boundary_marker(
  const std::vector<ProjectionWithSegment> & side_to_boundary, Marker marker, std::string && ns,
  const double base_link_z)
{
  marker.ns = ns;
  const auto to_geom = [base_link_z](const auto & pt) { return to_msg(pt.to_3d(base_link_z)); };
  for (const auto & [projection, segment, dist_from_start] : side_to_boundary) {
    const auto & [orig, proj, dist] = projection;
    marker.color = color::blue();
    marker.points.push_back(to_geom(orig));
    marker.points.push_back(to_geom(proj));
    marker.points.push_back(to_geom(segment.first));
    marker.points.push_back(to_geom(segment.second));
  }
  return marker;
}

MarkerArray create_debug_marker_array(
  [[maybe_unused]] const lane_departure_checker::Output & output,
  const rclcpp::Clock::SharedPtr & clock_ptr, [[maybe_unused]] const double base_link_z)
{
  const auto line_list = visualization_msgs::msg::Marker::LINE_LIST;
  const auto curr_time = clock_ptr->now();
  const auto color = color::green();
  const auto m_scale = create_marker_scale(0.05, 0, 0);

  MarkerArray marker_array;

  auto marker = create_default_marker("map", curr_time, "", 0, line_list, m_scale, color);

  marker_array.markers.push_back(
    create_ego_sides_marker(output.ego_sides_from_footprints, marker, "ego_sides", base_link_z));
  marker_array.markers.push_back(create_side_to_boundary_marker(
    output.side_to_bound_projections.left, marker, "closest_to_left_side", base_link_z));
  marker_array.markers.push_back(create_side_to_boundary_marker(
    output.side_to_bound_projections.right, marker, "closest_to_right_side", base_link_z));

  return marker_array;
}

}  // namespace autoware::motion_velocity_planner::debug
