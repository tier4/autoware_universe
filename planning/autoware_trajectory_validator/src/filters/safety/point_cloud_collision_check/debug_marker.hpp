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

#ifndef FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__DEBUG_MARKER_HPP_
#define FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__DEBUG_MARKER_HPP_

#include "planner_data_lite.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cstdio>
#include <string>
#include <utility>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

// Minimal debug state for planner_data / no_ground_pointcloud verification.
struct DebugData
{
  bool is_feasible{true};
  geometry_msgs::msg::Point ego_position{};
  std::string no_ground_frame_id{};
  std::size_t no_ground_point_count{0};
  std::size_t filtered_point_count{0};
};

namespace detail
{
inline std_msgs::msg::ColorRGBA rgba(
  const float r, const float g, const float b, const float a = 1.0f)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

inline Marker make_text(
  const std::string & ns, const int32_t id, const rclcpp::Time & stamp,
  const geometry_msgs::msg::Point & pos, const double z_offset, const double scale_z,
  const std_msgs::msg::ColorRGBA & color, const std::string & text)
{
  Marker m;
  m.header.frame_id = "map";
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = Marker::TEXT_VIEW_FACING;
  m.action = Marker::ADD;
  m.pose.position = pos;
  m.pose.position.z += z_offset;
  m.pose.orientation.w = 1.0;
  m.scale.z = scale_z;
  m.color = color;
  m.text = text;
  m.lifetime = rclcpp::Duration::from_seconds(0.0);
  return m;
}

inline int count_feasibility_markers(const MarkerArray & markers)
{
  constexpr const char * suffix = "/feasibility";
  const std::string s{suffix};
  return static_cast<int>(
    std::count_if(markers.markers.begin(), markers.markers.end(), [&s](const Marker & m) {
      return m.ns.size() >= s.size() && m.ns.compare(m.ns.size() - s.size(), s.size(), s) == 0;
    }));
}
}  // namespace detail

inline void emit_debug_markers(
  MarkerArray & markers, DebugData & debug, const PlannerData & planner_data,
  const bool is_feasible, const rclcpp::Time & stamp)
{
  const auto & no_ground = planner_data.no_ground_pointcloud.pointcloud;
  const auto filtered_ptr = planner_data.no_ground_pointcloud.get_filtered_pointcloud_ptr();

  debug.is_feasible = is_feasible;
  debug.ego_position = planner_data.current_odometry.pose.pose.position;
  debug.no_ground_frame_id =
    no_ground.header.frame_id.empty() ? std::string{"unknown"} : no_ground.header.frame_id;
  debug.no_ground_point_count = no_ground.size();
  debug.filtered_point_count = filtered_ptr ? filtered_ptr->size() : 0U;

  const bool first_candidate = markers.markers.empty();
  const int k = detail::count_feasibility_markers(markers);
  const auto color = is_feasible ? detail::rgba(0.2f, 1.0f, 0.3f) : detail::rgba(1.0f, 0.1f, 0.1f);

  // Filtered (corridor) points in map frame.
  if (filtered_ptr && !filtered_ptr->empty()) {
    Marker pts;
    pts.header.frame_id = "map";
    pts.header.stamp = stamp;
    pts.ns = std::to_string(k) + "/filtered";
    pts.id = static_cast<int32_t>(markers.markers.size());
    pts.type = Marker::POINTS;
    pts.action = Marker::ADD;
    pts.pose.orientation.w = 1.0;
    pts.scale.x = 0.2;
    pts.scale.y = 0.2;
    pts.color = detail::rgba(0.1f, 0.6f, 1.0f, 0.9f);
    pts.lifetime = rclcpp::Duration::from_seconds(0.0);
    pts.points.reserve(filtered_ptr->size());
    for (const auto & p : filtered_ptr->points) {
      geometry_msgs::msg::Point gp;
      gp.x = p.x;
      gp.y = p.y;
      gp.z = p.z;
      pts.points.push_back(gp);
    }
    markers.markers.push_back(std::move(pts));
  }

  // Per-candidate feasibility + no_ground summary.
  {
    char buf[192];
    std::snprintf(
      buf, sizeof(buf), "cand%d: %s | no_ground frame:%s pts:%zu | filtered:%zu", k,
      is_feasible ? "SAFE" : "STOP REQUIRED", debug.no_ground_frame_id.c_str(),
      debug.no_ground_point_count, debug.filtered_point_count);
    markers.markers.push_back(
      detail::make_text(
        std::to_string(k) + "/feasibility", static_cast<int32_t>(markers.markers.size()), stamp,
        debug.ego_position, 1.5 + 0.7 * static_cast<double>(k), 0.6, color, buf));
  }

  // One status banner per cycle (first candidate only).
  if (first_candidate) {
    const std::string text = std::string{"PCC: "} + (is_feasible ? "SAFE" : "STOP REQUIRED") +
                             " | no_ground frame:" + debug.no_ground_frame_id +
                             " pts:" + std::to_string(debug.no_ground_point_count) +
                             " | filtered_pts:" + std::to_string(debug.filtered_point_count);
    markers.markers.push_back(
      detail::make_text(
        "status", static_cast<int32_t>(markers.markers.size()), stamp, debug.ego_position, 3.0, 0.9,
        color, text));
  }
}

}  // namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check

#endif  // FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__DEBUG_MARKER_HPP_
