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

#include "debug_marker.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace autoware::goal_selector
{
namespace
{
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_scale;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

constexpr double line_width = 0.2;
constexpr double text_height_offset = 1.0;

Marker create_text_marker(
  const std::string & ns, const int32_t id, const std::string & text,
  const geometry_msgs::msg::Point & position, const rclcpp::Time & stamp)
{
  auto marker = create_default_marker(
    "map", stamp, ns, id, Marker::TEXT_VIEW_FACING, create_marker_scale(0.0, 0.0, 1.0),
    create_marker_color(1.0, 1.0, 1.0, 0.999));
  marker.pose.position = position;
  marker.pose.position.z += text_height_offset;
  marker.text = text;
  return marker;
}

// The trigger has no elevation of its own, so the circle is drawn at the elevation of the
// first goal candidate.
Marker create_trigger_area(const Trigger & trigger, const int32_t id, const rclcpp::Time & stamp)
{
  auto marker = create_default_marker(
    "map", stamp, "trigger_area", id, Marker::LINE_STRIP, create_marker_scale(line_width, 0.0, 0.0),
    create_marker_color(1.0, 0.9, 0.2, 0.999));

  constexpr int32_t steps = 36;
  for (int32_t i = 0; i <= steps; ++i) {
    const double theta = 2.0 * M_PI * i / steps;
    geometry_msgs::msg::Point point;
    point.x = trigger.x + trigger.radius * std::cos(theta);
    point.y = trigger.y + trigger.radius * std::sin(theta);
    point.z =
      trigger.goal_candidates.empty() ? 0.0 : trigger.goal_candidates.front().pose.position.z;
    marker.points.push_back(point);
  }
  return marker;
}

Marker create_candidate_area(
  const GoalCandidate & candidate, const int32_t id, const bool is_vacant,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double margin,
  const rclcpp::Time & stamp)
{
  const auto color = is_vacant ? create_marker_color(0.2, 1.0, 0.2, 0.999)
                               : create_marker_color(1.0, 0.2, 0.2, 0.999);
  auto marker = create_default_marker(
    "map", stamp, "goal_candidate", id, Marker::LINE_STRIP,
    create_marker_scale(line_width, 0.0, 0.0), color);

  const auto footprint = autoware_utils::transform_vector(
    vehicle_info.createFootprint(margin), autoware_utils::pose2transform(candidate.pose));
  for (const auto & vertex : footprint) {
    geometry_msgs::msg::Point point;
    point.x = vertex.x();
    point.y = vertex.y();
    point.z = candidate.pose.position.z;
    marker.points.push_back(point);
  }
  return marker;
}

// Labels are placed ahead of the candidate and staggered, so that candidates standing side by
// side do not print their labels on top of each other.
geometry_msgs::msg::Point create_label_position(
  const GoalCandidate & candidate, const size_t index,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double margin)
{
  const double offset =
    vehicle_info.max_longitudinal_offset_m + margin + 1.0 + 2.0 * static_cast<double>(index);
  return autoware_utils::calc_offset_pose(candidate.pose, offset, 0.0, 0.0).position;
}

}  // namespace

MarkerArray create_debug_marker_array(
  const std::vector<Trigger> & triggers, const std::map<std::string, bool> & vacancies,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double margin,
  const rclcpp::Time & stamp)
{
  MarkerArray markers;
  int32_t candidate_id = 0;

  for (size_t i = 0; i < triggers.size(); ++i) {
    const auto & trigger = triggers.at(i);
    const auto area = create_trigger_area(trigger, static_cast<int32_t>(i), stamp);
    markers.markers.push_back(area);
    markers.markers.push_back(create_text_marker(
      "trigger_label", static_cast<int32_t>(i), trigger.name, area.points.front(), stamp));

    for (size_t j = 0; j < trigger.goal_candidates.size(); ++j) {
      const auto & candidate = trigger.goal_candidates.at(j);
      markers.markers.push_back(create_candidate_area(
        candidate, candidate_id, vacancies.at(candidate.name), vehicle_info, margin, stamp));
      markers.markers.push_back(create_text_marker(
        "goal_candidate_label", candidate_id, candidate.name,
        create_label_position(candidate, j, vehicle_info, margin), stamp));
      ++candidate_id;
    }
  }
  return markers;
}

}  // namespace autoware::goal_selector
