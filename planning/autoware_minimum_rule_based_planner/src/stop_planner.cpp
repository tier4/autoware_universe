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

#include "stop_planner.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>

#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

namespace
{
std_msgs::msg::ColorRGBA stop_line_color()
{
  std_msgs::msg::ColorRGBA c;
  c.r = 1.0f;
  c.g = 0.0f;
  c.b = 0.0f;
  c.a = 0.8f;
  return c;
}
}  // namespace

bool is_possibility_type(StopLineType type)
{
  // Signals (traffic lights) are "possibility" targets: the go trajectory need not stop for them.
  // Painted stop lines and stop signs are mandatory stop targets.
  return type == StopLineType::TrafficLight;
}

StopPlanner::StopPlanner(const rclcpp::Logger & logger) : logger_(logger)
{
}

std::vector<StopLine> StopPlanner::collect_stop_lines(
  const lanelet::ConstLanelets & route_lanelets) const
{
  std::vector<StopLine> stop_lines;
  std::unordered_set<lanelet::Id> added;

  const auto try_add = [&](const lanelet::ConstLineString3d & ls, const StopLineType type) {
    if (added.insert(ls.id()).second) {
      stop_lines.push_back(StopLine{ls, type});
    }
  };

  for (const auto & lanelet : route_lanelets) {
    // 1. Explicit stop line road markings (e.g. painted stop lines, crosswalk stop lines)
    for (const auto & road_marking :
         lanelet.regulatoryElementsAs<lanelet::autoware::RoadMarking>()) {
      const auto & marking = road_marking->roadMarking();
      // NOTE: use std::string so the comparison is by content. Passing a string literal would
      // deduce const char* and compare pointers (never equal).
      if (
        marking.attributeOr(lanelet::AttributeName::Type, std::string("none")) ==
        lanelet::AttributeValueString::StopLine) {
        try_add(marking, StopLineType::RoadMarking);
      }
    }

    // 2. Stop lines referenced by traffic lights (signalized intersections)
    for (const auto & traffic_light :
         lanelet.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>()) {
      if (const auto stop_line = traffic_light->stopLine()) {
        try_add(*stop_line, StopLineType::TrafficLight);
      }
    }

    // 3. Reference lines of traffic signs (e.g. stop signs at intersections)
    for (const auto & traffic_sign : lanelet.regulatoryElementsAs<lanelet::TrafficSign>()) {
      for (const auto & ref_line : traffic_sign->refLines()) {
        try_add(ref_line, StopLineType::TrafficSign);
      }
    }
  }

  return stop_lines;
}

std::vector<StopLine> StopPlanner::filter_stop_lines_on_trajectory(
  const std::vector<StopLine> & stop_lines,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points) const
{
  // Need at least one segment to intersect against.
  if (stop_lines.empty() || trajectory_points.size() < 2) {
    return {};
  }

  lanelet::BasicLineString2d trajectory_line;
  trajectory_line.reserve(trajectory_points.size());
  for (const auto & point : trajectory_points) {
    trajectory_line.emplace_back(point.pose.position.x, point.pose.position.y);
  }

  std::vector<StopLine> intersecting;
  for (const auto & stop_line : stop_lines) {
    const auto stop_line_2d = lanelet::utils::to2D(stop_line.line).basicLineString();
    if (boost::geometry::intersects(trajectory_line, stop_line_2d)) {
      intersecting.push_back(stop_line);
    }
  }
  return intersecting;
}

std::optional<double> StopPlanner::select_stop_arc_length(
  const std::vector<StopLine> & stop_lines,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points,
  const double ego_velocity, const StopSelectionParams & params,
  const bool include_possibility) const
{
  if (stop_lines.empty() || trajectory_points.size() < 2) {
    return std::nullopt;
  }

  lanelet::BasicLineString2d trajectory_line;
  trajectory_line.reserve(trajectory_points.size());
  for (const auto & point : trajectory_points) {
    trajectory_line.emplace_back(point.pose.position.x, point.pose.position.y);
  }

  // Nearest stop point (already adjusted for vehicle front offset and stop margin) among the
  // allowed stop line types.
  std::optional<double> nearest_stop_point_arc_length;
  for (const auto & stop_line : stop_lines) {
    // The go trajectory (include_possibility == false) ignores possibility targets (e.g. signals).
    if (!include_possibility && is_possibility_type(stop_line.type)) {
      continue;
    }

    const auto stop_line_2d = lanelet::utils::to2D(stop_line.line).basicLineString();
    std::vector<lanelet::BasicPoint2d> intersections;
    boost::geometry::intersection(trajectory_line, stop_line_2d, intersections);

    for (const auto & intersection : intersections) {
      geometry_msgs::msg::Point crossing;
      crossing.x = intersection.x();
      crossing.y = intersection.y();
      const double crossing_arc_length =
        autoware::motion_utils::calcSignedArcLength(trajectory_points, 0UL, crossing);
      // Stop so the vehicle front bumper stops a margin before the crossing point.
      const double stop_point_arc_length =
        crossing_arc_length - params.base_link_to_front - params.stop_margin_distance;
      if (stop_point_arc_length <= 0.0) {
        continue;
      }
      if (
        !nearest_stop_point_arc_length || stop_point_arc_length < *nearest_stop_point_arc_length) {
        nearest_stop_point_arc_length = stop_point_arc_length;
      }
    }
  }

  if (!nearest_stop_point_arc_length) {
    return std::nullopt;
  }

  // Reachability: keep the stop point only if the vehicle can decelerate to a stop before it,
  // given the current speed and the maximum deceleration.
  const double braking_distance = ego_velocity * ego_velocity / (2.0 * params.max_deceleration);
  if (*nearest_stop_point_arc_length < braking_distance) {
    return std::nullopt;
  }

  return nearest_stop_point_arc_length;
}

visualization_msgs::msg::MarkerArray StopPlanner::create_stop_line_marker_array(
  const std::vector<StopLine> & stop_lines) const
{
  constexpr float marker_thickness_m = 0.3f;
  std::vector<lanelet::ConstLineString3d> lines;
  lines.reserve(stop_lines.size());
  for (const auto & stop_line : stop_lines) {
    lines.push_back(stop_line.line);
  }
  return lanelet::visualization::lineStringsAsMarkerArray(
    lines, "stop_lines", stop_line_color(), marker_thickness_m);
}

}  // namespace autoware::minimum_rule_based_planner
