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

StopPlanner::StopPlanner(const rclcpp::Logger & logger) : logger_(logger)
{
}

std::vector<lanelet::ConstLineString3d> StopPlanner::collect_stop_lines(
  const lanelet::ConstLanelets & route_lanelets) const
{
  std::vector<lanelet::ConstLineString3d> stop_lines;
  std::unordered_set<lanelet::Id> added;

  const auto try_add = [&](const lanelet::ConstLineString3d & ls) {
    if (added.insert(ls.id()).second) {
      stop_lines.push_back(ls);
    }
  };

  for (const auto & lanelet : route_lanelets) {
    // 1. Explicit stop line road markings (e.g. painted stop lines, crosswalk stop lines)
    for (const auto & road_marking :
         lanelet.regulatoryElementsAs<lanelet::autoware::RoadMarking>()) {
      const auto & marking = road_marking->roadMarking();
      if (
        marking.attributeOr(lanelet::AttributeName::Type, "none") ==
        lanelet::AttributeValueString::StopLine) {
        try_add(marking);
      }
    }

    // 2. Stop lines referenced by traffic lights (signalized intersections)
    for (const auto & traffic_light :
         lanelet.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>()) {
      if (const auto stop_line = traffic_light->stopLine()) {
        try_add(*stop_line);
      }
    }

    // 3. Reference lines of traffic signs (e.g. stop signs at intersections)
    for (const auto & traffic_sign : lanelet.regulatoryElementsAs<lanelet::TrafficSign>()) {
      for (const auto & ref_line : traffic_sign->refLines()) {
        try_add(ref_line);
      }
    }
  }

  return stop_lines;
}

std::vector<lanelet::ConstLineString3d> StopPlanner::filter_stop_lines_on_trajectory(
  const std::vector<lanelet::ConstLineString3d> & stop_lines,
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

  std::vector<lanelet::ConstLineString3d> intersecting;
  for (const auto & stop_line : stop_lines) {
    const auto stop_line_2d = lanelet::utils::to2D(stop_line).basicLineString();
    if (boost::geometry::intersects(trajectory_line, stop_line_2d)) {
      intersecting.push_back(stop_line);
    }
  }
  return intersecting;
}

std::optional<double> StopPlanner::calc_nearest_stop_arc_length(
  const std::vector<lanelet::ConstLineString3d> & stop_lines,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points) const
{
  if (stop_lines.empty() || trajectory_points.size() < 2) {
    return std::nullopt;
  }

  lanelet::BasicLineString2d trajectory_line;
  trajectory_line.reserve(trajectory_points.size());
  for (const auto & point : trajectory_points) {
    trajectory_line.emplace_back(point.pose.position.x, point.pose.position.y);
  }

  std::optional<double> nearest_arc_length;
  for (const auto & stop_line : stop_lines) {
    const auto stop_line_2d = lanelet::utils::to2D(stop_line).basicLineString();

    std::vector<lanelet::BasicPoint2d> intersections;
    boost::geometry::intersection(trajectory_line, stop_line_2d, intersections);

    for (const auto & intersection : intersections) {
      geometry_msgs::msg::Point crossing;
      crossing.x = intersection.x();
      crossing.y = intersection.y();
      const double arc_length =
        autoware::motion_utils::calcSignedArcLength(trajectory_points, 0UL, crossing);
      if (!nearest_arc_length || arc_length < *nearest_arc_length) {
        nearest_arc_length = arc_length;
      }
    }
  }

  return nearest_arc_length;
}

visualization_msgs::msg::MarkerArray StopPlanner::create_stop_line_marker_array(
  const std::vector<lanelet::ConstLineString3d> & stop_lines) const
{
  constexpr float marker_thickness_m = 0.3f;
  return lanelet::visualization::lineStringsAsMarkerArray(
    stop_lines, "stop_lines", stop_line_color(), marker_thickness_m);
}

}  // namespace autoware::minimum_rule_based_planner
