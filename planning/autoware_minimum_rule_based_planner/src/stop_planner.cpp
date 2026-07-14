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
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

namespace
{
std_msgs::msg::ColorRGBA make_color(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}
}  // namespace

bool is_possibility_type(StopLineType type)
{
  // Mandatory stop targets (停止対象箇所): the go trajectory also stops for these.
  // Everything else is a possibility target (停止可能性対象箇所): only the stop trajectory stops.
  switch (type) {
    case StopLineType::StopLine:
    case StopLineType::Walkway:
      return false;
    case StopLineType::Crosswalk:
    case StopLineType::TrafficLight:
    case StopLineType::Intersection:
    case StopLineType::PrivateArea:
      return true;
  }
  return true;
}

namespace
{
bool is_private(const lanelet::ConstLanelet & lanelet)
{
  return lanelet.attributeOr(lanelet::AttributeNamesString::Location, std::string("")) ==
         lanelet::AttributeValueString::Private;
}

// Synthesize the entry edge of a lanelet (the segment joining the first points of its left and
// right bounds) as a virtual stop line for zones without an explicit painted line.
lanelet::ConstLineString3d make_entry_edge(const lanelet::ConstLanelet & lanelet)
{
  const auto & lp = lanelet.leftBound().front();
  const auto & rp = lanelet.rightBound().front();
  return lanelet::LineString3d(
    lanelet::InvalId, {lanelet::Point3d(lanelet::InvalId, lp.x(), lp.y(), lp.z()),
                       lanelet::Point3d(lanelet::InvalId, rp.x(), rp.y(), rp.z())});
}

// Concatenate the centerlines of the given lanelets into a single reference path used to reason
// about travel direction (e.g. which side of a crosswalk the route enters first).
std::vector<autoware_planning_msgs::msg::TrajectoryPoint> build_route_path(
  const lanelet::ConstLanelets & lanelets)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> path;
  for (const auto & lanelet : lanelets) {
    for (const auto & point : lanelet.centerline()) {
      autoware_planning_msgs::msg::TrajectoryPoint tp;
      tp.pose.position.x = point.x();
      tp.pose.position.y = point.y();
      tp.pose.position.z = point.z();
      path.push_back(tp);
    }
  }
  return path;
}

// Nearest arc length (from the path start) at which the path crosses the given line, or nullopt.
std::optional<double> nearest_crossing_arc(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & path,
  const lanelet::ConstLineString3d & line)
{
  if (path.size() < 2) return std::nullopt;

  lanelet::BasicLineString2d path_2d;
  path_2d.reserve(path.size());
  for (const auto & point : path) {
    path_2d.emplace_back(point.pose.position.x, point.pose.position.y);
  }

  const auto line_2d = lanelet::utils::to2D(line).basicLineString();
  std::vector<lanelet::BasicPoint2d> intersections;
  boost::geometry::intersection(path_2d, line_2d, intersections);

  std::optional<double> nearest;
  for (const auto & intersection : intersections) {
    geometry_msgs::msg::Point crossing;
    crossing.x = intersection.x();
    crossing.y = intersection.y();
    const double arc = autoware::motion_utils::calcSignedArcLength(path, 0UL, crossing);
    if (!nearest || arc < *nearest) nearest = arc;
  }
  return nearest;
}

// The entry-side bound of a crosswalk/walkway lanelet: whichever of the left/right bounds the
// route path crosses first. Returns nullopt if the path crosses neither (i.e. not on the route).
std::optional<lanelet::ConstLineString3d> entry_side_bound(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & route_path,
  const lanelet::ConstLanelet & lanelet)
{
  const auto left_arc = nearest_crossing_arc(route_path, lanelet.leftBound());
  const auto right_arc = nearest_crossing_arc(route_path, lanelet.rightBound());
  if (!left_arc && !right_arc) return std::nullopt;
  if (left_arc && (!right_arc || *left_arc <= *right_arc)) return lanelet.leftBound();
  return lanelet.rightBound();
}

bool is_crosswalk_or_walkway(const lanelet::ConstLanelet & lanelet, bool & is_walkway)
{
  const auto subtype = lanelet.attributeOr(lanelet::AttributeNamesString::Subtype, std::string(""));
  is_walkway = subtype == lanelet::AttributeValueString::Walkway ||
               subtype == lanelet::AttributeValueString::SharedWalkway;
  return is_walkway || subtype == lanelet::AttributeValueString::Crosswalk;
}
}  // namespace

StopPlanner::StopPlanner(const rclcpp::Logger & logger) : logger_(logger)
{
}

std::vector<StopLine> StopPlanner::collect_stop_lines(const RouteContext & route_context) const
{
  const auto & route_lanelets = route_context.route_lanelets;
  const auto & preferred_lanelets = route_context.preferred_lanelets;

  std::vector<StopLine> stop_lines;
  // Dedup key is (type, id): the id is the line string id for map lines, or the source lanelet id
  // for synthesized entry edges (which all share lanelet::InvalId as a line string id).
  std::set<std::pair<int, lanelet::Id>> added;

  const auto add_line =
    [&](const lanelet::ConstLineString3d & ls, const StopLineType type, const lanelet::Id key_id) {
      if (added.insert({static_cast<int>(type), key_id}).second) {
        stop_lines.push_back(StopLine{ls, type});
      }
    };

  for (const auto & lanelet : route_lanelets) {
    // 1. 一時停止線: explicit stop line road markings.
    for (const auto & road_marking :
         lanelet.regulatoryElementsAs<lanelet::autoware::RoadMarking>()) {
      const auto & marking = road_marking->roadMarking();
      // NOTE: use std::string so the comparison is by content. Passing a string literal would
      // deduce const char* and compare pointers (never equal).
      if (
        marking.attributeOr(lanelet::AttributeName::Type, std::string("none")) ==
        lanelet::AttributeValueString::StopLine) {
        add_line(marking, StopLineType::StopLine, marking.id());
      }
    }

    // 1'. 一時停止線: reference lines of traffic signs (e.g. stop signs).
    for (const auto & traffic_sign : lanelet.regulatoryElementsAs<lanelet::TrafficSign>()) {
      for (const auto & ref_line : traffic_sign->refLines()) {
        add_line(ref_line, StopLineType::StopLine, ref_line.id());
      }
    }

    // 2. 信号: stop lines referenced by traffic lights.
    for (const auto & traffic_light :
         lanelet.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>()) {
      if (const auto stop_line = traffic_light->stopLine()) {
        add_line(*stop_line, StopLineType::TrafficLight, stop_line->id());
      }
    }

    // 3. 交差点: lanelets carrying a turn_direction attribute.
    if (lanelet.hasAttribute("turn_direction")) {
      add_line(make_entry_edge(lanelet), StopLineType::Intersection, lanelet.id());
    }
  }

  // 4. 横断歩道 / 歩道: crosswalk / walkway lanelets that geometrically cross the route path.
  // Walkways carry no regulatory element, so we search the whole map layer by subtype and keep the
  // ones the route path crosses. Only the entry-side bound is used as the stop line.
  if (route_context.lanelet_map_ptr) {
    const auto route_path = build_route_path(preferred_lanelets);
    for (const auto & lanelet : route_context.lanelet_map_ptr->laneletLayer) {
      bool is_walkway = false;
      if (!is_crosswalk_or_walkway(lanelet, is_walkway)) continue;
      const auto entry_bound = entry_side_bound(route_path, lanelet);
      if (!entry_bound) continue;  // not crossed by the route path
      const auto type = is_walkway ? StopLineType::Walkway : StopLineType::Crosswalk;
      add_line(*entry_bound, type, entry_bound->id());
    }
  }

  // 5. 私有地入退出: transitions of the location attribute between private and non-private along
  // the preferred lane sequence (a single ordered chain, unlike the flattened route_lanelets which
  // interleaves parallel lanes). Stop at the entry edge of the lanelet being entered.
  for (size_t i = 1; i < preferred_lanelets.size(); ++i) {
    if (is_private(preferred_lanelets[i]) != is_private(preferred_lanelets[i - 1])) {
      add_line(
        make_entry_edge(preferred_lanelets[i]), StopLineType::PrivateArea,
        preferred_lanelets[i].id());
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

  // Reachability: the vehicle can only stop at points at least a braking distance ahead, given the
  // current speed and the maximum deceleration.
  const double braking_distance = ego_velocity * ego_velocity / (2.0 * params.max_deceleration);

  // Nearest *reachable* stop point (already adjusted for vehicle front offset and stop margin)
  // among the allowed stop line types. Unreachable candidates (too close to stop for) are skipped
  // so that a farther but still reachable target is used instead of giving up entirely.
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
      // Skip stop points behind the vehicle and those too close to stop for.
      if (stop_point_arc_length <= 0.0 || stop_point_arc_length < braking_distance) {
        continue;
      }
      if (
        !nearest_stop_point_arc_length || stop_point_arc_length < *nearest_stop_point_arc_length) {
        nearest_stop_point_arc_length = stop_point_arc_length;
      }
    }
  }

  return nearest_stop_point_arc_length;
}

visualization_msgs::msg::MarkerArray StopPlanner::create_stop_line_marker_array(
  const std::vector<StopLine> & stop_lines) const
{
  constexpr float marker_thickness_m = 0.3f;

  // Split the candidates into mandatory stop targets (停止対象箇所) and possibility targets
  // (停止可能性対象箇所) so they can be told apart by namespace and color in rviz.
  std::vector<lanelet::ConstLineString3d> mandatory_lines;
  std::vector<lanelet::ConstLineString3d> possibility_lines;
  for (const auto & stop_line : stop_lines) {
    if (is_possibility_type(stop_line.type)) {
      possibility_lines.push_back(stop_line.line);
    } else {
      mandatory_lines.push_back(stop_line.line);
    }
  }

  // Raise the markers above the map surface so they are not hidden by the map in a
  // top-down orthographic view.
  constexpr double marker_z_offset_m = 0.1;

  visualization_msgs::msg::MarkerArray marker_array;
  const auto append = [&marker_array](visualization_msgs::msg::MarkerArray markers) {
    for (auto & marker : markers.markers) {
      marker.pose.position.z += marker_z_offset_m;
      for (auto & point : marker.points) {
        point.z += marker_z_offset_m;
      }
    }
    marker_array.markers.insert(
      marker_array.markers.end(), markers.markers.begin(), markers.markers.end());
  };

  // Mandatory stop targets: red.
  append(
    lanelet::visualization::lineStringsAsMarkerArray(
      mandatory_lines, "stop_target", make_color(1.0f, 0.0f, 0.0f, 0.8f), marker_thickness_m));
  // Possibility stop targets: yellow.
  append(
    lanelet::visualization::lineStringsAsMarkerArray(
      possibility_lines, "stop_possibility", make_color(1.0f, 1.0f, 0.0f, 0.8f),
      marker_thickness_m));

  return marker_array;
}

}  // namespace autoware::minimum_rule_based_planner
