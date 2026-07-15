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

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>

#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <map>
#include <memory>
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

const char * to_string(const StopLineType type)
{
  switch (type) {
    case StopLineType::StopLine:
      return "stop_line";
    case StopLineType::Walkway:
      return "walkway";
    case StopLineType::Crosswalk:
      return "crosswalk";
    case StopLineType::TrafficLight:
      return "traffic_light";
    case StopLineType::Intersection:
      return "intersection";
    case StopLineType::PrivateArea:
      return "private_area";
  }
  return "unknown";
}

bool is_crosswalk_or_walkway(const lanelet::ConstLanelet & lanelet, bool & is_walkway)
{
  const auto subtype = lanelet.attributeOr(lanelet::AttributeNamesString::Subtype, std::string(""));
  is_walkway = subtype == lanelet::AttributeValueString::Walkway ||
               subtype == lanelet::AttributeValueString::SharedWalkway;
  return is_walkway || subtype == lanelet::AttributeValueString::Crosswalk;
}

// Explicit stop line for a crosswalk/walkway lanelet from a RoadMarking regulatory element whose
// "crosswalk_id" attribute points back to the lanelet (upstream getStopLineFromMap equivalent).
std::optional<lanelet::ConstLineString3d> road_marking_stop_line_for_crosswalk(
  const lanelet::ConstLanelet & crosswalk)
{
  for (const auto & road_marking :
       crosswalk.regulatoryElementsAs<lanelet::autoware::RoadMarking>()) {
    const auto & marking = road_marking->roadMarking();
    const auto type = marking.attributeOr(lanelet::AttributeName::Type, std::string("none"));
    const auto target_id = marking.attributeOr("crosswalk_id", lanelet::Id(lanelet::InvalId));
    if (type == lanelet::AttributeValueString::StopLine && target_id == crosswalk.id()) {
      return marking;
    }
  }
  return std::nullopt;
}
}  // namespace

StopPlanner::StopPlanner(
  const rclcpp::Logger & logger, std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
: logger_(logger),
  time_keeper_(
    time_keeper ? std::move(time_keeper) : std::make_shared<autoware_utils_debug::TimeKeeper>())
{
}

std::vector<StopLine> StopPlanner::collect_stop_lines(const RouteContext & route_context) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & route_lanelets = route_context.route_lanelets;
  const auto & preferred_lanelets = route_context.preferred_lanelets;

  std::vector<StopLine> stop_lines;
  // Dedup key is (type, id): the id is the line string id for map lines, or the source lanelet id
  // for synthesized entry edges (which all share lanelet::InvalId as a line string id).
  std::set<std::pair<int, lanelet::Id>> added;

  const auto add_line = [&](
                          const lanelet::ConstLineString3d & ls, const StopLineType type,
                          const lanelet::Id key_id, const bool without_explicit_stop_line = false) {
    if (added.insert({static_cast<int>(type), key_id}).second) {
      stop_lines.push_back(StopLine{ls, type, without_explicit_stop_line});
    }
  };

  {
    autoware_utils_debug::ScopedTimeTrack st_route("route_regulatory_elements", *time_keeper_);
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
  }

  // 4. 横断歩道 / 歩道: crosswalk / walkway lanelets overlapping the route lanelets, found via
  // the map's spatial index (RTree) per route lanelet plus a 2D polygon-overlap check — the same
  // approach as the upstream behavior_velocity crosswalk/walkway modules
  // (RoutingGraphContainer::conflictingInGraph), without needing a pedestrian routing graph.
  if (route_context.lanelet_map_ptr) {
    autoware_utils_debug::ScopedTimeTrack st_scan("crosswalk_walkway_search", *time_keeper_);

    // Explicit stop lines from crosswalk regulatory elements referenced by the route lanelets:
    // crosswalk lanelet id -> stop line.
    std::map<lanelet::Id, lanelet::ConstLineString3d> regelem_stop_lines;
    for (const auto & lanelet : route_lanelets) {
      for (const auto & crosswalk_regelem :
           lanelet.regulatoryElementsAs<lanelet::autoware::Crosswalk>()) {
        const auto crosswalk_stop_lines = crosswalk_regelem->stopLines();
        if (!crosswalk_stop_lines.empty()) {
          regelem_stop_lines.emplace(
            crosswalk_regelem->crosswalkLanelet().id(), crosswalk_stop_lines.front());
        }
      }
    }

    const auto route_path = build_route_path(preferred_lanelets);
    std::set<lanelet::Id> found_crosswalks;
    for (const auto & route_lanelet : preferred_lanelets) {
      const auto nearby_lanelets = route_context.lanelet_map_ptr->laneletLayer.search(
        lanelet::geometry::boundingBox2d(route_lanelet));
      for (const auto & candidate : nearby_lanelets) {
        bool is_walkway = false;
        if (!is_crosswalk_or_walkway(candidate, is_walkway)) continue;
        if (found_crosswalks.count(candidate.id()) > 0) continue;
        if (!lanelet::geometry::overlaps2d(route_lanelet, candidate)) continue;
        found_crosswalks.insert(candidate.id());

        const auto type = is_walkway ? StopLineType::Walkway : StopLineType::Crosswalk;
        // Stop line priority (upstream semantics): an explicit stop line from the crosswalk
        // regulatory element, then a RoadMarking bound to the crosswalk by its "crosswalk_id"
        // attribute, and only as a fallback the entry-side bound of the crosswalk lanelet.
        if (const auto it = regelem_stop_lines.find(candidate.id());
            it != regelem_stop_lines.end()) {
          add_line(it->second, type, it->second.id());
        } else if (const auto marking_line = road_marking_stop_line_for_crosswalk(candidate)) {
          add_line(*marking_line, type, marking_line->id());
        } else if (const auto entry_bound = entry_side_bound(route_path, candidate)) {
          add_line(*entry_bound, type, entry_bound->id(), /*without_explicit_stop_line=*/true);
        }
      }
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
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

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
  const geometry_msgs::msg::Pose & ego_pose, const double ego_velocity,
  const double ego_acceleration, const StopSelectionParams & params,
  const bool include_possibility) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (stop_lines.empty() || trajectory_points.size() < 2) {
    return std::nullopt;
  }

  lanelet::BasicLineString2d trajectory_line;
  trajectory_line.reserve(trajectory_points.size());
  for (const auto & point : trajectory_points) {
    trajectory_line.emplace_back(point.pose.position.x, point.pose.position.y);
  }

  // Reachability: the vehicle can only stop at points at least a braking distance ahead. The
  // braking distance accounts for the jerk-limited ramp-up to the maximum deceleration; the
  // fallback formula is only reachable with degenerate limits, which parameter validation
  // prevents. The remaining distance is measured from the ego position, not from the trajectory
  // start, which lies a backward path length behind the vehicle.
  const double braking_distance =
    autoware::motion_utils::calculate_stop_distance(
      ego_velocity, ego_acceleration, params.max_deceleration, params.max_jerk)
      .value_or(ego_velocity * ego_velocity / (2.0 * params.max_deceleration));
  const double ego_arc_length =
    autoware::motion_utils::calcSignedArcLength(trajectory_points, 0UL, ego_pose.position);

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
      // Stop so the vehicle front bumper stops a margin before the crossing point. Crosswalks and
      // walkways without an explicit stop line use the larger stop_distance_from_crosswalk margin
      // (upstream behavior_velocity semantics).
      const double stop_margin = stop_line.without_explicit_stop_line
                                   ? params.stop_distance_from_crosswalk
                                   : params.stop_margin_distance;
      const double stop_point_arc_length =
        crossing_arc_length - params.base_link_to_front - stop_margin;
      // Skip stop points the vehicle has already passed and those too close to stop for with the
      // maximum deceleration (both judged on the remaining distance from ego).
      const double distance_to_stop_point = stop_point_arc_length - ego_arc_length;
      if (distance_to_stop_point <= 0.0 || distance_to_stop_point < braking_distance) {
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
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

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

  // Type labels: view-facing text floating above the middle of each stop line. The height is
  // staggered per type (1.0 m + 0.3 m per enum value) so labels of overlapping lines of
  // different types do not cover each other.
  constexpr double text_z_offset_m = 1.0;
  constexpr double text_z_step_per_type_m = 0.3;
  constexpr double text_height_m = 0.5;
  int32_t text_id = 0;
  for (const auto & stop_line : stop_lines) {
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.ns = "stop_line_type";
    text_marker.id = text_id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    const auto & front = stop_line.line.front();
    const auto & back = stop_line.line.back();
    text_marker.pose.position.x = 0.5 * (front.x() + back.x());
    text_marker.pose.position.y = 0.5 * (front.y() + back.y());
    text_marker.pose.position.z = 0.5 * (front.z() + back.z()) + text_z_offset_m +
                                  text_z_step_per_type_m * static_cast<double>(stop_line.type);
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = text_height_m;
    text_marker.color = make_color(1.0f, 1.0f, 1.0f, 0.9f);
    text_marker.text = to_string(stop_line.type);
    marker_array.markers.push_back(text_marker);
  }

  return marker_array;
}

}  // namespace autoware::minimum_rule_based_planner
