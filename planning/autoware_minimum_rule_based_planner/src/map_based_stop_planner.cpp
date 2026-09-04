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

#include "map_based_stop_planner.hpp"

#include <autoware/lanelet2_utils/kind.hpp>
#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory_processor/trajectory_modifier_utils/utils.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <tf2/utils.h>

#include <algorithm>
#include <array>
#include <cmath>
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
  switch (type) {
    case StopLineType::StopLine:
    case StopLineType::Walkway:
      return false;
    case StopLineType::Crosswalk:
    case StopLineType::TrafficLight:
    case StopLineType::Intersection:
    case StopLineType::PrivateArea:
    case StopLineType::RoadShoulder:
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
// NOTE(odashima): synthesized line ids must be unique and negative. The marker visualization
// (lineStringsAsMarkerArray) de-duplicates line strings by id, so sharing InvalId would drop every
// synthesized line but the first; negative ids can never collide with real map ids.
lanelet::ConstLineString3d make_entry_edge(const lanelet::ConstLanelet & lanelet)
{
  const auto & lp = lanelet.leftBound().front();
  const auto & rp = lanelet.rightBound().front();
  return lanelet::LineString3d(
    -lanelet.id(), {lanelet::Point3d(lanelet::InvalId, lp.x(), lp.y(), lp.z()),
                    lanelet::Point3d(lanelet::InvalId, rp.x(), rp.y(), rp.z())});
}

// Point at half the arc length of the line string; unlike the chord midpoint, it always lies on
// the line even when it bends.
lanelet::BasicPoint3d arc_length_midpoint(const lanelet::ConstLineString3d & line)
{
  if (line.empty()) return lanelet::BasicPoint3d(0.0, 0.0, 0.0);

  double total_length = 0.0;
  for (size_t i = 0; i + 1 < line.size(); ++i) {
    total_length += (line[i + 1].basicPoint() - line[i].basicPoint()).norm();
  }

  const double half_length = 0.5 * total_length;
  double accumulated = 0.0;
  for (size_t i = 0; i + 1 < line.size(); ++i) {
    const auto p0 = line[i].basicPoint();
    const auto p1 = line[i + 1].basicPoint();
    const double segment_length = (p1 - p0).norm();
    if (segment_length > 0.0 && accumulated + segment_length >= half_length) {
      const double ratio = (half_length - accumulated) / segment_length;
      return p0 + ratio * (p1 - p0);
    }
    accumulated += segment_length;
  }
  return line.back().basicPoint();
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
    case StopLineType::RoadShoulder:
      return "road_shoulder";
  }
  return "unknown";
}

// Vehicle footprint rectangle at the given pose, counter-clockwise and open as boost expects for
// lanelet::BasicPolygon2d.
lanelet::BasicPolygon2d make_vehicle_footprint(
  const geometry_msgs::msg::Pose & pose, const VehicleInfo & vehicle_info)
{
  const double yaw = tf2::getYaw(pose.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const std::array<std::pair<double, double>, 4> corners{
    {{vehicle_info.max_longitudinal_offset_m, vehicle_info.max_lateral_offset_m},
     {vehicle_info.min_longitudinal_offset_m, vehicle_info.max_lateral_offset_m},
     {vehicle_info.min_longitudinal_offset_m, vehicle_info.min_lateral_offset_m},
     {vehicle_info.max_longitudinal_offset_m, vehicle_info.min_lateral_offset_m}}};

  lanelet::BasicPolygon2d footprint;
  footprint.reserve(corners.size());
  for (const auto & [x, y] : corners) {
    footprint.emplace_back(
      pose.position.x + cos_yaw * x - sin_yaw * y, pose.position.y + sin_yaw * x + cos_yaw * y);
  }
  return footprint;
}

lanelet::BoundingBox2d bounding_box_2d(const lanelet::BasicPolygon2d & polygon)
{
  lanelet::BoundingBox2d box;
  for (const auto & point : polygon) {
    box.extend(point);
  }
  return box;
}

bool is_crosswalk_or_walkway(const lanelet::ConstLanelet & lanelet, bool & is_walkway)
{
  const auto subtype = lanelet.attributeOr(lanelet::AttributeNamesString::Subtype, std::string(""));
  is_walkway = subtype == lanelet::AttributeValueString::Walkway ||
               subtype == lanelet::AttributeValueString::SharedWalkway;
  return is_walkway || subtype == lanelet::AttributeValueString::Crosswalk;
}

// Arc-length range [begin, end] of each lanelet's centerline on the concatenated route path.
std::vector<std::pair<double, double>> lanelet_arc_ranges(const lanelet::ConstLanelets & lanelets)
{
  std::vector<std::pair<double, double>> ranges;
  ranges.reserve(lanelets.size());
  double accumulated = 0.0;
  for (const auto & lanelet : lanelets) {
    double length = 0.0;
    const auto & centerline = lanelet.centerline();
    for (size_t i = 0; i + 1 < centerline.size(); ++i) {
      length += std::hypot(
        centerline[i + 1].x() - centerline[i].x(), centerline[i + 1].y() - centerline[i].y());
    }
    ranges.emplace_back(accumulated, accumulated + length);
    accumulated += length;
  }
  return ranges;
}

// Arc lengths (on the route path) where the path crosses the outline of the given lanelet
// polygon, restricted to [min_arc, max_arc].
std::vector<double> lanelet_outline_crossing_arcs(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & route_path,
  const lanelet::ConstLanelet & lanelet, const double min_arc, const double max_arc)
{
  std::vector<double> arcs;
  if (route_path.size() < 2) return arcs;

  lanelet::BasicLineString2d path_2d;
  path_2d.reserve(route_path.size());
  for (const auto & point : route_path) {
    path_2d.emplace_back(point.pose.position.x, point.pose.position.y);
  }

  const auto outline = lanelet.polygon2d().basicPolygon();
  lanelet::BasicLineString2d ring(outline.begin(), outline.end());
  if (ring.empty()) return arcs;
  ring.push_back(ring.front());  // close the ring

  std::vector<lanelet::BasicPoint2d> crossings;
  boost::geometry::intersection(path_2d, ring, crossings);
  for (const auto & crossing : crossings) {
    geometry_msgs::msg::Point point;
    point.x = crossing.x();
    point.y = crossing.y();
    const double arc = autoware::motion_utils::calcSignedArcLength(route_path, 0UL, point);
    if (arc >= min_arc && arc <= max_arc) {
      arcs.push_back(arc);
    }
  }
  return arcs;
}

// Synthesize a stop line perpendicular to the route path at the given arc length.
lanelet::ConstLineString3d make_perpendicular_line(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & route_path, const double arc,
  const double half_width, const lanelet::Id id)
{
  double accumulated = 0.0;
  for (size_t i = 0; i + 1 < route_path.size(); ++i) {
    const auto & p0 = route_path[i].pose.position;
    const auto & p1 = route_path[i + 1].pose.position;
    const double segment_length = std::hypot(p1.x - p0.x, p1.y - p0.y);
    const bool is_last_segment = (i + 2 == route_path.size());
    if (segment_length > 0.0 && (accumulated + segment_length >= arc || is_last_segment)) {
      const double ratio = std::clamp((arc - accumulated) / segment_length, 0.0, 1.0);
      const double cx = p0.x + ratio * (p1.x - p0.x);
      const double cy = p0.y + ratio * (p1.y - p0.y);
      const double cz = p0.z + ratio * (p1.z - p0.z);
      const double nx = -(p1.y - p0.y) / segment_length;
      const double ny = (p1.x - p0.x) / segment_length;
      return lanelet::LineString3d(
        id, {lanelet::Point3d(lanelet::InvalId, cx - nx * half_width, cy - ny * half_width, cz),
             lanelet::Point3d(lanelet::InvalId, cx + nx * half_width, cy + ny * half_width, cz)});
    }
    accumulated += segment_length;
  }
  // degenerate path: fall back to a point-sized line at the first point
  const auto & p = route_path.front().pose.position;
  return lanelet::LineString3d(
    id, {lanelet::Point3d(lanelet::InvalId, p.x, p.y, p.z),
         lanelet::Point3d(lanelet::InvalId, p.x, p.y, p.z)});
}

// Explicit stop line for a crosswalk/walkway lanelet from a RoadMarking regulatory element whose
// "crosswalk_id" attribute points back to the lanelet.
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

MapBasedStopPlanner::MapBasedStopPlanner(
  const rclcpp::Logger & logger, std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
: logger_(logger),
  time_keeper_(
    time_keeper ? std::move(time_keeper) : std::make_shared<autoware_utils_debug::TimeKeeper>())
{
}

void MapBasedStopPlanner::set_planner_data(
  const LaneletMapBin::ConstSharedPtr & lanelet_map_bin_ptr,
  const LaneletRoute::ConstSharedPtr & route_ptr, const RouteContext & route_context)
{
  lanelet_map_ptr_ = route_context.lanelet_map_ptr;
  if (lanelet_map_bin_ptr == stop_lines_map_ptr_ && route_ptr == stop_lines_route_ptr_) {
    return;
  }
  intersection_debug_lanelets_ = {};
  stop_lines_ = collect_stop_lines(route_context, &intersection_debug_lanelets_);
  stop_lines_map_ptr_ = lanelet_map_bin_ptr;
  stop_lines_route_ptr_ = route_ptr;
}

MapBasedStopPlanner::Result MapBasedStopPlanner::plan(
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & ego_pose,
  const double ego_velocity, const double ego_acceleration,
  const StopSelectionParams & params) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  Result result;
  result.go_trajectory = trajectory;

  if (trajectory.points.size() < 2) return result;

  const auto stop_lines = filter_stop_lines_on_trajectory(stop_lines_, trajectory.points);
  const auto road_shoulder_stop_arc_length =
    select_road_shoulder_stop_arc_length(trajectory.points, ego_pose, params);

  {
    // NOTE(odashima): the shoulder stop target is not a map line, so it is synthesized for
    // visualization only; it is fed to plan_single_stop as an arc length instead, because its
    // margin is measured from the footprint rather than from base_link_to_front.
    constexpr double stop_line_half_width_m = 3.0;
    constexpr lanelet::Id road_shoulder_line_id = -(INT64_C(1) << 43);
    auto marker_lines = stop_lines;
    if (road_shoulder_stop_arc_length) {
      marker_lines.push_back(
        StopLine{
          make_perpendicular_line(
            trajectory.points, *road_shoulder_stop_arc_length, stop_line_half_width_m,
            road_shoulder_line_id),
          StopLineType::RoadShoulder, false});
    }
    result.stop_line_markers = create_stop_line_marker_array(marker_lines);
  }

  // Debug visualization of the intersection lanelet groups behind the non-priority judgement.
  {
    const auto append_lanelets = [&result](
                                   const std::string & ns, const lanelet::ConstLanelets & lanelets,
                                   const std_msgs::msg::ColorRGBA & color) {
      if (lanelets.empty()) return;
      auto markers = lanelet::visualization::laneletsBoundaryAsMarkerArray(
        lanelets, color, /*viz_centerline=*/false);
      int32_t marker_id = 0;
      for (auto & marker : markers.markers) {
        marker.ns = ns;
        marker.id = marker_id++;
      }
      result.stop_line_markers.markers.insert(
        result.stop_line_markers.markers.end(), markers.markers.begin(), markers.markers.end());
    };
    append_lanelets(
      "intersection_conflicting_lanelets", intersection_debug_lanelets_.conflicting,
      make_color(1.0f, 0.5f, 0.0f, 0.99f));
    append_lanelets(
      "intersection_yield_lanelets", intersection_debug_lanelets_.yield,
      make_color(0.0f, 1.0f, 0.0f, 0.99f));
    append_lanelets(
      "intersection_ego_lanelets", intersection_debug_lanelets_.ego,
      make_color(0.0f, 0.5f, 1.0f, 0.99f));
    append_lanelets(
      "intersection_attention_lanelets", intersection_debug_lanelets_.attention,
      make_color(1.0f, 0.0f, 0.0f, 0.99f));
  }

  if (stop_lines.empty() && !road_shoulder_stop_arc_length) return result;

  // The go trajectory stops only at mandatory targets (e.g. stop lines); the stop trajectory
  // additionally stops at possibility targets (e.g. traffic lights).
  const auto go_stop = plan_single_stop(
    stop_lines, trajectory, ego_pose, ego_velocity, ego_acceleration, params,
    /*include_possibility=*/false, road_shoulder_stop_arc_length);
  const auto stop_stop = plan_single_stop(
    stop_lines, trajectory, ego_pose, ego_velocity, ego_acceleration, params,
    /*include_possibility=*/true, road_shoulder_stop_arc_length);

  if (go_stop) result.go_trajectory = go_stop->trajectory;

  const bool stop_differs_from_go =
    stop_stop &&
    (!go_stop || std::abs(stop_stop->stop_point_arc_length - go_stop->stop_point_arc_length) >
                   params.stop_point_diff_threshold);
  if (stop_differs_from_go) result.stop_trajectory = stop_stop->trajectory;

  return result;
}

std::optional<MapBasedStopPlanner::SingleStopResult> MapBasedStopPlanner::plan_single_stop(
  const std::vector<StopLine> & stop_lines, const Trajectory & trajectory,
  const geometry_msgs::msg::Pose & ego_pose, const double ego_velocity,
  const double ego_acceleration, const StopSelectionParams & params, const bool include_possibility,
  const std::optional<double> & road_shoulder_stop_arc_length) const
{
  autoware_utils_debug::ScopedTimeTrack st(
    include_possibility ? "plan_single_stop(stop)" : "plan_single_stop(go)", *time_keeper_);

  auto stop_point_arc_length = select_stop_arc_length(
    stop_lines, trajectory.points, ego_pose, ego_velocity, ego_acceleration, params,
    include_possibility);
  if (
    include_possibility && road_shoulder_stop_arc_length &&
    (!stop_point_arc_length || *road_shoulder_stop_arc_length < *stop_point_arc_length)) {
    stop_point_arc_length = road_shoulder_stop_arc_length;
  }
  if (!stop_point_arc_length) return std::nullopt;

  if (autoware::trajectory_processor::utils::stop_point_exists(
        trajectory.points, *stop_point_arc_length)) {
    return std::nullopt;
  }

  Trajectory stop_trajectory = trajectory;
  if (!autoware::trajectory_processor::utils::insert_stop_point(
        stop_trajectory.points, *stop_point_arc_length)) {
    return std::nullopt;
  }
  return SingleStopResult{*stop_point_arc_length, std::move(stop_trajectory)};
}

std::vector<StopLine> MapBasedStopPlanner::collect_stop_lines(
  const RouteContext & route_context, IntersectionDebugLanelets * debug_lanelets) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & route_lanelets = route_context.route_lanelets;
  const auto & preferred_lanelets = route_context.preferred_lanelets;

  std::vector<StopLine> stop_lines;
  // De-duplication key is (type, id): the line string id for map lines, or the source lanelet id
  // for synthesized entry edges.
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
      // Only stop lines referenced by a traffic sign (vm-02-02: stop sign) are legally mandatory
      // stops -> StopLine.
      for (const auto & traffic_sign : lanelet.regulatoryElementsAs<lanelet::TrafficSign>()) {
        for (const auto & ref_line : traffic_sign->refLines()) {
          add_line(ref_line, StopLineType::StopLine, ref_line.id());
        }
      }

      // Stop lines referenced by traffic lights -> TrafficLight.
      for (const auto & traffic_light :
           lanelet.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>()) {
        if (const auto stop_line = traffic_light->stopLine()) {
          add_line(*stop_line, StopLineType::TrafficLight, stop_line->id());
        }
      }
    }
  }

  const auto route_path = build_route_path(preferred_lanelets);
  const auto arc_ranges = lanelet_arc_ranges(preferred_lanelets);

  // Crosswalk / walkway lanelets overlapping the route lanelets, found via the map's spatial
  // index (RTree) per route lanelet plus a 2D polygon-overlap check. Walkways carry no regulatory
  // element, so a regulatory-element-only search cannot detect them.
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
        // Stop line priority: crosswalk regulatory element stop line, then a "crosswalk_id"
        // RoadMarking, and only as a fallback the entry-side bound of the crosswalk lanelet.
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

  // Intersection
  {
    autoware_utils_debug::ScopedTimeTrack st_intersection(
      "intersection_conflict_search", *time_keeper_);
    constexpr double stop_line_half_width_m = 3.0;
    constexpr lanelet::Id intersection_conflict_id_offset = INT64_C(1) << 42;
    const auto & routing_graph = route_context.routing_graph_ptr;

    const auto contains = [](const lanelet::ConstLanelets & lanelets, const lanelet::Id id) {
      return std::any_of(
        lanelets.begin(), lanelets.end(), [id](const auto & ll) { return ll.id() == id; });
    };

    for (size_t i = 0; i < preferred_lanelets.size(); ++i) {
      const auto & lanelet = preferred_lanelets[i];
      if (!lanelet.hasAttribute("turn_direction")) continue;

      // Without a routing graph the conflict topology is unknown; fall back to the entry edge.
      if (!routing_graph) {
        add_line(make_entry_edge(lanelet), StopLineType::Intersection, lanelet.id());
        continue;
      }

      // A signalized straight lane is arbitrated by the signal (vm-03-10).
      const auto turn_direction = lanelet.attributeOr("turn_direction", std::string(""));
      const bool has_traffic_light = !lanelet.regulatoryElementsAs<lanelet::TrafficLight>().empty();
      if (turn_direction == "straight" && has_traffic_light) continue;

      // Lanes yielding to ego: yield lanelets (and their upstreams) of RightOfWay regulatory
      // elements that grant ego the right of way.
      lanelet::ConstLanelets yield_lanelets;
      for (const auto & right_of_way : lanelet.regulatoryElementsAs<lanelet::RightOfWay>()) {
        if (!contains(right_of_way->rightOfWayLanelets(), lanelet.id())) continue;
        for (const auto & yield_lanelet : right_of_way->yieldLanelets()) {
          yield_lanelets.push_back(yield_lanelet);
          for (const auto & previous : routing_graph->previous(yield_lanelet)) {
            yield_lanelets.push_back(previous);
          }
        }
      }

      // Ego's own lanes: the assigned lanelet, its upstreams and their sibling branches.
      lanelet::ConstLanelets ego_lanelets{lanelet};
      for (const auto & previous : routing_graph->previous(lanelet)) {
        ego_lanelets.push_back(previous);
        for (const auto & following : routing_graph->following(previous)) {
          ego_lanelets.push_back(following);
        }
      }

      lanelet::ConstLanelets conflicting_lanelets;
      for (const auto & conflicting : routing_graph->conflicting(lanelet)) {
        if (const auto conflicting_lanelet = conflicting.lanelet()) {
          conflicting_lanelets.push_back(*conflicting_lanelet);
        }
      }

      // Priority lanes ego must yield to. Empty means ego has priority (or the turn does not
      // cross another lane): no stop line.
      lanelet::ConstLanelets attention_lanelets;
      for (const auto & conflicting_lanelet : conflicting_lanelets) {
        if (contains(yield_lanelets, conflicting_lanelet.id())) continue;
        if (contains(ego_lanelets, conflicting_lanelet.id())) continue;
        attention_lanelets.push_back(conflicting_lanelet);
      }

      if (debug_lanelets) {
        const auto append_unique =
          [&contains](lanelet::ConstLanelets & to, const lanelet::ConstLanelets & from) {
            for (const auto & ll : from) {
              if (!contains(to, ll.id())) to.push_back(ll);
            }
          };
        append_unique(debug_lanelets->conflicting, conflicting_lanelets);
        append_unique(debug_lanelets->yield, yield_lanelets);
        append_unique(debug_lanelets->ego, ego_lanelets);
        append_unique(debug_lanelets->attention, attention_lanelets);
      }

      if (attention_lanelets.empty()) continue;

      // First point (within this intersection lanelet) where the route path enters a priority
      // lane's outline.
      std::optional<double> first_conflict_arc;
      for (const auto & attention_lanelet : attention_lanelets) {
        for (const double arc : lanelet_outline_crossing_arcs(
               route_path, attention_lanelet, arc_ranges[i].first, arc_ranges[i].second)) {
          if (!first_conflict_arc || arc < *first_conflict_arc) first_conflict_arc = arc;
        }
      }
      // The route never enters a priority lane (e.g. the conflict area lies off the path).
      if (!first_conflict_arc) continue;

      add_line(
        make_perpendicular_line(
          route_path, *first_conflict_arc, stop_line_half_width_m,
          -(lanelet.id() + intersection_conflict_id_offset)),
        StopLineType::Intersection, lanelet.id() + intersection_conflict_id_offset);
    }
  }

  // Maximal runs of location=private lanelets along the preferred lane sequence -> PrivateArea.
  // The private connector lanelet spatially overlaps the public road at its ends, so its entry
  // edge lies on the roadway; instead, the stop line is placed where the route path actually
  // leaves the public road (entry) or enters it again (exit), found as crossings of the route
  // path with the outlines of non-private road lanelets overlapping the connector. The connector
  // entry edge is only a fallback when no overlapping road is found.
  {
    constexpr double stop_line_half_width_m = 3.0;
    // NOTE(odashima): keep private entry/exit line ids distinct from each other and from the -id
    // intersection edges possibly synthesized for the same lanelet (marker de-duplication by id).
    constexpr lanelet::Id private_entry_id_offset = INT64_C(1) << 40;
    constexpr lanelet::Id private_exit_id_offset = INT64_C(1) << 41;

    // Non-private road lanelets spatially overlapping the given connector lanelet.
    const auto overlapping_public_roads = [&](const lanelet::ConstLanelet & connector) {
      lanelet::ConstLanelets public_roads;
      if (!route_context.lanelet_map_ptr) return public_roads;
      const auto nearby_lanelets = route_context.lanelet_map_ptr->laneletLayer.search(
        lanelet::geometry::boundingBox2d(connector));
      for (const auto & candidate : nearby_lanelets) {
        if (candidate.id() == connector.id()) continue;
        if (
          candidate.attributeOr(lanelet::AttributeNamesString::Subtype, std::string("")) !=
          lanelet::AttributeValueString::Road)
          continue;
        if (is_private(candidate)) continue;
        if (!lanelet::geometry::overlaps2d(connector, candidate)) continue;
        public_roads.push_back(candidate);
      }
      return public_roads;
    };

    for (size_t i = 0; i < preferred_lanelets.size();) {
      if (!is_private(preferred_lanelets[i])) {
        ++i;
        continue;
      }
      const size_t run_start = i;
      while (i < preferred_lanelets.size() && is_private(preferred_lanelets[i])) ++i;
      const size_t run_end = i - 1;

      // Entry: stop where the route path leaves the public road (the farthest crossing with an
      // overlapping public road outline inside the entry connector lanelet).
      if (run_start > 0) {
        const auto & connector = preferred_lanelets[run_start];
        std::optional<double> departure_arc;
        for (const auto & road : overlapping_public_roads(connector)) {
          for (const double arc : lanelet_outline_crossing_arcs(
                 route_path, road, arc_ranges[run_start].first, arc_ranges[run_start].second)) {
            if (!departure_arc || arc > *departure_arc) departure_arc = arc;
          }
        }
        if (departure_arc) {
          add_line(
            make_perpendicular_line(
              route_path, *departure_arc, stop_line_half_width_m,
              -(connector.id() + private_entry_id_offset)),
            StopLineType::PrivateArea, connector.id() + private_entry_id_offset);
        } else {
          add_line(make_entry_edge(connector), StopLineType::PrivateArea, connector.id());
        }
      }

      // Exit: stop where the route path enters the public road again (the nearest crossing with
      // an overlapping public road outline inside the exit connector lanelet).
      if (run_end + 1 < preferred_lanelets.size()) {
        const auto & connector = preferred_lanelets[run_end];
        std::optional<double> merge_arc;
        for (const auto & road : overlapping_public_roads(connector)) {
          for (const double arc : lanelet_outline_crossing_arcs(
                 route_path, road, arc_ranges[run_end].first, arc_ranges[run_end].second)) {
            if (!merge_arc || arc < *merge_arc) merge_arc = arc;
          }
        }
        if (merge_arc) {
          add_line(
            make_perpendicular_line(
              route_path, *merge_arc, stop_line_half_width_m,
              -(connector.id() + private_exit_id_offset)),
            StopLineType::PrivateArea, connector.id() + private_exit_id_offset);
        } else {
          add_line(
            make_entry_edge(preferred_lanelets[run_end + 1]), StopLineType::PrivateArea,
            preferred_lanelets[run_end + 1].id());
        }
      }
    }
  }

  return stop_lines;
}

std::vector<StopLine> MapBasedStopPlanner::filter_stop_lines_on_trajectory(
  const std::vector<StopLine> & stop_lines,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

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

std::optional<double> MapBasedStopPlanner::select_stop_arc_length(
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

  // NOTE(odashima): reachability must be judged on the remaining distance from ego, not on arc
  // lengths from the trajectory start — the trajectory starts a backward path length behind the
  // vehicle, which would otherwise keep already-passed or unreachable stop points alive.
  const double braking_distance =
    autoware::motion_utils::calculate_stop_distance(
      ego_velocity, ego_acceleration, params.max_deceleration, params.max_jerk)
      .value_or(ego_velocity * ego_velocity / (2.0 * params.max_deceleration));
  const double ego_arc_length =
    autoware::motion_utils::calcSignedArcLength(trajectory_points, 0UL, ego_pose.position);

  std::optional<double> nearest_stop_point_arc_length;
  for (const auto & stop_line : stop_lines) {
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
      // Stop the front bumper a margin before the crossing point; crosswalks/walkways without an
      // explicit stop line and private-area boundaries keep an extra type-specific distance.
      double stop_margin = params.stop_margin_distance;
      if (stop_line.without_explicit_stop_line) {
        stop_margin += params.stop_distance_from_crosswalk;
      }
      if (stop_line.type == StopLineType::PrivateArea) {
        stop_margin += params.stop_distance_from_private_area;
      }
      if (stop_line.type == StopLineType::Intersection) {
        stop_margin += params.stop_distance_from_intersection;
      }
      const double stop_point_arc_length =
        crossing_arc_length - params.base_link_to_front - stop_margin;
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

std::optional<double> MapBasedStopPlanner::select_road_shoulder_stop_arc_length(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points,
  const geometry_msgs::msg::Pose & ego_pose, const StopSelectionParams & params) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!lanelet_map_ptr_ || trajectory_points.size() < 2) {
    return std::nullopt;
  }

  // Lanelets of the given kind whose polygon overlaps the footprint.
  const auto overlaps = [this](const lanelet::BasicPolygon2d & footprint, const bool road_lane) {
    for (const auto & lanelet : lanelet_map_ptr_->laneletLayer.search(bounding_box_2d(footprint))) {
      const bool kind_matches =
        road_lane ? autoware::experimental::lanelet2_utils::is_road_lane(lanelet)
                  : autoware::experimental::lanelet2_utils::is_shoulder_lane(lanelet);
      if (!kind_matches) continue;
      if (boost::geometry::intersects(footprint, lanelet.polygon2d().basicPolygon())) return true;
    }
    return false;
  };

  const auto ego_footprint = make_vehicle_footprint(ego_pose, params.vehicle_info);
  const bool ego_in_road_lane = overlaps(ego_footprint, /*road_lane=*/true);
  const bool ego_in_shoulder_lane = overlaps(ego_footprint, /*road_lane=*/false);
  const bool departing = !ego_in_road_lane && ego_in_shoulder_lane;
  const bool entering = ego_in_road_lane && !ego_in_shoulder_lane;
  if (!departing && !entering) {
    return std::nullopt;
  }

  const double ego_arc_length =
    autoware::motion_utils::calcSignedArcLength(trajectory_points, 0UL, ego_pose.position);

  // NOTE(odashima): the braking-distance reachability check applied to map stop lines is
  // deliberately skipped, and the stop point is clamped to ego instead: crossing the shoulder
  // boundary is what the vehicle must not do, and around a shoulder it travels slowly enough that
  // the clamp holds it in place rather than demanding an infeasible deceleration.
  const auto stop_arc_length = [&](const double crossing_arc_length) {
    return std::max(
      crossing_arc_length - params.stop_margin_distance - params.stop_distance_from_road_shoulder,
      ego_arc_length);
  };

  double arc_length = 0.0;
  double shoulder_touch_arc_length = 0.0;
  bool shoulder_touched = false;
  for (size_t i = 0; i < trajectory_points.size(); ++i) {
    if (i > 0) {
      arc_length += autoware_utils::calc_distance2d(trajectory_points[i - 1], trajectory_points[i]);
    }
    if (arc_length < ego_arc_length) continue;
    const auto footprint = make_vehicle_footprint(trajectory_points[i].pose, params.vehicle_info);
    const bool in_road_lane = overlaps(footprint, /*road_lane=*/true);
    const bool in_shoulder_lane = overlaps(footprint, /*road_lane=*/false);

    if (departing) {
      if (in_road_lane) return stop_arc_length(arc_length);
      continue;
    }

    // Entering: stop where the footprint starts to straddle the boundary, but only once the
    // trajectory is known to end up wholly inside the shoulder — merely clipping a shoulder while
    // staying in the road lane is normal driving and must not stop the vehicle.
    if (!in_shoulder_lane) {
      // The straddle that led here did not complete, so a later one anchors the stop instead.
      shoulder_touched = false;
      continue;
    }
    if (!shoulder_touched) {
      shoulder_touch_arc_length = arc_length;
      shoulder_touched = true;
    }
    if (!in_road_lane) {
      return stop_arc_length(shoulder_touch_arc_length);
    }
  }
  return std::nullopt;
}

visualization_msgs::msg::MarkerArray MapBasedStopPlanner::create_stop_line_marker_array(
  const std::vector<StopLine> & stop_lines) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  constexpr float marker_thickness_m = 0.3f;

  std::vector<lanelet::ConstLineString3d> mandatory_lines;
  std::vector<lanelet::ConstLineString3d> possibility_lines;
  for (const auto & stop_line : stop_lines) {
    if (is_possibility_type(stop_line.type)) {
      possibility_lines.push_back(stop_line.line);
    } else {
      mandatory_lines.push_back(stop_line.line);
    }
  }

  // Raise the markers above the map surface so they are not hidden by the map in a top-down
  // orthographic view.
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

  // Mandatory stop targets: red. Possibility stop targets: yellow.
  append(
    lanelet::visualization::lineStringsAsMarkerArray(
      mandatory_lines, "stop_target", make_color(1.0f, 0.0f, 0.0f, 0.8f), marker_thickness_m));
  append(
    lanelet::visualization::lineStringsAsMarkerArray(
      possibility_lines, "stop_possibility", make_color(1.0f, 1.0f, 0.0f, 0.8f),
      marker_thickness_m));

  // Type labels: view-facing text above the middle of each stop line. The height is staggered
  // per type so labels of overlapping lines of different types do not cover each other.
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
    const auto midpoint = arc_length_midpoint(stop_line.line);
    text_marker.pose.position.x = midpoint.x();
    text_marker.pose.position.y = midpoint.y();
    text_marker.pose.position.z =
      midpoint.z() + text_z_offset_m + text_z_step_per_type_m * static_cast<double>(stop_line.type);
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = text_height_m;
    text_marker.color = make_color(1.0f, 1.0f, 1.0f, 0.9f);
    text_marker.text = to_string(stop_line.type);
    marker_array.markers.push_back(text_marker);
  }

  return marker_array;
}

}  // namespace autoware::minimum_rule_based_planner
