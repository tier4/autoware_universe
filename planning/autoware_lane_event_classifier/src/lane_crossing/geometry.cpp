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

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/kind.hpp>
#include <autoware/object_recognition_utils/matching.hpp>
#include <autoware_lane_event_classifier/detail/geometry_utils.hpp>
#include <autoware_lane_event_classifier/lane_crossing/geometry.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::lane_event_classifier
{

namespace
{
namespace lanelet2_utils = autoware::experimental::lanelet2_utils;
using autoware_utils_geometry::LineString2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

// A point range (trajectory samples or a lanelet bound) as a boost linestring for intersection.
template <class PointRange>
LineString2d to_line_string(const PointRange & points)
{
  LineString2d line_string;
  line_string.reserve(points.size());
  for (const auto & point : points) {
    line_string.emplace_back(point.x(), point.y());
  }
  return line_string;
}

// A lanelet's 2D polygon as a corrected boost polygon, ready for the overlap-area helpers.
Polygon2d to_boost_polygon(const lanelet::ConstLanelet & lane)
{
  const auto basic_polygon = lane.polygon2d().basicPolygon();
  Polygon2d polygon;
  polygon.outer().reserve(basic_polygon.size() + 1);
  for (const auto & point : basic_polygon) {
    polygon.outer().emplace_back(point.x(), point.y());
  }
  boost::geometry::correct(polygon);
  return polygon;
}
}  // namespace

LaneCrossingGeometry::LaneCrossingGeometry(
  double crossing_look_ahead_m, double footprint_boundary_overshoot_m,
  double object_static_speed_threshold_mps, double object_longitudinal_window_m,
  double object_overlap_area_threshold_m2)
: crossing_look_ahead_m_{crossing_look_ahead_m},
  footprint_boundary_overshoot_m_{footprint_boundary_overshoot_m},
  object_static_speed_threshold_mps_{object_static_speed_threshold_mps},
  object_longitudinal_window_m_{object_longitudinal_window_m},
  object_overlap_area_threshold_m2_{object_overlap_area_threshold_m2}
{
}

LaneCrossingObservation LaneCrossingGeometry::observe(
  const LaneTracker & tracker, const LaneEventInput & input) const
{
  LaneCrossingObservation observation;
  if (!input.odometry_ptr || !input.route_ptr) {
    return observation;
  }
  const auto reference_lane_id = tracker.reference_lane().reference_lane_id;
  const auto reference_lane_opt = tracker.get_lanelet(reference_lane_id);
  if (!reference_lane_opt) {
    return observation;
  }

  // Compute the per-cycle intermediates once and share them across the helpers below: the forward
  // trajectory samples feed the predictive crossing, the footprint lanes feed both the return and
  // the full-entry escape, and the straight sequence classifies which lanes count as "the corridor".
  const auto trajectory_points = std::invoke([&]() -> std::vector<lanelet::BasicPoint2d> {
    if (!input.trajectory_ptr) {
      return {};
    }
    const auto & ego_position = input.odometry_ptr->pose.pose.position;
    return forward_trajectory_points(
      *input.trajectory_ptr, {ego_position.x, ego_position.y}, crossing_look_ahead_m_);
  });
  const auto footprint_ids = tracker.footprint_lane_ids(input.footprint);
  const auto & sequence_ids =
    tracker.straight_lane_sequence_ids(*reference_lane_opt, crossing_look_ahead_m_);

  observation.is_on_route_straight = driving_straight_stays_on_route(tracker, reference_lane_id);
  observation.crossing = compute_crossing(
    tracker, *reference_lane_opt, sequence_ids, trajectory_points, input.footprint, footprint_ids,
    observation.crossing_diagnostic);
  observation.has_qualifying_object =
    compute_has_qualifying_object(tracker, input, reference_lane_id, observation.object_diagnostic);
  observation.is_footprint_inside_reference_sequence =
    compute_is_footprint_inside_reference_sequence(tracker, input, sequence_ids, footprint_ids);
  observation.full_entry_lane_id =
    compute_full_entry_lane_id(tracker, input, sequence_ids, footprint_ids);
  return observation;
}

bool LaneCrossingGeometry::driving_straight_stays_on_route(
  const LaneTracker & tracker, lanelet::Id reference_lane_id)
{
  if (!tracker.is_route_primitive(reference_lane_id)) {
    return false;
  }
  const auto next_ids = tracker.next_lane_ids(reference_lane_id);
  return std::any_of(next_ids.cbegin(), next_ids.cend(), [&tracker](const lanelet::Id next_id) {
    return tracker.is_route_primitive(next_id);
  });
}

std::optional<LaneCrossingCrossing> LaneCrossingGeometry::compute_crossing(
  const LaneTracker & tracker, const lanelet::ConstLanelet & reference_lane,
  const std::unordered_set<lanelet::Id> & sequence_ids,
  const std::vector<lanelet::BasicPoint2d> & trajectory_points,
  const std::vector<lanelet::BasicPoint2d> & footprint,
  const std::vector<lanelet::Id> & footprint_ids, std::string & diagnostic) const
{
  const bool has_trajectory = trajectory_points.size() >= 2;
  const bool has_footprint = footprint.size() >= 3;
  if (!has_trajectory && !has_footprint) {
    diagnostic = "no trajectory or footprint";
    return std::nullopt;
  }
  const auto reference_lane_id = reference_lane.id();

  // Scope gate (docs/lane_crossing.md, "Scope"): only on-route-straight driving. The reference lane
  // is a route primitive whose straight successor is also a route primitive, so going straight stays
  // on-route and a lateral move is a dodge, not a lane change. This is the exact complement of the
  // lane-change straight-on-route skip, so the two classifiers never double-classify.
  if (!driving_straight_stays_on_route(tracker, reference_lane_id)) {
    diagnostic = "out of scope (not straight-on-route)";
    return std::nullopt;
  }

  // Onset exemption (docs/lane_crossing.md, "Exemptions"): a turn-direction / intersection reference
  // lane. Going out of lane there is turning, not dodging, so no crossing is possible.
  if (is_turn_direction_lane(reference_lane)) {
    diagnostic = "exempt: reference lane is a turn/intersection lane";
    return std::nullopt;
  }

  // Onset (docs/lane_crossing.md, "Onset"): a lane crossing is a PARTIAL sideways move, so the
  // planned path stays mostly inside the (wide) origin lane and never places a sample point inside
  // the neighbour lanelet the way a committed lane change does. Detect it from two sources:
  //   1. Predictive: the forward trajectory centerline crosses a corridor boundary linestring.
  //   2. Physical: the ego footprint occupies a lane OUTSIDE the straight sequence. This is the
  //      timely signal - the tracker already reports it (footprint_lane_ids) the moment the body
  //      enters the neighbour, and it fires even when the planned centreline never leaves the lane.
  //
  // The corridor boundary spans the whole forward straight sequence, not just the single reference
  // lanelet the ego sits in: the ego dodges around a static object that can be many metres ahead, so
  // the poke crosses the boundary of whichever sequence lane lies at the object, not necessarily the
  // reference lanelet. Concatenating the sequence lanes' left/right bounds gives one continuous
  // boundary polyline per side (consecutive route lanes share their junction endpoints).
  const auto corridor_lanes =
    get_forward_route_lane_sequence(tracker, reference_lane_id, crossing_look_ahead_m_);
  LineString2d left_bound_line;
  LineString2d right_bound_line;
  for (const auto & lane : corridor_lanes) {
    const auto lane_left = to_line_string(lane.leftBound2d());
    const auto lane_right = to_line_string(lane.rightBound2d());
    left_bound_line.insert(left_bound_line.end(), lane_left.cbegin(), lane_left.cend());
    right_bound_line.insert(right_bound_line.end(), lane_right.cbegin(), lane_right.cend());
  }

  // A candidate crossing point goes into the left or right bucket depending on which corridor
  // boundary it sits nearer to (that is the boundary the ego is crossing).
  std::vector<Point2d> left_intersections;
  std::vector<Point2d> right_intersections;
  const auto assign_to_side = [&](const Point2d & point) {
    const double distance_to_left = boost::geometry::distance(point, left_bound_line);
    const double distance_to_right = boost::geometry::distance(point, right_bound_line);
    if (distance_to_left <= distance_to_right) {
      left_intersections.push_back(point);
    } else {
      right_intersections.push_back(point);
    }
  };

  // Source 1 - predictive trajectory crossing.
  int trajectory_hit_count = 0;
  if (has_trajectory) {
    const auto trajectory_line = to_line_string(trajectory_points);
    std::vector<Point2d> hits;
    boost::geometry::intersection(trajectory_line, left_bound_line, hits);
    trajectory_hit_count += static_cast<int>(hits.size());
    left_intersections.insert(left_intersections.end(), hits.begin(), hits.end());
    hits.clear();
    boost::geometry::intersection(trajectory_line, right_bound_line, hits);
    trajectory_hit_count += static_cast<int>(hits.size());
    right_intersections.insert(right_intersections.end(), hits.begin(), hits.end());
  }

  // Source 2 - physical footprint occupancy of a non-sequence neighbour lane. Guard a cornering
  // graze (docs/lane_crossing.md, "Exemptions"): a rigid body overhangs the boundary slightly on a
  // curve without the driver avoiding anything, so only count the neighbour when the body pokes past
  // the reference boundary by more than footprint_boundary_overshoot_m. A shoulder neighbour is
  // exempt (that is a road-shoulder use, handled by the lane-following check).
  std::vector<std::string> footprint_notes;  // per-neighbour breakdown for the diagnostic
  if (has_footprint) {
    for (const auto neighbour_id : footprint_ids) {
      if (sequence_ids.count(neighbour_id) != 0) {
        continue;  // a lane of the corridor, not a crossing
      }
      double neighbour_overshoot_m = 0.0;
      std::optional<Point2d> deepest_corner;
      for (const auto & corner : footprint) {
        const auto inside_neighbour = tracker.distance_to_lane(neighbour_id, corner);
        if (!inside_neighbour || *inside_neighbour > 0.0) {
          continue;  // this corner is not inside the neighbour lane
        }
        const auto overshoot = tracker.distance_to_lane(reference_lane_id, corner);
        if (overshoot && *overshoot > neighbour_overshoot_m) {
          neighbour_overshoot_m = *overshoot;
          deepest_corner = Point2d{corner.x(), corner.y()};
        }
      }
      const auto neighbour_lane = tracker.get_lanelet(neighbour_id);
      const bool is_shoulder =
        neighbour_lane && lanelet2_utils::is_shoulder_lane(*neighbour_lane);
      footprint_notes.push_back(fmt::format(
        "{}:{:.2f}m{}", neighbour_id, neighbour_overshoot_m, is_shoulder ? " shoulder-exempt" : ""));
      if (is_shoulder) {
        continue;
      }
      if (deepest_corner && neighbour_overshoot_m >= footprint_boundary_overshoot_m_) {
        assign_to_side(*deepest_corner);
      }
    }
  }

  // The crossing point is the boundary crossing nearest the ego (the outbound crossing); the
  // trajectory front (else the footprint) is the ego end of the samples.
  const lanelet::BasicPoint2d ego_point =
    has_trajectory ? trajectory_points.front() : footprint.front();
  const auto nearest_intersection =
    [&ego_point](const std::vector<Point2d> & intersections)
    -> std::optional<std::pair<lanelet::BasicPoint2d, double>> {
    std::optional<std::pair<lanelet::BasicPoint2d, double>> nearest;
    for (const auto & intersection : intersections) {
      const double distance_to_ego =
        std::hypot(intersection.x() - ego_point.x(), intersection.y() - ego_point.y());
      if (!nearest || distance_to_ego < nearest->second) {
        nearest =
          std::make_pair(lanelet::BasicPoint2d{intersection.x(), intersection.y()}, distance_to_ego);
      }
    }
    return nearest;
  };
  const auto left_crossing = nearest_intersection(left_intersections);
  const auto right_crossing = nearest_intersection(right_intersections);
  const auto footprint_summary =
    footprint_notes.empty() ? std::string{"none"} : fmt::format("{}", fmt::join(footprint_notes, " "));
  if (!left_crossing && !right_crossing) {
    diagnostic = fmt::format(
      "no crossing (traj_hits={} footprint_neighbours=[{}] overshoot_min={:.2f}m)",
      trajectory_hit_count, footprint_summary, footprint_boundary_overshoot_m_);
    return std::nullopt;
  }

  // Side: whichever boundary the crossing reaches first from the ego.
  const bool is_to_left = (left_crossing && right_crossing)
                            ? left_crossing->second <= right_crossing->second
                            : static_cast<bool>(left_crossing);
  const lanelet::BasicPoint2d crossing_point =
    is_to_left ? left_crossing->first : right_crossing->first;

  // Onset exemption (docs/lane_crossing.md, "Exemptions"): a virtual crossed boundary.
  const auto & crossed_bound = is_to_left ? reference_lane.leftBound() : reference_lane.rightBound();
  if (is_virtual_linestring(crossed_bound)) {
    diagnostic = fmt::format("exempt: crossed {} boundary is virtual", is_to_left ? "left" : "right");
    return std::nullopt;
  }

  // Best-effort target lane: a lanelet sharing the crossed boundary at the crossing point that is
  // not part of the reference straight sequence. It may stay InvalId when no neighbour lane is
  // mapped (a dodge over the line into open space is still a crossing); the classifier keys
  // persistence on the side and the crossing point, not on this id.
  lanelet::Id target_lane_id = lanelet::InvalId;
  for (const auto id : tracker.lanelet_ids_at(crossing_point)) {
    if (sequence_ids.count(id) != 0) {
      continue;
    }
    target_lane_id = id;
    break;
  }

  diagnostic = fmt::format(
    "crossing to {} (target={} traj_hits={} footprint_neighbours=[{}])",
    is_to_left ? "left" : "right", target_lane_id, trajectory_hit_count, footprint_summary);
  LaneCrossingCrossing crossing;
  crossing.target_lane_id = target_lane_id;
  crossing.crossing_point = crossing_point;
  crossing.is_to_left = is_to_left;
  return crossing;
}

lanelet::ConstLanelets LaneCrossingGeometry::get_forward_route_lane_sequence(
  const LaneTracker & tracker, lanelet::Id reference_lane_id, double downstream_length_m)
{
  lanelet::ConstLanelets sequence;
  const auto reference_lane_opt = tracker.get_lanelet(reference_lane_id);
  if (!reference_lane_opt) {
    return sequence;
  }
  sequence.push_back(*reference_lane_opt);
  std::unordered_set<lanelet::Id> visited_ids{reference_lane_id};
  lanelet::Id current_id = reference_lane_id;

  // Walk the on-route straight continuation, adding a full window of downstream length beyond the
  // reference lane: the ego can be anywhere along the reference lane, so covering that extra window
  // guarantees an object (or a dodge crossing) up to downstream_length_m ahead of the ego is in the
  // sequence. Under the onset scope gate a route-primitive successor exists at each straight step.
  double downstream_length = 0.0;
  while (downstream_length < downstream_length_m) {
    const auto next_ids = tracker.next_lane_ids(current_id);
    const auto next_on_route = std::find_if(
      next_ids.cbegin(), next_ids.cend(), [&](const lanelet::Id next_id) {
        return visited_ids.count(next_id) == 0 && tracker.is_route_primitive(next_id);
      });
    if (next_on_route == next_ids.cend()) {
      break;
    }
    const auto next_lane_opt = tracker.get_lanelet(*next_on_route);
    if (!next_lane_opt) {
      break;
    }
    sequence.push_back(*next_lane_opt);
    downstream_length += lanelet::geometry::length2d(*next_lane_opt);
    visited_ids.insert(*next_on_route);
    current_id = *next_on_route;
  }
  return sequence;
}

bool LaneCrossingGeometry::compute_has_qualifying_object(
  const LaneTracker & tracker, const LaneEventInput & input, lanelet::Id reference_lane_id,
  std::string & diagnostic) const
{
  if (!input.objects_ptr || input.objects_ptr->objects.empty()) {
    diagnostic = "no perceived objects";
    return false;
  }
  const auto forward_lanes =
    get_forward_route_lane_sequence(tracker, reference_lane_id, object_longitudinal_window_m_);
  if (forward_lanes.empty()) {
    diagnostic = "no forward corridor";
    return false;
  }

  std::vector<Polygon2d> forward_lane_polygons;
  forward_lane_polygons.reserve(forward_lanes.size());
  std::transform(
    forward_lanes.cbegin(), forward_lanes.cend(), std::back_inserter(forward_lane_polygons),
    [](const lanelet::ConstLanelet & lane) { return to_boost_polygon(lane); });

  const double ego_arc_length =
    lanelet2_utils::get_arc_coordinates(forward_lanes, input.odometry_ptr->pose.pose).length;

  // Scan every object against the corridor and keep counts + the best candidate for the diagnostic,
  // so the log can explain why a crossing did or did not have a qualifying object this cycle.
  int in_corridor_count = 0;   // overlaps the corridor above the area threshold (any speed/position)
  int qualifying_count = 0;    // static AND ahead within the window AND overlapping above threshold
  double nearest_ahead_m = std::numeric_limits<double>::max();
  double nearest_ahead_overlap_m2 = 0.0;
  double nearest_ahead_speed_mps = 0.0;
  for (const auto & object : input.objects_ptr->objects) {
    // Qualifying object (docs/lane_crossing.md, "Qualifying object"): static, ahead within the
    // longitudinal window, and overlapping the reference straight sequence above the area threshold.
    const auto object_polygon = autoware_utils_geometry::to_polygon2d(object);
    double overlap_area = 0.0;
    for (const auto & lane_polygon : forward_lane_polygons) {
      overlap_area +=
        autoware::object_recognition_utils::getIntersectionArea(object_polygon, lane_polygon);
    }
    if (overlap_area < object_overlap_area_threshold_m2_) {
      continue;
    }
    ++in_corridor_count;

    const auto & velocity = object.kinematics.initial_twist_with_covariance.twist.linear;
    const double object_speed_mps = std::hypot(velocity.x, velocity.y);
    const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
    const double distance_ahead =
      lanelet2_utils::get_arc_coordinates(forward_lanes, object_pose).length - ego_arc_length;
    if (distance_ahead > 0.0 && distance_ahead < nearest_ahead_m) {
      nearest_ahead_m = distance_ahead;
      nearest_ahead_overlap_m2 = overlap_area;
      nearest_ahead_speed_mps = object_speed_mps;
    }
    const bool object_is_ahead_of_ego =
      distance_ahead > 0.0 && distance_ahead <= object_longitudinal_window_m_;
    if (object_is_ahead_of_ego && object_speed_mps < object_static_speed_threshold_mps_) {
      ++qualifying_count;
    }
  }

  if (in_corridor_count == 0) {
    diagnostic = fmt::format("{} objects, none overlap corridor", input.objects_ptr->objects.size());
  } else {
    diagnostic = fmt::format(
      "corridor_objects={} qualifying={} nearest_ahead={:.1f}m (overlap={:.1f}m2 speed={:.1f}mps)",
      in_corridor_count, qualifying_count,
      nearest_ahead_m == std::numeric_limits<double>::max() ? -1.0 : nearest_ahead_m,
      nearest_ahead_overlap_m2, nearest_ahead_speed_mps);
  }
  return qualifying_count > 0;
}

bool LaneCrossingGeometry::compute_is_footprint_inside_reference_sequence(
  const LaneTracker & tracker, const LaneEventInput & input,
  const std::unordered_set<lanelet::Id> & sequence_ids,
  const std::vector<lanelet::Id> & footprint_ids)
{
  if (input.footprint.empty()) {
    return false;
  }
  return std::any_of(
    footprint_ids.cbegin(), footprint_ids.cend(), [&](const lanelet::Id lane_id) {
      return sequence_ids.count(lane_id) != 0 &&
             tracker.is_footprint_fully_inside_lane(lane_id, input.footprint);
    });
}

std::optional<lanelet::Id> LaneCrossingGeometry::compute_full_entry_lane_id(
  const LaneTracker & tracker, const LaneEventInput & input,
  const std::unordered_set<lanelet::Id> & sequence_ids,
  const std::vector<lanelet::Id> & footprint_ids)
{
  if (input.footprint.empty()) {
    return std::nullopt;
  }
  // Full entry into a lane outside the straight sequence: the move is a lane change, not a crossing.
  const auto full_entry = std::find_if(
    footprint_ids.cbegin(), footprint_ids.cend(), [&](const lanelet::Id lane_id) {
      return sequence_ids.count(lane_id) == 0 &&
             tracker.is_footprint_fully_inside_lane(lane_id, input.footprint);
    });
  if (full_entry == footprint_ids.cend()) {
    return std::nullopt;
  }
  return *full_entry;
}

}  // namespace autoware::lane_event_classifier
