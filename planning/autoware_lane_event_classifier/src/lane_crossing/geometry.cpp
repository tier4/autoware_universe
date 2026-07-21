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

#include <autoware/lanelet2_utils/kind.hpp>
#include <autoware_lane_event_classifier/detail/geometry_utils.hpp>
#include <autoware_lane_event_classifier/lane_crossing/geometry.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>

#include <algorithm>
#include <cmath>
#include <functional>
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

// The left / right boundary polylines of the lane sequence, each spanning the whole forward
// straight sequence.
struct LaneSequenceBounds
{
  LineString2d left;
  LineString2d right;
};

// The lane-sequence boundary spans the whole forward straight sequence, not just the single
// reference lanelet the ego sits in: the ego dodges around an object that can be many metres ahead,
// so the poke crosses the boundary of whichever sequence lane lies at the object. Concatenating the
// sequence lanes' left/right bounds gives one continuous boundary polyline per side (consecutive
// route lanes share their junction endpoints).
LaneSequenceBounds build_lane_sequence_bounds(const lanelet::ConstLanelets & lane_sequence)
{
  LaneSequenceBounds bounds;
  for (const auto & lane : lane_sequence) {
    const auto lane_left = to_line_string(lane.leftBound2d());
    const auto lane_right = to_line_string(lane.rightBound2d());
    bounds.left.insert(bounds.left.end(), lane_left.cbegin(), lane_left.cend());
    bounds.right.insert(bounds.right.end(), lane_right.cbegin(), lane_right.cend());
  }
  return bounds;
}

// Which lane-sequence boundary a point sits nearer to (the boundary the ego crosses there).
bool point_is_nearer_left_boundary(
  const lanelet::BasicPoint2d & point, const LaneSequenceBounds & bounds)
{
  const Point2d boost_point{point.x(), point.y()};
  const double distance_to_left = boost::geometry::distance(boost_point, bounds.left);
  const double distance_to_right = boost::geometry::distance(boost_point, bounds.right);
  return distance_to_left <= distance_to_right;
}

// A single crossing of the lane-sequence boundary: which side, and where.
struct CrossingCandidate
{
  bool is_to_left{false};
  lanelet::BasicPoint2d point{0.0, 0.0};
};

// ---------------------------------------------------------------------------------------------
// Source (a) - predictive: the planned trajectory centerline forms a closed departure around the
// object (crosses a boundary out before it, back in after it). Early, but blind to a shallow dodge
// where only the body crosses; source (b) covers that.
// ---------------------------------------------------------------------------------------------

// Arc length along the trajectory polyline to the projection of a query point onto it. Both the
// boundary crossings (which lie on the trajectory, so the projection is exact) and the candidate
// objects are placed in this single arc frame, so "exit before the object, re-enter after" reduces
// to comparing arc lengths.
double project_arc_length(
  const std::vector<lanelet::BasicPoint2d> & path, double query_x, double query_y)
{
  double best_arc = 0.0;
  double best_distance_sq = std::numeric_limits<double>::max();
  double cumulative = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const double ax = path[index].x();
    const double ay = path[index].y();
    const double dx = path[index + 1].x() - ax;
    const double dy = path[index + 1].y() - ay;
    const double segment_length_sq = dx * dx + dy * dy;
    double ratio = 0.0;
    if (segment_length_sq > 0.0) {
      ratio = ((query_x - ax) * dx + (query_y - ay) * dy) / segment_length_sq;
      ratio = std::clamp(ratio, 0.0, 1.0);
    }
    const double projection_x = ax + ratio * dx;
    const double projection_y = ay + ratio * dy;
    const double distance_sq = (query_x - projection_x) * (query_x - projection_x) +
                               (query_y - projection_y) * (query_y - projection_y);
    const double segment_length = std::sqrt(segment_length_sq);
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_arc = cumulative + ratio * segment_length;
    }
    cumulative += segment_length;
  }
  return best_arc;
}

// A point where the trajectory crosses a lane-sequence boundary, in the trajectory-arc frame.
struct BoundaryCrossing
{
  double arc_m{0.0};
  bool is_to_left{false};
  lanelet::BasicPoint2d point{0.0, 0.0};
};

// Every trajectory-vs-boundary crossing, ordered by arc length along the trajectory. The trajectory
// starts inside the sequence, so the crossings alternate exit, re-enter, exit, ...
std::vector<BoundaryCrossing> get_ordered_boundary_crossings(
  const std::vector<lanelet::BasicPoint2d> & trajectory_points, const LaneSequenceBounds & bounds)
{
  std::vector<BoundaryCrossing> crossings;
  if (trajectory_points.size() < 2) {
    return crossings;
  }
  const auto trajectory_line = to_line_string(trajectory_points);
  const auto collect = [&](const LineString2d & boundary, bool is_to_left) {
    std::vector<Point2d> hits;
    boost::geometry::intersection(trajectory_line, boundary, hits);
    for (const auto & hit : hits) {
      crossings.push_back(
        {project_arc_length(trajectory_points, hit.x(), hit.y()), is_to_left, {hit.x(), hit.y()}});
    }
  };
  collect(bounds.left, true);
  collect(bounds.right, false);
  std::sort(
    crossings.begin(), crossings.end(),
    [](const BoundaryCrossing & a, const BoundaryCrossing & b) { return a.arc_m < b.arc_m; });
  return crossings;
}

// A closed sideways departure: the trajectory exits the lane sequence at exit_arc and re-enters at
// reenter_arc. Only closed departures count - a path that leaves and never returns within the
// look-ahead is a lane change, not a crossing, and is dropped here.
struct DepartureInterval
{
  double exit_arc_m{0.0};
  double reenter_arc_m{0.0};
  bool is_to_left{false};
  lanelet::BasicPoint2d exit_point{0.0, 0.0};
};

std::vector<DepartureInterval> get_departure_intervals(
  const std::vector<BoundaryCrossing> & crossings)
{
  std::vector<DepartureInterval> intervals;
  for (std::size_t index = 0; index + 1 < crossings.size(); index += 2) {
    const auto & exit = crossings[index];
    const auto & reenter = crossings[index + 1];
    intervals.push_back({exit.arc_m, reenter.arc_m, exit.is_to_left, exit.point});
  }
  return intervals;
}

// The first (nearest to the ego) departure that goes around a candidate object: the object's arc
// falls strictly between the exit and the re-enter. Absent when no departure brackets any
// candidate.
std::optional<CrossingCandidate> get_trajectory_crossing(
  const std::vector<lanelet::BasicPoint2d> & trajectory_points, const LaneSequenceBounds & bounds,
  const std::vector<geometry_msgs::msg::Pose> & candidate_object_poses,
  std::size_t & departure_count)
{
  const auto crossings = get_ordered_boundary_crossings(trajectory_points, bounds);
  const auto departures = get_departure_intervals(crossings);
  departure_count = departures.size();

  std::vector<double> candidate_arcs;
  candidate_arcs.reserve(candidate_object_poses.size());
  for (const auto & pose : candidate_object_poses) {
    candidate_arcs.push_back(
      project_arc_length(trajectory_points, pose.position.x, pose.position.y));
  }
  for (const auto & departure : departures) {
    const bool brackets_candidate =
      std::any_of(candidate_arcs.cbegin(), candidate_arcs.cend(), [&departure](const double arc) {
        return arc > departure.exit_arc_m && arc < departure.reenter_arc_m;
      });
    if (brackets_candidate) {
      return CrossingCandidate{departure.is_to_left, departure.exit_point};
    }
  }
  return std::nullopt;
}

// ---------------------------------------------------------------------------------------------
// Source (b) - physical: the current ego footprint (the real body, so a yawed corner poking over
// the line counts) crosses the lane-sequence boundary into a lane outside the sequence. This fires
// the moment the body is over the boundary, regardless of where the planned centerline sits.
// ---------------------------------------------------------------------------------------------

// The footprint corner poking deepest into a neighbour lane, with how far it overshoots the
// reference boundary. Only corners inside the neighbour lane count; the overshoot is that corner's
// distance past the reference boundary. Absent when no corner is inside the neighbour.
struct NeighbourOvershoot
{
  double overshoot_m{0.0};
  std::optional<lanelet::BasicPoint2d> deepest_corner;
};

NeighbourOvershoot deepest_footprint_overshoot(
  const LaneTracker & tracker, lanelet::Id reference_lane_id, lanelet::Id neighbour_id,
  const std::vector<lanelet::BasicPoint2d> & footprint)
{
  NeighbourOvershoot deepest;
  for (const auto & corner : footprint) {
    const auto inside_neighbour = tracker.distance_to_lane(neighbour_id, corner);
    if (!inside_neighbour || *inside_neighbour > 0.0) {
      continue;  // this corner is not inside the neighbour lane
    }
    const auto overshoot = tracker.distance_to_lane(reference_lane_id, corner);
    if (overshoot && *overshoot > deepest.overshoot_m) {
      deepest.overshoot_m = *overshoot;
      deepest.deepest_corner = corner;
    }
  }
  return deepest;
}

// The footprint crossing (deepest qualifying corner across the neighbour lanes) plus a
// per-neighbour note breakdown for the diagnostic. A shoulder neighbour is exempt (that is a
// road-shoulder use, handled by the lane-following check). A corner must poke past the reference
// boundary by more than footprint_boundary_overshoot_m so a slight cornering graze does not count.
struct FootprintCrossing
{
  std::optional<CrossingCandidate> crossing;
  std::string note;
};

FootprintCrossing get_footprint_crossing(
  const LaneTracker & tracker, lanelet::Id reference_lane_id,
  const std::unordered_set<lanelet::Id> & sequence_ids,
  const std::vector<lanelet::BasicPoint2d> & footprint,
  const std::vector<lanelet::Id> & footprint_ids, const LaneSequenceBounds & bounds,
  double footprint_boundary_overshoot_m)
{
  std::optional<CrossingCandidate> best;
  double deepest_overshoot_m = footprint_boundary_overshoot_m;
  std::vector<std::string> notes;
  for (const auto neighbour_id : footprint_ids) {
    if (sequence_ids.count(neighbour_id) != 0) {
      continue;  // a lane of the sequence, not a crossing
    }
    const auto overshoot =
      deepest_footprint_overshoot(tracker, reference_lane_id, neighbour_id, footprint);
    const auto neighbour_lane = tracker.get_lanelet(neighbour_id);
    const bool is_shoulder = neighbour_lane && lanelet2_utils::is_shoulder_lane(*neighbour_lane);
    notes.push_back(
      fmt::format(
        "{}:{:.2f}m{}", neighbour_id, overshoot.overshoot_m,
        is_shoulder ? " shoulder-exempt" : ""));
    if (is_shoulder) {
      continue;
    }
    if (overshoot.deepest_corner && overshoot.overshoot_m >= deepest_overshoot_m) {
      deepest_overshoot_m = overshoot.overshoot_m;
      best = CrossingCandidate{
        point_is_nearer_left_boundary(*overshoot.deepest_corner, bounds),
        *overshoot.deepest_corner};
    }
  }
  return {best, notes.empty() ? std::string{"none"} : fmt::format("{}", fmt::join(notes, " "))};
}

// A crossing decision plus its diagnostic (nullopt crossing with a reason when there is none or an
// exemption applies).
struct ResolvedCrossing
{
  std::optional<LaneCrossingCrossing> crossing;
  std::string diagnostic;
};

// Turn a chosen crossing into the observation crossing: apply the virtual-boundary exemption on the
// crossed side and find the best-effort off-sequence target lane at the crossing point.
ResolvedCrossing resolve_crossing(
  const LaneTracker & tracker, const lanelet::ConstLanelet & reference_lane,
  const std::unordered_set<lanelet::Id> & sequence_ids, const CrossingCandidate & candidate,
  const std::string & source, const std::string & detail)
{
  const bool is_to_left = candidate.is_to_left;
  const lanelet::BasicPoint2d crossing_point = candidate.point;

  // Onset exemption (docs/lane_crossing.md, "Exemptions"): a virtual crossed boundary.
  const auto & crossed_bound =
    is_to_left ? reference_lane.leftBound() : reference_lane.rightBound();
  if (is_virtual_linestring(crossed_bound)) {
    return {
      std::nullopt,
      fmt::format("exempt: crossed {} boundary is virtual", is_to_left ? "left" : "right")};
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

  LaneCrossingCrossing crossing;
  crossing.target_lane_id = target_lane_id;
  crossing.crossing_point = crossing_point;
  crossing.is_to_left = is_to_left;
  return {
    crossing, fmt::format(
                "crossing to {} (target={} via {}; {})", is_to_left ? "left" : "right",
                target_lane_id, source, detail)};
}
}  // namespace

LaneCrossingGeometry::LaneCrossingGeometry(
  double crossing_look_ahead_m, double footprint_boundary_overshoot_m)
: crossing_look_ahead_m_{crossing_look_ahead_m},
  footprint_boundary_overshoot_m_{footprint_boundary_overshoot_m}
{
}

LaneCrossingObservation LaneCrossingGeometry::observe(
  const LaneTracker & tracker, const LaneEventInput & input,
  const std::vector<geometry_msgs::msg::Pose> & candidate_object_poses) const
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
  // trajectory samples feed the predictive crossing, the footprint lanes feed the physical
  // crossing, the return, and the full-entry escape, and the straight sequence classifies which
  // lanes count as part of the lane sequence.
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
  auto crossing_result = compute_crossing(
    tracker, *reference_lane_opt, sequence_ids, trajectory_points, input.footprint, footprint_ids,
    candidate_object_poses);
  observation.crossing = std::move(crossing_result.crossing);
  observation.crossing_diagnostic = std::move(crossing_result.diagnostic);
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

LaneCrossingGeometry::CrossingResult LaneCrossingGeometry::compute_crossing(
  const LaneTracker & tracker, const lanelet::ConstLanelet & reference_lane,
  const std::unordered_set<lanelet::Id> & sequence_ids,
  const std::vector<lanelet::BasicPoint2d> & trajectory_points,
  const std::vector<lanelet::BasicPoint2d> & footprint,
  const std::vector<lanelet::Id> & footprint_ids,
  const std::vector<geometry_msgs::msg::Pose> & candidate_object_poses) const
{
  const bool has_trajectory = trajectory_points.size() >= 2;
  const bool has_footprint = footprint.size() >= 3;
  if (!has_trajectory && !has_footprint) {
    return {std::nullopt, "no trajectory or footprint"};
  }
  const auto reference_lane_id = reference_lane.id();

  // On-route-straight condition (docs/lane_crossing.md, "Scope"): only on-route-straight driving.
  // The reference lane is a route primitive whose straight successor is also a route primitive, so
  // going straight stays on-route and a lateral move is a dodge, not a lane change. This is the
  // exact complement of the lane-change straight-on-route skip, so the two classifiers never
  // double-classify.
  if (!driving_straight_stays_on_route(tracker, reference_lane_id)) {
    return {std::nullopt, "out of scope (not straight-on-route)"};
  }

  // Onset exemption (docs/lane_crossing.md, "Exemptions"): a turn-direction / intersection
  // reference lane. Going out of lane there is turning, not dodging, so no crossing is possible.
  if (is_turn_direction_lane(reference_lane)) {
    return {std::nullopt, "exempt: reference lane is a turn/intersection lane"};
  }

  // A crossing is only meaningful when the ego has an object to go around: both sources are gated
  // on a candidate object ahead on the sequence (docs/lane_crossing.md, "Onset").
  if (candidate_object_poses.empty()) {
    return {std::nullopt, "no crossing (no candidate object to go around)"};
  }

  const auto lane_sequence =
    tracker.get_forward_route_lane_sequence(reference_lane_id, crossing_look_ahead_m_);
  const auto lane_sequence_bounds = build_lane_sequence_bounds(lane_sequence);

  // Source (a) - predictive trajectory bracket (early, centerline based).
  std::size_t departure_count = 0;
  const auto trajectory_crossing = has_trajectory ? get_trajectory_crossing(
                                                      trajectory_points, lane_sequence_bounds,
                                                      candidate_object_poses, departure_count)
                                                  : std::nullopt;

  // Source (b) - physical footprint crossing (robust, the real body over the line).
  const auto footprint_crossing =
    has_footprint ? get_footprint_crossing(
                      tracker, reference_lane_id, sequence_ids, footprint, footprint_ids,
                      lane_sequence_bounds, footprint_boundary_overshoot_m_)
                  : FootprintCrossing{std::nullopt, "none"};

  // Predictive fires earlier, so prefer it when present; otherwise the body-over-the-line signal
  // still onsets a shallow dodge the centerline never shows.
  const std::string detail = fmt::format(
    "departures={} candidates={} footprint_neighbours=[{}]", departure_count,
    candidate_object_poses.size(), footprint_crossing.note);
  if (trajectory_crossing) {
    return std::invoke([&] {
      auto resolved = resolve_crossing(
        tracker, reference_lane, sequence_ids, *trajectory_crossing, "trajectory", detail);
      return CrossingResult{std::move(resolved.crossing), std::move(resolved.diagnostic)};
    });
  }
  if (footprint_crossing.crossing) {
    return std::invoke([&] {
      auto resolved = resolve_crossing(
        tracker, reference_lane, sequence_ids, *footprint_crossing.crossing, "footprint", detail);
      return CrossingResult{std::move(resolved.crossing), std::move(resolved.diagnostic)};
    });
  }
  return {std::nullopt, fmt::format("no crossing ({})", detail)};
}

bool LaneCrossingGeometry::compute_is_footprint_inside_reference_sequence(
  const LaneTracker & tracker, const LaneEventInput & input,
  const std::unordered_set<lanelet::Id> & sequence_ids,
  const std::vector<lanelet::Id> & footprint_ids)
{
  if (input.footprint.empty()) {
    return false;
  }
  return std::any_of(footprint_ids.cbegin(), footprint_ids.cend(), [&](const lanelet::Id lane_id) {
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
  // Full entry into a lane outside the straight sequence: the move is a lane change, not a
  // crossing.
  const auto full_entry =
    std::find_if(footprint_ids.cbegin(), footprint_ids.cend(), [&](const lanelet::Id lane_id) {
      return sequence_ids.count(lane_id) == 0 &&
             tracker.is_footprint_fully_inside_lane(lane_id, input.footprint);
    });
  if (full_entry == footprint_ids.cend()) {
    return std::nullopt;
  }
  return *full_entry;
}

}  // namespace autoware::lane_event_classifier
