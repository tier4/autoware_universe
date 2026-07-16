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

#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware/lanelet2_utils/kind.hpp>
#include <autoware_lane_event_classifier/detail/geometry_utils.hpp>
#include <autoware_lane_event_classifier/lane_change/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>

#include <algorithm>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::lane_event_classifier
{

namespace
{
namespace lanelet2_utils = autoware::experimental::lanelet2_utils;

// Onset exemption (docs/lane_change.md, "Exemptions"): a turn-direction ("left"/"right") or
// intersection lanelet is exempt from lane-change onset.
bool is_turn_direction_lane(const lanelet::ConstLanelet & lane)
{
  if (lanelet2_utils::is_intersection_lanelet(lane)) {
    return true;
  }
  const auto turn_direction = lane.attributeOr("turn_direction", std::string{});
  return turn_direction == "left" || turn_direction == "right";
}

// Ordered trajectory points from the one nearest the ego, forward, until the accumulated arc length
// reaches look_ahead_m.
std::vector<lanelet::BasicPoint2d> forward_trajectory_points(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const lanelet::BasicPoint2d & ego,
  double look_ahead_m)
{
  std::vector<lanelet::BasicPoint2d> points;
  if (trajectory.points.empty()) {
    return points;
  }
  const auto point_of = [](const auto & trajectory_point) {
    return lanelet::BasicPoint2d{
      trajectory_point.pose.position.x, trajectory_point.pose.position.y};
  };
  const auto find_nearest_index = [&]() {
    std::size_t nearest_index = 0;
    double nearest_distance_sq = std::numeric_limits<double>::max();
    for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
      const double distance_sq =
        (std::invoke(point_of, trajectory.points[index]) - ego).squaredNorm();
      if (distance_sq < nearest_distance_sq) {
        nearest_distance_sq = distance_sq;
        nearest_index = index;
      }
    }
    return nearest_index;
  };

  const std::size_t nearest_index = std::invoke(find_nearest_index);
  points.reserve(trajectory.points.size() - nearest_index);
  lanelet::BasicPoint2d previous = std::invoke(point_of, trajectory.points[nearest_index]);
  points.push_back(previous);
  double accumulated_length = 0.0;
  for (std::size_t index = nearest_index + 1; index < trajectory.points.size(); ++index) {
    const lanelet::BasicPoint2d current = std::invoke(point_of, trajectory.points[index]);
    accumulated_length += (current - previous).norm();
    points.push_back(current);
    previous = current;
    if (accumulated_length >= look_ahead_m) {
      break;
    }
  }
  return points;
}
}  // namespace

LaneChangeGeometry::LaneChangeGeometry(double crossing_look_ahead_m)
: crossing_look_ahead_m_{crossing_look_ahead_m}
{
}

LaneChangeObservation LaneChangeGeometry::observe(
  const LaneTracker & tracker, const LaneEventInput & input) const
{
  // Compute the two per-cycle intermediates once and share them across the helpers below: the
  // forward trajectory samples feed both crossing and abort observations, and the footprint lanes
  // feed both the confidence booster and the settle check.
  const auto trajectory_points = std::invoke([&]() -> std::vector<lanelet::BasicPoint2d> {
    if (!input.trajectory_ptr) {
      return {};
    }
    const auto & ego_position = input.odometry_ptr->pose.pose.position;
    return forward_trajectory_points(
      *input.trajectory_ptr, {ego_position.x, ego_position.y}, crossing_look_ahead_m_);
  });
  const auto footprint_ids = tracker.footprint_lane_ids(input.footprint);

  LaneChangeObservation observation;
  observation.crossing = compute_crossing(tracker, input, trajectory_points);
  observation.trajectory_returns_to_reference = compute_trajectory_returns_to_reference(
    tracker, observation.crossing.has_value(), trajectory_points);
  observation.is_footprint_off_route_primitives =
    is_footprint_off_route_primitives(tracker, input, footprint_ids);
  observation.is_footprint_inside_reference_lane = tracker.is_footprint_fully_inside_lane(
    tracker.reference_lane().reference_lane_id, input.footprint);
  observation.settle_lane_id = compute_settle_lane_id(tracker, input, footprint_ids);
  return observation;
}

std::optional<LaneChangeCrossing> LaneChangeGeometry::compute_crossing(
  const LaneTracker & tracker, const LaneEventInput & input,
  const std::vector<lanelet::BasicPoint2d> & trajectory_points) const
{
  if (trajectory_points.empty()) {
    return std::nullopt;
  }
  const auto reference_lane_id = tracker.reference_lane().reference_lane_id;
  const auto reference_lane_opt = tracker.get_lanelet(reference_lane_id);
  if (!reference_lane_opt || !input.route_ptr) {
    return std::nullopt;
  }
  const auto & reference_lane = *reference_lane_opt;

  // Straight-on-route skip (docs/lane_change.md, "Finding a crossing"): if the reference lane is a
  // route primitive whose straight successor is also a route primitive, going straight stays
  // on-route, so any lateral crossing targets an off-route lane.
  if (tracker.is_route_primitive(reference_lane_id)) {
    const auto next_ids = tracker.next_lane_ids(reference_lane_id);
    const bool driving_straight_stays_on_route = std::any_of(
      next_ids.cbegin(), next_ids.cend(),
      [&tracker](const lanelet::Id next_id) { return tracker.is_route_primitive(next_id); });
    if (driving_straight_stays_on_route) {
      return std::nullopt;
    }
  }

  const auto & lane_sequence =
    tracker.straight_lane_sequence_ids(reference_lane, crossing_look_ahead_m_);

  std::optional<lanelet::Id> target_lane_id;
  std::optional<lanelet::BasicPoint2d> crossing_point;
  bool heads_to_route_primitive = false;

  // Classifies the lanes containing a trajectory point: whether the point is still in the straight
  // lane sequence, and the first off-sequence lane it entered. Also flags the necessity check
  // (docs/lane_change.md, "Finding a crossing") when the trajectory reaches a route primitive
  // laterally, possibly through intermediate lanes.
  const auto classify_point_lanes = [&lane_sequence, &tracker, &heads_to_route_primitive](
                                      const std::vector<lanelet::Id> & containing_ids) {
    bool is_in_lane_sequence = false;
    std::optional<lanelet::Id> off_sequence_id;
    for (const auto id : containing_ids) {
      if (lane_sequence.count(id) != 0) {
        is_in_lane_sequence = true;
      } else {
        if (!off_sequence_id) {
          off_sequence_id = id;
        }
        if (tracker.is_route_primitive(id)) {
          heads_to_route_primitive = true;
        }
      }
    }
    return std::make_pair(is_in_lane_sequence, off_sequence_id);
  };

  for (const auto & point : trajectory_points) {
    const auto [is_in_lane_sequence, off_sequence_id] =
      std::invoke(classify_point_lanes, tracker.lanelet_ids_at(point));
    // The first trajectory point that is inside a lane but no longer in the straight lane sequence
    // marks where the trajectory crosses the reference lane's lateral boundary.
    if (!target_lane_id && !is_in_lane_sequence && off_sequence_id) {
      target_lane_id = *off_sequence_id;
      crossing_point = point;
    }
    // Once the crossing is located and the trajectory is known to reach a route primitive, the
    // remaining samples cannot change the outcome — stop querying lanes for them.
    if (target_lane_id && heads_to_route_primitive) {
      break;
    }
  }
  if (!target_lane_id || !heads_to_route_primitive) {
    return std::nullopt;
  }

  // Onset exemptions (docs/lane_change.md, "Exemptions"): turn-direction / intersection reference
  // lane, or a shoulder target lane.
  if (is_turn_direction_lane(reference_lane)) {
    return std::nullopt;
  }
  const auto target_lane_opt = tracker.get_lanelet(*target_lane_id);
  if (target_lane_opt && lanelet2_utils::is_shoulder_lane(*target_lane_opt)) {
    return std::nullopt;
  }

  // Which side of the reference lane the trajectory crosses (signed lateral offset of the crossing
  // point from the centerline; positive == left of travel direction).
  const double lateral_offset =
    lanelet::geometry::toArcCoordinates(reference_lane.centerline2d(), *crossing_point).distance;
  const bool is_to_left = lateral_offset > 0.0;

  // Onset exemption (docs/lane_change.md, "Exemptions"): the crossed boundary is a virtual
  // linestring.
  const auto & crossed_bound =
    is_to_left ? reference_lane.leftBound() : reference_lane.rightBound();
  if (is_virtual_linestring(crossed_bound)) {
    return std::nullopt;
  }

  LaneChangeCrossing crossing;
  crossing.target_lane_id = *target_lane_id;
  crossing.crossing_point = *crossing_point;
  crossing.is_to_left = is_to_left;
  return crossing;
}

bool LaneChangeGeometry::compute_trajectory_returns_to_reference(
  const LaneTracker & tracker, bool has_forward_crossing,
  const std::vector<lanelet::BasicPoint2d> & trajectory_points)
{
  // A forward crossing means the trajectory is still heading out, not back.
  if (has_forward_crossing || trajectory_points.empty()) {
    return false;
  }
  const auto reference_lane_opt = tracker.get_lanelet(tracker.reference_lane().reference_lane_id);
  if (!reference_lane_opt) {
    return false;
  }
  // The far look-ahead point is back inside the reference lane.
  return lanelet::geometry::inside(*reference_lane_opt, trajectory_points.back());
}

bool LaneChangeGeometry::is_footprint_off_route_primitives(
  const LaneTracker & tracker, const LaneEventInput & input,
  const std::vector<lanelet::Id> & footprint_ids)
{
  if (!input.route_ptr) {
    return false;
  }

  // "Off the route primitives" == no footprint corner lies inside any route-preferred primitive.
  return std::none_of(
    footprint_ids.cbegin(), footprint_ids.cend(),
    [&tracker](const lanelet::Id id) { return tracker.is_route_primitive(id); });
}

std::optional<lanelet::Id> LaneChangeGeometry::compute_settle_lane_id(
  const LaneTracker & tracker, const LaneEventInput & input,
  const std::vector<lanelet::Id> & footprint_ids)
{
  if (!input.route_ptr) {
    return std::nullopt;
  }
  const auto reference_lane_id = tracker.reference_lane().reference_lane_id;

  // A settle candidate is a route-preferred primitive other than the reference lane
  const auto settled =
    std::find_if(footprint_ids.cbegin(), footprint_ids.cend(), [&](const lanelet::Id candidate_id) {
      return candidate_id != reference_lane_id && tracker.is_route_primitive(candidate_id) &&
             tracker.is_footprint_fully_inside_lane(candidate_id, input.footprint);
    });
  if (settled == footprint_ids.cend()) {
    return std::nullopt;
  }
  return *settled;
}

}  // namespace autoware::lane_event_classifier
