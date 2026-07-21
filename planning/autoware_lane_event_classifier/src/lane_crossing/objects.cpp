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
#include <autoware_lane_event_classifier/lane_crossing/objects.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>

#include <fmt/format.h>
#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::lane_event_classifier
{

namespace
{
namespace lanelet2_utils = autoware::experimental::lanelet2_utils;
using autoware_utils_geometry::Polygon2d;
using PredictedObject = autoware_perception_msgs::msg::PredictedObject;

// A lanelet's 2D polygon as a corrected boost polygon, ready for the overlap helpers.
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

// The lane-sequence lanes as corrected boost polygons (returned by value).
std::vector<Polygon2d> to_lane_sequence_polygons(const lanelet::ConstLanelets & lane_sequence)
{
  std::vector<Polygon2d> polygons;
  polygons.reserve(lane_sequence.size());
  std::transform(
    lane_sequence.cbegin(), lane_sequence.cend(), std::back_inserter(polygons),
    [](const lanelet::ConstLanelet & lane) { return to_boost_polygon(lane); });
  return polygons;
}

// True when the object footprint touches any lane-sequence polygon (a bare boundary graze counts).
bool object_touches_lane_sequence(
  const PredictedObject & object, const std::vector<Polygon2d> & lane_sequence_polygons)
{
  const auto object_polygon = autoware_utils_geometry::to_polygon2d(object);
  return std::any_of(
    lane_sequence_polygons.cbegin(), lane_sequence_polygons.cend(),
    [&object_polygon](const auto & lane_sequence_polygon) {
      return !boost::geometry::disjoint(object_polygon, lane_sequence_polygon);
    });
}

double object_speed_mps(const PredictedObject & object)
{
  const auto & velocity = object.kinematics.initial_twist_with_covariance.twist.linear;
  return std::hypot(velocity.x, velocity.y);
}

// Arc length of the object ahead of the ego along the lane sequence (negative when behind).
double arc_distance_ahead_of_ego(
  const lanelet::ConstLanelets & lane_sequence, const PredictedObject & object,
  double ego_arc_length_m)
{
  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  return lanelet2_utils::get_arc_coordinates(lane_sequence, object_pose).length - ego_arc_length_m;
}
}  // namespace

LaneCrossingObjects::LaneCrossingObjects(double object_longitudinal_window_m)
: object_longitudinal_window_m_{object_longitudinal_window_m}
{
}

bool LaneCrossingObjects::object_is_ahead_within_window(double arc_distance_ahead_m) const
{
  return arc_distance_ahead_m > 0.0 && arc_distance_ahead_m <= object_longitudinal_window_m_;
}

LaneCrossingObjects::LaneSequenceScan LaneCrossingObjects::scan_lane_sequence_objects(
  const lanelet::ConstLanelets & lane_sequence, const std::vector<PredictedObject> & objects,
  double ego_arc_length_m) const
{
  const auto lane_sequence_polygons = to_lane_sequence_polygons(lane_sequence);

  LaneSequenceScan scan;
  double nearest_ahead_m = std::numeric_limits<double>::max();
  for (const auto & object : objects) {
    // Candidate object (docs/lane_crossing.md, "Onset"): touches the reference straight sequence
    // and lies ahead within the longitudinal window. Speed is not a qualifier - the trajectory
    // bracket decides whether the ego actually goes around it.
    if (!object_touches_lane_sequence(object, lane_sequence_polygons)) {
      continue;
    }
    ++scan.lane_sequence_object_count;

    const double arc_distance_ahead =
      arc_distance_ahead_of_ego(lane_sequence, object, ego_arc_length_m);
    if (arc_distance_ahead > 0.0 && arc_distance_ahead < nearest_ahead_m) {
      nearest_ahead_m = arc_distance_ahead;
      scan.nearest_ahead_m = arc_distance_ahead;
      scan.nearest_ahead_speed_mps = object_speed_mps(object);
    }
    if (object_is_ahead_within_window(arc_distance_ahead)) {
      scan.candidate_object_poses.push_back(object.kinematics.initial_pose_with_covariance.pose);
    }
  }
  return scan;
}

LaneCrossingObjects::Result LaneCrossingObjects::observe(
  const LaneTracker & tracker, const LaneEventInput & input) const
{
  if (!input.odometry_ptr) {
    return {{}, "no odometry"};
  }
  if (!input.objects_ptr || input.objects_ptr->objects.empty()) {
    return {{}, "no perceived objects"};
  }
  const auto reference_lane_id = tracker.reference_lane().reference_lane_id;
  const auto lane_sequence =
    tracker.get_forward_route_lane_sequence(reference_lane_id, object_longitudinal_window_m_);
  if (lane_sequence.empty()) {
    return {{}, "no forward lane sequence"};
  }

  const double ego_arc_length =
    lanelet2_utils::get_arc_coordinates(lane_sequence, input.odometry_ptr->pose.pose).length;
  LaneSequenceScan scan =
    scan_lane_sequence_objects(lane_sequence, input.objects_ptr->objects, ego_arc_length);

  if (scan.lane_sequence_object_count == 0) {
    return {
      {},
      fmt::format("{} objects, none touch the lane sequence", input.objects_ptr->objects.size())};
  }
  return {
    std::move(scan.candidate_object_poses),
    fmt::format(
      "lane_sequence_objects={} candidates={} nearest_ahead={:.1f}m (speed={:.1f}mps)",
      scan.lane_sequence_object_count, scan.candidate_object_poses.size(), scan.nearest_ahead_m,
      scan.nearest_ahead_speed_mps)};
}

}  // namespace autoware::lane_event_classifier
