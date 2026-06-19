// Copyright 2026 TIER IV, inc.
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

#include "map_based_prediction/priority_utils.hpp"

#include "map_based_prediction/lanelet_util.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <tf2/utils.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/utility/Utilities.h>
#include <lanelet2_routing/LaneletPath.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction::priority
{
namespace
{
// Map stop lines are straight: treat one as the segment between its endpoints,
// extended sideways by margin * length on both ends.
std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> stopLineChord(
  const lanelet::ConstLineString3d & stop_line, const double margin)
{
  const double sx = stop_line.back().x() - stop_line.front().x();
  const double sy = stop_line.back().y() - stop_line.front().y();
  geometry_msgs::msg::Point c1;
  c1.x = stop_line.front().x() - margin * sx;
  c1.y = stop_line.front().y() - margin * sy;
  c1.z = stop_line.front().z();
  geometry_msgs::msg::Point c2;
  c2.x = stop_line.back().x() + margin * sx;
  c2.y = stop_line.back().y() + margin * sy;
  c2.z = stop_line.back().z();
  return {c1, c2};
}

// Truncate the path at its first crossing with the stop line; a path that never
// crosses the line is left untouched.
void clipPathAtStopLine(PredictedPath & path, const lanelet::ConstLineString3d & stop_line)
{
  if (path.path.size() < 2 || stop_line.size() < 2) {
    return;
  }

  // The chord is extended sideways so a laterally-offset path still clips at
  // the longitudinal stop position.
  const auto [c1, c2] = stopLineChord(stop_line, 0.0);
  for (size_t i = 1; i < path.path.size(); ++i) {
    const auto crossing_point =
      autoware_utils::intersect(path.path.at(i - 1).position, path.path.at(i).position, c1, c2);
    if (crossing_point) {
      auto crossing = path.path.at(i);  // keep orientation / time fields
      crossing.position = *crossing_point;
      path.path.resize(i);
      path.path.push_back(crossing);
      return;
    }
  }
}
}  // namespace

std::optional<double> arcLengthToStopLine(
  const PosePath & ref_path, const lanelet::ConstLineString3d & stop_line)
{
  if (ref_path.size() < 2 || stop_line.size() < 2) {
    return std::nullopt;
  }

  // Use the geometric crossing, not the nearest vertex, which drifts when the
  // stop line is oblique to / laterally offset from the path.
  const auto [c1, c2] = stopLineChord(stop_line, 0.0);
  double arc_length = 0.0;
  for (size_t i = 1; i < ref_path.size(); ++i) {
    const auto & a = ref_path.at(i - 1).position;
    const auto & b = ref_path.at(i).position;
    if (const auto crossing = autoware_utils::intersect(a, b, c1, c2)) {
      return std::max(arc_length + std::hypot(crossing->x - a.x, crossing->y - a.y), 0.0);
    }
    arc_length += std::hypot(b.x - a.x, b.y - a.y);
  }

  // No crossing: the only trusted case is the path ending just before the stop
  // line, i.e. the stop line is nearest to the path END. A nearest vertex at the
  // start or middle means the stop line lies behind / beside this path (e.g. a
  // branch candidate starting ahead of the object), where no on-path distance
  // exists -- returning a clamped value here used to fabricate a positive
  // distance for objects already past the line.
  const double center_x = 0.5 * (c1.x + c2.x);
  const double center_y = 0.5 * (c1.y + c2.y);

  double cum_length = 0.0;
  double nearest_arc_length = 0.0;
  size_t nearest_index = 0;
  double nearest_distance_sq = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < ref_path.size(); ++i) {
    if (i > 0) {
      cum_length += autoware_utils::calc_distance2d(ref_path.at(i - 1), ref_path.at(i));
    }
    const double dx = ref_path.at(i).position.x - center_x;
    const double dy = ref_path.at(i).position.y - center_y;
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < nearest_distance_sq) {
      nearest_distance_sq = distance_sq;
      nearest_arc_length = cum_length;
      nearest_index = i;
    }
  }
  if (nearest_index + 1 < ref_path.size()) {
    return std::nullopt;
  }
  return std::max(nearest_arc_length, 0.0);
}

bool hasStopLineAhead(
  const geometry_msgs::msg::Point & position, const PosePath & ref_path,
  const lanelet::ConstLineString3d & stop_line)
{
  const auto arc_length = arcLengthToStopLine(ref_path, stop_line);
  if (!arc_length) {
    return false;
  }
  // ref_path[0] may sit a lanelet ahead of / behind the object, so the arc length
  // measured from ref_path[0] is compared against the object's signed offset
  // along the path.
  const auto & r0 = ref_path.front();
  const double h0 = tf2::getYaw(r0.orientation);
  const double s_obj =
    (position.x - r0.position.x) * std::cos(h0) + (position.y - r0.position.y) * std::sin(h0);
  return *arc_length > s_obj;
}

bool findTrafficLightLaneletOnPath(
  const lanelet::routing::LaneletPath & lanelet_path, lanelet::ConstLanelet & signal_lanelet)
{
  for (const auto & lanelet : lanelet_path) {
    if (lanelet_util::hasTrafficLight(lanelet)) {
      signal_lanelet = lanelet;
      return true;
    }
  }
  return false;
}

bool evaluateSignalStopRequirement(
  const lanelet::ConstLanelet & lanelet, const std::optional<TrafficLightGroup> & signal)
{
  if (!signal) {
    return false;
  }
  return autoware::traffic_light_utils::isTrafficSignalStop(lanelet, *signal);
}

bool shouldAddStopHypothesis(
  const bool signal_requires_stop, const bool has_stop_line_ahead,
  const PriorityCalibrationParams & params)
{
  return params.use_signal_priority && signal_requires_stop && has_stop_line_ahead;
}

double weakenConfidenceInLaneChange(const Maneuver & maneuver, const double stop_weight)
{
  // A stopping object most likely stays in its lane: the lane-follow copy keeps
  // the full stop weight and lane-change copies are penalized so the center
  // hypothesis is always the strongest.
  constexpr double lane_change_penalty = 0.5;
  return maneuver == Maneuver::LANE_FOLLOW ? stop_weight : stop_weight * lane_change_penalty;
}

std::vector<PredictedPath> addTrafficSignalStopHypotheses(
  const ObjectPrediction & prediction,
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & traffic_signal_id_map,
  const PriorityPredictionParams & params, debug_util::StopHypothesisDebug & debug)
{
  const TrackedObject & object = prediction.object;
  const std::vector<PredictedRefPath> & ref_paths = prediction.ref_paths;
  const std::vector<lanelet::routing::LaneletPath> & lanelet_paths = prediction.lanelet_paths;
  const std::vector<PredictedPath> & predicted_paths = prediction.predicted_paths;

  // predicted_paths[i] corresponds to ref_paths[i] only when no path was dropped
  // (e.g. by the lateral-acceleration constraint); dropped-path objects are out
  // of scope for the priority calibration and returned unchanged.
  if (
    predicted_paths.empty() || predicted_paths.size() != ref_paths.size() ||
    lanelet_paths.size() != ref_paths.size()) {
    return predicted_paths;
  }

  debug.counter.vehicles++;

  // Start from a copy of the go paths and replace only the entries that get a
  // stop hypothesis; every other path is left untouched in place.
  std::vector<PredictedPath> result = predicted_paths;

  for (size_t i = 0; i < predicted_paths.size(); ++i) {
    // predicted_path is a mutable copy: it is clipped in place and written back
    // to result[i] when a stop hypothesis replaces the go path.
    PredictedPath predicted_path = predicted_paths.at(i);
    const PredictedRefPath & ref_path = ref_paths.at(i);
    const lanelet::routing::LaneletPath & lanelet_path = lanelet_paths.at(i);

    if (ref_path.path.size() < 2 || predicted_path.path.size() < 2) {
      continue;
    }

    // 1. Get signal status , line info for target path.
    lanelet::ConstLanelet target_lanelet_signal_object;
    if (!findTrafficLightLaneletOnPath(lanelet_path, target_lanelet_signal_object)) {
      continue;
    }
    const std::optional<TrafficLightGroup> signal_status =
      lanelet_util::getSignalForLanelet(traffic_signal_id_map, target_lanelet_signal_object);
    const std::optional<lanelet::ConstLineString3d> related_stop_line =
      lanelet_util::getStopLineOrEntryEdge(target_lanelet_signal_object);

    // 2. Signal state and whether the stop line is still ahead of the object.
    const bool signal_requires_stop =
      evaluateSignalStopRequirement(target_lanelet_signal_object, signal_status);
    const bool stop_line_ahead =
      related_stop_line &&
      hasStopLineAhead(
        object.kinematics.pose_with_covariance.pose.position, ref_path.path, *related_stop_line);
    debug.counter.signal_stop += signal_requires_stop ? 1 : 0;
    debug.counter.stopline_found += stop_line_ahead ? 1 : 0;

    // 3. Add a stop hypothesis only on a red signal whose stop line is still ahead.
    if (!shouldAddStopHypothesis(signal_requires_stop, stop_line_ahead, params.calibration)) {
      continue;
    }

    // 4. The stop hypothesis is a copy of the go path cut at the stop line.
    clipPathAtStopLine(predicted_path, *related_stop_line);

    if (predicted_path.path.size() < 2) {
      continue;  // nothing to clip to -> the go path stands
    }

    predicted_path.confidence = static_cast<float>(
      weakenConfidenceInLaneChange(ref_path.maneuver, params.calibration.stop_probability_boost));

    // The stop hypothesis replaces the go path in place (the go path is dropped).
    result.at(i) = predicted_path;

    debug.stop_hypothesis_path_indices[autoware_utils::to_hex_string(object.object_id)].insert(i);
    debug.stop_lines.push_back(*related_stop_line);
    debug.counter.stop_hypothesis_added++;
  }

  return result;
}

}  // namespace autoware::map_based_prediction::priority
