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
  const auto [c1, c2] = stopLineChord(stop_line, 1.0);
  for (size_t i = 1; i < path.path.size(); ++i) {
    const auto crossing_point =
      autoware_utils::intersect(path.path[i - 1].position, path.path[i].position, c1, c2);
    if (crossing_point) {
      auto crossing = path.path[i];  // keep orientation / time fields
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

PathSignalInfo classifyPathAtTrafficLight(const lanelet::routing::LaneletPath & lanelet_path)
{
  PathSignalInfo info;
  for (const auto & lanelet : lanelet_path) {
    if (!lanelet_util::hasTrafficLight(lanelet)) {
      continue;
    }
    info.found = true;
    info.signal_lanelet = lanelet;
    info.stop_line = lanelet_util::getStopLine(lanelet);
    if (!info.stop_line) {
      // No tagged stop line: fall back to the lanelet's entry edge so a stopping
      // object still has a finite target at the junction entrance.
      const auto & left = lanelet.leftBound();
      const auto & right = lanelet.rightBound();
      if (!left.empty() && !right.empty()) {
        const auto lp = left.front();
        const auto rp = right.front();
        info.stop_line = lanelet::ConstLineString3d(
          lanelet::LineString3d(
            lanelet::utils::getId(),
            {lanelet::Point3d(lanelet::utils::getId(), lp.x(), lp.y(), lp.z()),
             lanelet::Point3d(lanelet::utils::getId(), rp.x(), rp.y(), rp.z())}));
      }
    }
    break;
  }
  return info;
}

SignalPriority evaluateSignalPriority(
  const lanelet::ConstLanelet & lanelet, const std::optional<TrafficLightGroup> & signal)
{
  if (!signal) {
    return SignalPriority::NOT_PRIORITIZED;
  }

  if (!autoware::traffic_light_utils::isTrafficSignalStop(lanelet, *signal)) {
    return SignalPriority::NOT_PRIORITIZED;
  }
  return SignalPriority::FULLY_PRIORITIZED;
}

ConservativeManeuver decideConservativeManeuver(
  const PriorityContext & context, const PriorityCalibrationParams & params)
{
  const bool stop_by_signal =
    params.use_signal_priority && context.signal_priority == SignalPriority::FULLY_PRIORITIZED;

  // No distance threshold: a far object just gets a stop path capped at the horizon.
  // A non-positive distance means the object has already passed the stop line.
  const bool stop_line_ahead =
    std::isfinite(context.distance_to_stopline) && context.distance_to_stopline > 0.0;
  return (stop_by_signal && stop_line_ahead) ? ConservativeManeuver::STOP
                                             : ConservativeManeuver::NONE;
}

PriorityCalibration weightsForManeuver(
  const ConservativeManeuver maneuver, const PriorityCalibrationParams & params)
{
  PriorityCalibration calibration;
  calibration.maneuver = maneuver;
  switch (maneuver) {
    case ConservativeManeuver::STOP:
      calibration.conservative_weight = params.stop_probability_boost;
      calibration.go_scale = params.go_probability_decay_on_yield;
      break;
    case ConservativeManeuver::NONE:
    default:
      calibration.conservative_weight = 0.0;
      calibration.go_scale = 1.0;
      break;
  }
  return calibration;
}

PriorityCalibration calibratePriority(
  const PriorityContext & context, const PriorityCalibrationParams & params)
{
  return weightsForManeuver(decideConservativeManeuver(context, params), params);
}

double conservativeConfidence(const Maneuver & maneuver, const double conservative_weight)
{
  // A stopping object most likely stays in its lane: the lane-follow copy keeps
  // the full stop weight and lane-change copies are scaled down so the center
  // hypothesis is always the strongest.
  constexpr double lane_change_scale = 0.5;
  return maneuver == Maneuver::LANE_FOLLOW ? conservative_weight
                                           : conservative_weight * lane_change_scale;
}

std::unordered_set<size_t> applyPriorityCalibration(
  const TrackedObject & object, const std::vector<PredictedRefPath> & ref_paths,
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & traffic_signal_id_map,
  const PriorityPredictionParams & params, std::vector<PredictedPath> & predicted_paths,
  debug_util::PriorityDebugCounters & counters,
  std::vector<lanelet::ConstLineString3d> & debug_stop_lines)
{
  std::unordered_set<size_t> conservative_indices;
  // predicted_paths[i] corresponds to ref_paths[i] only when no path was dropped
  // (e.g. by the lateral-acceleration constraint); dropped-path objects are out
  // of scope for the priority calibration.
  if (predicted_paths.empty() || predicted_paths.size() != ref_paths.size()) {
    return conservative_indices;
  }

  counters.vehicles++;

  const size_t original_count = predicted_paths.size();
  // Conservative hypotheses are appended after the loop so the (object, path
  // index) bookkeeping stays correct when go paths are suppressed.
  std::vector<bool> suppress_go(original_count, false);
  std::vector<PredictedPath> conservative_to_add;
  bool object_has_stop = false;
  for (size_t i = 0; i < original_count; ++i) {
    const auto & ref_path = ref_paths[i];
    if (ref_path.path.size() < 2) {
      continue;
    }

    const auto info = classifyPathAtTrafficLight(ref_path.lanelet_path);

    PriorityContext context;
    if (info.found) {
      const auto signal =
        lanelet_util::getSignalForLanelet(traffic_signal_id_map, info.signal_lanelet);
      context.signal_priority = evaluateSignalPriority(info.signal_lanelet, signal);
    }
    if (context.signal_priority == SignalPriority::FULLY_PRIORITIZED) {
      counters.signal_stop++;
    }

    // ref_path[0] may sit a lanelet ahead of / behind the object, so arc lengths
    // measured from ref_path[0] are converted to object-relative by subtracting
    // s_obj -- the same convention as the path generator.
    const auto & op = object.kinematics.pose_with_covariance.pose.position;
    const auto & r0 = ref_path.path.front();
    const double h0 = tf2::getYaw(r0.orientation);
    const double s_obj =
      (op.x - r0.position.x) * std::cos(h0) + (op.y - r0.position.y) * std::sin(h0);

    if (info.stop_line) {
      if (const auto distance = arcLengthToStopLine(ref_path.path, *info.stop_line)) {
        context.distance_to_stopline = *distance - s_obj;
        counters.stopline_found++;
      }
    }

    const auto maneuver = decideConservativeManeuver(context, params.calibration);
    const auto calibration = weightsForManeuver(maneuver, params.calibration);

    if (calibration.maneuver == ConservativeManeuver::NONE) {
      continue;
    }

    // The stop hypothesis is a copy of the go path, cut at the stop-line crossing;
    // a path that ends before the line stays as-is.
    PredictedPath conservative_path = predicted_paths[i];
    if (info.stop_line) {
      clipPathAtStopLine(conservative_path, *info.stop_line);
    }
    if (conservative_path.path.size() < 2) {
      continue;
    }
    conservative_path.confidence = static_cast<float>(
      conservativeConfidence(ref_path.maneuver, calibration.conservative_weight));
    object_has_stop = true;
    if (params.suppress_go_on_conservative) {
      suppress_go[i] = true;
    }
    conservative_to_add.push_back(conservative_path);
    if (info.stop_line) {
      debug_stop_lines.push_back(*info.stop_line);
    }
    counters.conservative_added++;
  }

  if (object_has_stop) {
    const auto go_scale = static_cast<float>(params.calibration.go_probability_decay_on_yield);
    for (size_t i = 0; i < original_count; ++i) {
      if (!suppress_go[i]) {
        predicted_paths[i].confidence *= go_scale;
      }
    }
  }

  if (!conservative_to_add.empty()) {
    std::vector<PredictedPath> rebuilt;
    rebuilt.reserve(predicted_paths.size() + conservative_to_add.size());
    for (size_t i = 0; i < predicted_paths.size(); ++i) {
      if (i < original_count && suppress_go[i]) {
        continue;
      }
      rebuilt.push_back(predicted_paths[i]);
    }
    for (size_t k = 0; k < conservative_to_add.size(); ++k) {
      conservative_indices.insert(rebuilt.size());
      rebuilt.push_back(conservative_to_add[k]);
    }
    predicted_paths = std::move(rebuilt);
  }

  return conservative_indices;
}

}  // namespace autoware::map_based_prediction::priority
