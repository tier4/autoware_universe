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
#include "map_based_prediction/path_generator.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <tf2/utils.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
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
// Truncate the path at its first crossing with the stop line; if it never crosses,
// bridge the last point to the line (bounded) so the path ends exactly on it.
void clipPathAtStopLine(PredictedPath & path, const lanelet::ConstLineString3d & stop_line)
{
  if (path.path.size() < 2 || stop_line.size() < 2) {
    return;
  }

  // The u (lateral) bound is generous so a laterally-offset path still clips at
  // the longitudinal stop position.
  for (size_t i = 1; i < path.path.size(); ++i) {
    const auto & a = path.path[i - 1].position;
    const auto & b = path.path[i].position;
    const double rx = b.x - a.x;
    const double ry = b.y - a.y;
    for (size_t j = 1; j < stop_line.size(); ++j) {
      const double cx = stop_line[j - 1].x();
      const double cy = stop_line[j - 1].y();
      const double sx = stop_line[j].x() - cx;
      const double sy = stop_line[j].y() - cy;
      const double denom = rx * sy - ry * sx;
      if (std::abs(denom) < 1e-9) {
        continue;  // parallel segments
      }
      const double t = ((cx - a.x) * sy - (cy - a.y) * sx) / denom;  // along path segment
      const double u = ((cx - a.x) * ry - (cy - a.y) * rx) / denom;  // along stop-line segment
      if (t >= -1e-6 && t <= 1.0 + 1e-6 && u >= -1.0 && u <= 2.0) {
        auto crossing = path.path[i];  // keep orientation / time fields
        crossing.position.x = a.x + std::clamp(t, 0.0, 1.0) * rx;
        crossing.position.y = a.y + std::clamp(t, 0.0, 1.0) * ry;
        crossing.position.z = a.z;
        path.path.resize(i);
        path.path.push_back(crossing);
        return;
      }
    }
  }

  constexpr double max_bridge = 3.0;  // [m]
  const auto & last = path.path.back().position;
  double best_d_sq = std::numeric_limits<double>::infinity();
  geometry_msgs::msg::Point target;
  for (size_t j = 1; j < stop_line.size(); ++j) {
    const double cx = stop_line[j - 1].x();
    const double cy = stop_line[j - 1].y();
    const double dx = stop_line[j].x() - cx;
    const double dy = stop_line[j].y() - cy;
    const double len_sq = dx * dx + dy * dy;
    const double u = len_sq > 1e-12
                       ? std::clamp(((last.x - cx) * dx + (last.y - cy) * dy) / len_sq, 0.0, 1.0)
                       : 0.0;
    const double qx = cx + u * dx;
    const double qy = cy + u * dy;
    const double d_sq = (last.x - qx) * (last.x - qx) + (last.y - qy) * (last.y - qy);
    if (d_sq < best_d_sq) {
      best_d_sq = d_sq;
      target.x = qx;
      target.y = qy;
      target.z = last.z;
    }
  }
  if (std::isfinite(best_d_sq) && std::sqrt(best_d_sq) <= max_bridge) {
    auto pose = path.path.back();
    pose.position = target;
    path.path.push_back(pose);
    return;
  }
}
}  // namespace

bool hasTrafficLight(const lanelet::ConstLanelet & lanelet)
{
  return !lanelet.regulatoryElementsAs<lanelet::TrafficLight>().empty();
}

std::optional<lanelet::ConstLineString3d> getStopLine(const lanelet::ConstLanelet & lanelet)
{
  for (const auto & traffic_light : lanelet.regulatoryElementsAs<lanelet::TrafficLight>()) {
    if (const auto stop_line = traffic_light->stopLine()) {
      return *stop_line;
    }
  }
  return std::nullopt;
}

std::optional<double> arcLengthToStopLine(
  const PosePath & ref_path, const lanelet::ConstLineString3d & stop_line)
{
  if (ref_path.size() < 2 || stop_line.empty()) {
    return std::nullopt;
  }

  // Use the geometric crossing, not the nearest vertex, which drifts when the
  // stop line is oblique to / laterally offset from the path.
  double arc_length = 0.0;
  for (size_t i = 1; i < ref_path.size(); ++i) {
    const double ax = ref_path.at(i - 1).position.x;
    const double ay = ref_path.at(i - 1).position.y;
    const double rx = ref_path.at(i).position.x - ax;
    const double ry = ref_path.at(i).position.y - ay;
    const double seg_len = std::hypot(rx, ry);
    for (size_t j = 1; j < stop_line.size(); ++j) {
      const double cx = stop_line[j - 1].x();
      const double cy = stop_line[j - 1].y();
      const double sx = stop_line[j].x() - cx;
      const double sy = stop_line[j].y() - cy;
      const double denom = rx * sy - ry * sx;
      if (std::abs(denom) < 1e-9) {
        continue;  // parallel segments
      }
      // a + t*r == c + u*s ; intersection lies on both when t,u in [0,1].
      const double t = ((cx - ax) * sy - (cy - ay) * sx) / denom;
      const double u = ((cx - ax) * ry - (cy - ay) * rx) / denom;
      constexpr double eps = 1e-6;
      if (t >= -eps && t <= 1.0 + eps && u >= -eps && u <= 1.0 + eps) {
        return std::max(arc_length + std::clamp(t, 0.0, 1.0) * seg_len, 0.0);
      }
    }
    arc_length += seg_len;
  }

  // No crossing (e.g. the path ends before the stop line): nearest vertex fallback.
  double center_x = 0.0;
  double center_y = 0.0;
  for (const auto & point : stop_line) {
    center_x += point.x();
    center_y += point.y();
  }
  center_x /= static_cast<double>(stop_line.size());
  center_y /= static_cast<double>(stop_line.size());

  double cum_length = 0.0;
  double nearest_arc_length = 0.0;
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
    }
  }
  return std::max(nearest_arc_length, 0.0);
}

PathSignalInfo classifyPathAtTrafficLight(const lanelet::routing::LaneletPath & lanelet_path)
{
  PathSignalInfo info;
  for (const auto & lanelet : lanelet_path) {
    if (!hasTrafficLight(lanelet)) {
      continue;
    }
    info.found = true;
    info.signal_lanelet = lanelet;
    info.stop_line = getStopLine(lanelet);
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
  const bool stop_line_known = std::isfinite(context.distance_to_stopline);
  return (stop_by_signal && stop_line_known) ? ConservativeManeuver::STOP
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

std::unordered_set<size_t> applyPriorityCalibration(
  const TrackedObject & object, const std::vector<PredictedRefPath> & ref_paths,
  const std::vector<int> & predicted_path_ref_index, const double time_horizon,
  const PathGenerator & path_generator,
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & traffic_signal_id_map,
  const PriorityPredictionParams & params, std::vector<PredictedPath> & predicted_paths,
  debug_util::PriorityDebugCounters & counters,
  std::vector<lanelet::ConstLineString3d> & debug_stop_lines)
{
  std::unordered_set<size_t> conservative_indices;
  if (predicted_paths.empty() || ref_paths.empty()) {
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
    const int ref_idx = predicted_path_ref_index[i];
    if (ref_idx < 0 || static_cast<size_t>(ref_idx) >= ref_paths.size()) {
      continue;
    }
    const auto & ref_path = ref_paths[ref_idx];
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

    PredictedPath conservative_path = path_generator.generateStoppingPathForOnLaneVehicle(
      object, ref_path.path, time_horizon, params.stop_deceleration, context.distance_to_stopline,
      ref_path.speed_limit, params.extend_stop_path_to_stopline);
    if (conservative_path.path.empty()) {
      continue;
    }
    // Clip regardless of the extend flag: lane curvature / angled stop lines can
    // produce a small overshoot even without extend.
    if (info.stop_line) {
      clipPathAtStopLine(conservative_path, *info.stop_line);
    }
    conservative_path.confidence = static_cast<float>(calibration.conservative_weight);
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
