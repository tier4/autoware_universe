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

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/utility/Utilities.h>
#include <lanelet2_routing/LaneletPath.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction::priority
{
namespace
{
using TrafficLightElement = autoware_perception_msgs::msg::TrafficLightElement;
}  // namespace

std::string getTurnDirection(const lanelet::ConstLanelet & lanelet)
{
  return lanelet.attributeOr("turn_direction", "straight");
}

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

std::optional<double> distanceToLeadObject(
  const PosePath & ref_path, const std::string & ego_id, const double ego_length,
  const std::vector<LaneObject> & objects, const double lateral_threshold, const double gap_margin,
  const double ego_arc)
{
  if (ref_path.size() < 2) {
    return std::nullopt;
  }

  std::vector<double> cum_length(ref_path.size(), 0.0);
  for (size_t i = 1; i < ref_path.size(); ++i) {
    cum_length[i] =
      cum_length[i - 1] + autoware_utils::calc_distance2d(ref_path[i - 1], ref_path[i]);
  }

  // Projects a map-frame point onto the polyline -> {arc_length, lateral}.
  const auto project = [&](const double px, const double py) {
    double best_lateral_sq = std::numeric_limits<double>::infinity();
    double best_arc = 0.0;
    for (size_t i = 1; i < ref_path.size(); ++i) {
      const double ax = ref_path[i - 1].position.x;
      const double ay = ref_path[i - 1].position.y;
      const double bx = ref_path[i].position.x;
      const double by = ref_path[i].position.y;
      const double dx = bx - ax;
      const double dy = by - ay;
      const double seg_sq = dx * dx + dy * dy;
      double t = 0.0;
      if (seg_sq > 1e-9) {
        t = ((px - ax) * dx + (py - ay) * dy) / seg_sq;
        t = std::clamp(t, 0.0, 1.0);
      }
      const double fx = ax + t * dx;
      const double fy = ay + t * dy;
      const double lat_sq = (px - fx) * (px - fx) + (py - fy) * (py - fy);
      if (lat_sq < best_lateral_sq) {
        best_lateral_sq = lat_sq;
        best_arc = cum_length[i - 1] + t * std::sqrt(seg_sq);
      }
    }
    return std::make_pair(best_arc, std::sqrt(best_lateral_sq));
  };

  // "Ahead" is measured from ego_arc, not ref_path[0]: the reference path may
  // start a lanelet behind the ego, and objects in that span are not leaders.
  const double min_ahead = ego_arc + std::max(ego_length * 0.5, 0.5);
  double nearest_arc = std::numeric_limits<double>::infinity();
  double nearest_length = 0.0;
  bool found = false;
  for (const auto & obj : objects) {
    if (obj.id == ego_id) {
      continue;
    }
    const auto [arc, lateral] = project(obj.pose.position.x, obj.pose.position.y);
    if (lateral > lateral_threshold || arc < min_ahead) {
      continue;
    }
    if (arc < nearest_arc) {
      nearest_arc = arc;
      nearest_length = obj.length;
      found = true;
    }
  }
  if (!found) {
    return std::nullopt;
  }

  const double target = nearest_arc - nearest_length * 0.5 - ego_length * 0.5 - gap_margin;
  return std::max(target, 0.0);
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
    info.turn_direction = getTurnDirection(lanelet);
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

  // A solid amber circle without a red is cautionary -> creep; anything else -> stop.
  const bool has_amber = autoware::traffic_light_utils::hasTrafficLightShapeAndColor(
    signal->elements, TrafficLightElement::CIRCLE, TrafficLightElement::AMBER);
  const bool has_red = autoware::traffic_light_utils::hasTrafficLightShapeAndColor(
    signal->elements, TrafficLightElement::CIRCLE, TrafficLightElement::RED);
  if (has_amber && !has_red) {
    return SignalPriority::PARTIALLY_PRIORITIZED;
  }
  return SignalPriority::FULLY_PRIORITIZED;
}

double judgeLineDistWithJerkLimit(
  const double velocity, const double acceleration, const double max_stop_acceleration,
  const double max_stop_jerk, const double delay_response_time)
{
  // Ported verbatim from planning_utils::calcJudgeLineDistWithJerkLimit; keep in sync.
  if (velocity <= 0.0) {
    return 0.0;
  }

  // t0: observe the signal and decide to stop / t1: braking starts (jerk-limited)
  // t2: reach max deceleration / t3: stop.
  const double t1 = delay_response_time;
  const double x1 = velocity * t1;

  const double v2 = velocity + (std::pow(max_stop_acceleration, 2) - std::pow(acceleration, 2)) /
                                 (2.0 * max_stop_jerk);

  if (v2 <= 0.0) {
    const double t2 = -1.0 *
                      (max_stop_acceleration +
                       std::sqrt(acceleration * acceleration - 2.0 * max_stop_jerk * velocity)) /
                      max_stop_jerk;
    const double x2 =
      velocity * t2 + acceleration * std::pow(t2, 2) / 2.0 + max_stop_jerk * std::pow(t2, 3) / 6.0;
    return std::max(0.0, x1 + x2);
  }

  const double t2 = (max_stop_acceleration - acceleration) / max_stop_jerk;
  const double x2 =
    velocity * t2 + acceleration * std::pow(t2, 2) / 2.0 + max_stop_jerk * std::pow(t2, 3) / 6.0;

  const double x3 = -1.0 * std::pow(v2, 2) / (2.0 * max_stop_acceleration);
  return std::max(0.0, x1 + x2 + x3);
}

YellowOutcome judgeYellow(
  const double velocity, const double acceleration, const double distance_to_stopline,
  const YellowJudgeParams & params)
{
  // Mirrors TrafficLightModule::isPassthrough, but surfaces the dilemma zone.
  const double stop_distance = judgeLineDistWithJerkLimit(
    velocity, acceleration, params.max_stop_acceleration, params.max_stop_jerk,
    params.delay_response_time);
  const bool stoppable =
    (stop_distance < distance_to_stopline) || (velocity < params.stop_velocity);
  if (stoppable) {
    return YellowOutcome::STOP;
  }
  const bool reachable = distance_to_stopline < velocity * params.lamp_period;
  return reachable ? YellowOutcome::PASS : YellowOutcome::DILEMMA;
}

ConservativeManeuver decideConservativeManeuver(
  const PriorityContext & context, const PriorityCalibrationParams & params)
{
  const bool stop_by_signal =
    params.use_signal_priority && context.signal_priority == SignalPriority::FULLY_PRIORITIZED;
  const bool creep_by_signal =
    params.use_signal_priority && context.signal_priority == SignalPriority::PARTIALLY_PRIORITIZED;

  ConservativeManeuver maneuver = ConservativeManeuver::NONE;
  if (stop_by_signal) {
    maneuver = ConservativeManeuver::STOP;
  } else if (creep_by_signal) {
    maneuver = ConservativeManeuver::CREEP;
  }

  // No distance threshold: a far object just gets a stop path capped at the horizon.
  const bool stop_line_known = std::isfinite(context.distance_to_stopline);
  if (!stop_line_known) {
    maneuver = ConservativeManeuver::NONE;
  }
  return maneuver;
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
    case ConservativeManeuver::CREEP:
      // Amber: the object may still proceed, so the go hypotheses are kept intact.
      calibration.conservative_weight = params.creep_probability_boost;
      calibration.go_scale = 1.0;
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

ConservativeManeuver updateHysteresis(
  HysteresisState & state, const ConservativeManeuver raw, const double now,
  const double hysteresis_time)
{
  if (raw == state.held) {
    state.candidate = state.held;
    return state.held;
  }
  if (raw != state.candidate) {
    state.candidate = raw;
    state.candidate_since = now;
  }
  if (now - state.candidate_since >= hysteresis_time) {
    state.held = raw;
  }
  return state.held;
}

}  // namespace autoware::map_based_prediction::priority
