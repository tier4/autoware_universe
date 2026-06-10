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

#ifndef MAP_BASED_PREDICTION__PRIORITY_UTILS_HPP_
#define MAP_BASED_PREDICTION__PRIORITY_UTILS_HPP_

#include "map_based_prediction/data_structure.hpp"
#include "map_based_prediction/debug_util.hpp"

#include <autoware_perception_msgs/msg/traffic_light_group.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_routing/Forward.h>

#include <cstddef>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::map_based_prediction
{
class PathGenerator;
}  // namespace autoware::map_based_prediction

namespace autoware::map_based_prediction::priority
{
using autoware_perception_msgs::msg::TrafficLightGroup;

enum class SignalPriority {
  NOT_PRIORITIZED = 0,  ///< green / matching green arrow -> object may proceed
  FULLY_PRIORITIZED,    ///< red / non-matching arrow -> object must stop
};

struct PriorityContext
{
  SignalPriority signal_priority{SignalPriority::NOT_PRIORITIZED};
  double distance_to_stopline{std::numeric_limits<double>::infinity()};
};

enum class ConservativeManeuver {
  NONE = 0,
  STOP,  ///< red
};

struct PriorityCalibrationParams
{
  bool use_signal_priority{true};
  double stop_probability_boost{0.35};
  double go_probability_decay_on_yield{0.5};
};

struct PriorityCalibration
{
  ConservativeManeuver maneuver{ConservativeManeuver::NONE};
  double conservative_weight{0.0};
  double go_scale{1.0};
};

bool hasTrafficLight(const lanelet::ConstLanelet & lanelet);

std::optional<lanelet::ConstLineString3d> getStopLine(const lanelet::ConstLanelet & lanelet);

/// Arc length [m] along @p ref_path from its start to where it crosses @p stop_line
/// (falling back to the vertex nearest the stop-line centroid). Clamped to >= 0.
std::optional<double> arcLengthToStopLine(
  const PosePath & ref_path, const lanelet::ConstLineString3d & stop_line);

struct PathSignalInfo
{
  bool found{false};
  lanelet::ConstLanelet signal_lanelet;
  std::optional<lanelet::ConstLineString3d> stop_line;
};

/// Classify a predicted reference path by the first traffic-light-controlled
/// lanelet its @p lanelet_path enters. The stop line falls back to the lanelet's
/// entry edge when the traffic light carries no tagged stop line.
PathSignalInfo classifyPathAtTrafficLight(const lanelet::routing::LaneletPath & lanelet_path);

/// Classify how a traffic signal constrains an object on @p lanelet, matching the
/// behavior_velocity traffic_light stop/go boundary (isTrafficSignalStop).
/// No observation -> NOT_PRIORITIZED.
SignalPriority evaluateSignalPriority(
  const lanelet::ConstLanelet & lanelet, const std::optional<TrafficLightGroup> & signal);

PriorityCalibration calibratePriority(
  const PriorityContext & context, const PriorityCalibrationParams & params);

ConservativeManeuver decideConservativeManeuver(
  const PriorityContext & context, const PriorityCalibrationParams & params);

PriorityCalibration weightsForManeuver(
  ConservativeManeuver maneuver, const PriorityCalibrationParams & params);

struct PriorityPredictionParams
{
  PriorityCalibrationParams calibration;
  double stop_deceleration{2.0};
  bool suppress_go_on_conservative{true};
  bool extend_stop_path_to_stopline{false};
};

/// Adds conservative stop hypotheses based on each path's traffic-signal context,
/// mutating @p predicted_paths in place: a red signal appends a stopping path
/// clipped at the stop line and decays / drops the go hypotheses.
/// @return indices into @p predicted_paths of the added conservative paths
std::unordered_set<size_t> applyPriorityCalibration(
  const TrackedObject & object, const std::vector<PredictedRefPath> & ref_paths,
  const std::vector<int> & predicted_path_ref_index, const double time_horizon,
  const PathGenerator & path_generator,
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & traffic_signal_id_map,
  const PriorityPredictionParams & params, std::vector<PredictedPath> & predicted_paths,
  debug_util::PriorityDebugCounters & counters,
  std::vector<lanelet::ConstLineString3d> & debug_stop_lines);

}  // namespace autoware::map_based_prediction::priority

#endif  // MAP_BASED_PREDICTION__PRIORITY_UTILS_HPP_
