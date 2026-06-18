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

#include <autoware_perception_msgs/msg/traffic_light_group.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_routing/Forward.h>

#include <limits>
#include <optional>
#include <string>

namespace autoware::map_based_prediction::priority
{
using autoware_perception_msgs::msg::TrafficLightGroup;

enum class SignalPriority {
  NOT_PRIORITIZED = 0,    ///< green / matching green arrow -> object may proceed
  PARTIALLY_PRIORITIZED,  ///< amber -> object should creep / be cautious
  FULLY_PRIORITIZED,      ///< red / non-matching arrow -> object must stop
};

struct PriorityContext
{
  SignalPriority signal_priority{SignalPriority::NOT_PRIORITIZED};
  bool has_traffic_light{false};
  double distance_to_stopline{std::numeric_limits<double>::infinity()};
};

// Both CREEP and STOP emit a stop-at-the-line path; CREEP is kept distinct so
// amber-driven stops can be weighted and coloured separately from red.
enum class ConservativeManeuver {
  NONE = 0,
  CREEP,  ///< amber
  STOP,   ///< red
};

struct PriorityCalibrationParams
{
  bool use_signal_priority{true};
  double stop_probability_boost{0.35};
  double creep_probability_boost{0.25};
  double go_probability_decay_on_yield{0.5};
};

struct PriorityCalibration
{
  ConservativeManeuver maneuver{ConservativeManeuver::NONE};
  double conservative_weight{0.0};
  double go_scale{1.0};
};

struct HysteresisState
{
  ConservativeManeuver held{ConservativeManeuver::NONE};
  ConservativeManeuver candidate{ConservativeManeuver::NONE};
  double candidate_since{0.0};
};

/// "turn_direction" attribute of a lanelet ("straight" if absent).
std::string getTurnDirection(const lanelet::ConstLanelet & lanelet);

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
  std::string turn_direction{"straight"};
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

/// Distance [m] needed to stop under deceleration and jerk limits after a response
/// delay. Ported verbatim from
/// autoware::behavior_velocity_planner::planning_utils::calcJudgeLineDistWithJerkLimit
/// (autoware_behavior_velocity_planner_common) to avoid a perception -> planning
/// dependency.
double judgeLineDistWithJerkLimit(
  double velocity, double acceleration, double max_stop_acceleration, double max_stop_jerk,
  double delay_response_time);

struct YellowJudgeParams
{
  double lamp_period{2.75};            ///< assumed remaining yellow duration [s]
  double max_stop_acceleration{-3.0};  ///< deceleration limit [m/s^2] (negative)
  double max_stop_jerk{-3.0};          ///< jerk limit [m/s^3] (negative)
  double delay_response_time{0.5};     ///< reaction delay before braking [s]
  double stop_velocity{1.0};           ///< below this speed the object always stops [m/s]
};

enum class YellowOutcome {
  STOP,     ///< can comfortably stop
  PASS,     ///< clears the junction within the yellow
  DILEMMA,  ///< can neither stop nor clearly clear (emit stop AND pass)
};

/// Amber-signal outcome from the object's kinematics and its distance to the stop
/// line. Mirrors TrafficLightModule::isPassthrough
/// (autoware_behavior_velocity_traffic_light_module) but surfaces the dilemma zone.
YellowOutcome judgeYellow(
  double velocity, double acceleration, double distance_to_stopline,
  const YellowJudgeParams & params);

PriorityCalibration calibratePriority(
  const PriorityContext & context, const PriorityCalibrationParams & params);

/// Raw conservative decision before hysteresis, split out so the node can
/// debounce it across frames before turning it into weights.
ConservativeManeuver decideConservativeManeuver(
  const PriorityContext & context, const PriorityCalibrationParams & params);

PriorityCalibration weightsForManeuver(
  ConservativeManeuver maneuver, const PriorityCalibrationParams & params);

/// Debounce: a new @p raw value becomes the held output only after persisting for
/// @p hysteresis_time seconds; flipping back to the held value cancels the switch.
ConservativeManeuver updateHysteresis(
  HysteresisState & state, ConservativeManeuver raw, double now, double hysteresis_time);

}  // namespace autoware::map_based_prediction::priority

#endif  // MAP_BASED_PREDICTION__PRIORITY_UTILS_HPP_
