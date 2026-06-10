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

// Traffic-signal-aware stop/creep prediction helpers.
//
// This module ports the traffic-signal-priority reasoning that the
// behavior_velocity intersection module computes from an *ego* point of view
// into a set of *object-centric* pure functions usable from the per-object
// prediction loop. Each predicted reference path is classified by the first
// traffic-light-controlled lanelet its lanelet sequence enters; the signal state
// for that lanelet then decides whether to add a stop (red) or creep (amber)
// hypothesis at the lanelet's stop line.
//
// Everything here depends only on the lanelet map and an (optional) traffic-
// signal observation. No node state is required, so the functions are trivially
// unit-testable and degrade to NOT_PRIORITIZED when no signal is observed.

#include "map_based_prediction/data_structure.hpp"

#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_routing/Forward.h>

#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace autoware::map_based_prediction::priority
{
using autoware_perception_msgs::msg::TrafficLightGroup;

/// How the traffic signal constrains an object on a given lanelet.
enum class SignalPriority {
  NOT_PRIORITIZED = 0,    ///< green / matching green arrow -> object may proceed
  PARTIALLY_PRIORITIZED,  ///< amber -> object should creep / be cautious
  FULLY_PRIORITIZED,      ///< red / non-matching arrow -> object must stop
};

/// Aggregated traffic-signal context for a single predicted reference path.
struct PriorityContext
{
  SignalPriority signal_priority{SignalPriority::NOT_PRIORITIZED};
  bool has_traffic_light{false};
  double distance_to_stopline{std::numeric_limits<double>::infinity()};
};

/// The conservative hypothesis to add for a signalled object.
///
/// Both CREEP and STOP currently emit a *stop-at-the-line* predicted path (the
/// amber pass/keep-go cases are filtered earlier by judgeYellow); they differ
/// only in weighting and -- for CREEP -- in keeping the go hypothesis in the
/// dilemma zone. CREEP is kept as a distinct value so amber-driven stops can be
/// coloured and weighted separately from red.
enum class ConservativeManeuver {
  NONE = 0,  ///< no conservative path (object keeps going)
  CREEP,     ///< amber: cautious yield, predicted as a stop at the line (see above)
  STOP,      ///< red: brake to a stop at the stop line
};

/// Tunables for calibratePriority, mirroring the priority_prediction param block.
struct PriorityCalibrationParams
{
  bool use_signal_priority{true};
  double stop_probability_boost{0.35};
  double creep_probability_boost{0.25};
  double go_probability_decay_on_yield{0.5};
};

/// Result of the priority calibration: which conservative path to add (if any),
/// the absolute confidence weight to give it, and how much to scale the existing
/// "go" path confidences before the node re-normalizes the whole set.
struct PriorityCalibration
{
  ConservativeManeuver maneuver{ConservativeManeuver::NONE};
  double conservative_weight{0.0};
  double go_scale{1.0};
};

/// Per-object debounce state for the conservative decision (chattering
/// suppression). Tracks the currently held maneuver plus the pending candidate
/// and when it first appeared.
struct HysteresisState
{
  ConservativeManeuver held{ConservativeManeuver::NONE};
  ConservativeManeuver candidate{ConservativeManeuver::NONE};
  double candidate_since{0.0};
};

/// Read the "turn_direction" attribute of a lanelet ("straight" if absent).
std::string getTurnDirection(const lanelet::ConstLanelet & lanelet);

/// True if the lanelet has at least one TrafficLight regulatory element.
bool hasTrafficLight(const lanelet::ConstLanelet & lanelet);

/// Extract the traffic-light stop line associated with @p lanelet.
///
/// Returns nullopt when the lanelet has no TrafficLight regulatory element with a
/// tagged stop line.
std::optional<lanelet::ConstLineString3d> getStopLine(const lanelet::ConstLanelet & lanelet);

/// Arc length [m] along @p ref_path from its start to where it crosses
/// @p stop_line (falling back to the vertex nearest the stop-line centroid).
///
/// @p ref_path is the object's lane-follow pose path (starting at the object).
/// Returns nullopt if the path has fewer than two points. The result is clamped
/// to >= 0 (an object already past the line yields ~0).
std::optional<double> arcLengthToStopLine(
  const PosePath & ref_path, const lanelet::ConstLineString3d & stop_line);

/// A neighbouring object reduced to what lead-vehicle reasoning needs: its id,
/// map-frame pose and longitudinal extent (shape.dimensions.x).
struct LaneObject
{
  std::string id;
  geometry_msgs::msg::Pose pose;
  double length{0.0};
};

/// Arc length [m] along @p ref_path, from its start, at which the ego object's
/// reference point must stop to keep a gap behind the nearest *in-lane leading*
/// object.
///
/// Followers in a queue must not target the stop line directly -- they should
/// stop behind the vehicle ahead. Each candidate in @p objects is projected onto
/// @p ref_path: an object counts as a leader when its lateral offset from the
/// path is within @p lateral_threshold and it lies ahead of @p ref_path's start.
/// The returned distance leaves @p gap_margin plus both half-lengths between the
/// two bodies. Returns nullopt when no leader is ahead.
///
/// The result is measured from @p ref_path's start (the same convention as
/// arcLengthToStopLine); the caller subtracts the ego's own offset to make it
/// object-relative.
///
/// @param ref_path          ego lane-follow reference path (map frame)
/// @param ego_id            ego object id (skipped when scanning @p objects)
/// @param ego_length        ego longitudinal extent (shape.dimensions.x)
/// @param objects           candidate neighbours (map frame)
/// @param lateral_threshold max lateral offset to be considered same-lane [m]
/// @param gap_margin        bumper-to-bumper gap to keep behind the leader [m]
/// @param ego_arc           the ego's own arc length along @p ref_path; "ahead"
///                          is measured from here (the path may start behind the
///                          ego), so objects between ref_path[0] and the ego are
///                          excluded. Defaults to 0 (path starts at the ego).
std::optional<double> distanceToLeadObject(
  const PosePath & ref_path, const std::string & ego_id, double ego_length,
  const std::vector<LaneObject> & objects, double lateral_threshold, double gap_margin,
  double ego_arc = 0.0);

/// Classification of a single predicted reference path at the traffic light it
/// enters.
///
/// Each predicted reference path is classified by the first traffic-light-
/// controlled lanelet its lanelet sequence enters, so the signal observed for
/// that lanelet (and its stop line) can drive a stop/creep hypothesis.
struct PathSignalInfo
{
  bool found{false};                     ///< a signalized lanelet was found on the path
  lanelet::ConstLanelet signal_lanelet;  ///< the first traffic-light-controlled lanelet entered
  std::optional<lanelet::ConstLineString3d> stop_line;  ///< stop line of the signalized lanelet
  std::string turn_direction{"straight"};  ///< turn_direction of the signalized lanelet
};

/// Classify a predicted reference path by the first traffic-light-controlled
/// lanelet its @p lanelet_path enters.
///
/// Walks the lanelet sequence from the object forward and stops at the first
/// lanelet that references a TrafficLight regulatory element. The stop line is
/// that traffic light's stop line, or the lanelet's entry edge when none is
/// tagged, so a stopping object always has a finite target at the junction
/// entrance. The turn_direction is read so the caller can evaluate arrow signals.
PathSignalInfo classifyPathAtTrafficLight(const lanelet::routing::LaneletPath & lanelet_path);

/// Classify how a traffic signal constrains an object on @p lanelet.
///
/// Reuses autoware::traffic_light_utils::isTrafficSignalStop so the stop / go
/// boundary matches the behavior_velocity traffic_light module exactly: a green
/// circle, or a green arrow matching the lanelet's turn_direction (a protected
/// movement), means the object may proceed (NOT_PRIORITIZED); red or a
/// non-matching arrow means it must stop. A solid amber circle without a red
/// (i.e. a stop is required but cautionary) is reported as PARTIALLY_PRIORITIZED
/// (creep). No observation -> NOT_PRIORITIZED.
///
/// @param lanelet  the signalized lanelet (its turn_direction selects the arrows)
/// @param signal   the latest signal observation for that lanelet, if any
SignalPriority evaluateSignalPriority(
  const lanelet::ConstLanelet & lanelet, const std::optional<TrafficLightGroup> & signal);

/// Distance [m] needed to stop from (@p velocity, @p acceleration) under
/// deceleration and jerk limits after a response delay.
///
/// NOTE: ported verbatim from
/// autoware::behavior_velocity_planner::planning_utils::calcJudgeLineDistWithJerkLimit
/// (autoware_behavior_velocity_planner_common) so that map_based_prediction
/// (perception) does not take a dependency on the planning stack. Keep in sync
/// with the original if its formula changes.
double judgeLineDistWithJerkLimit(
  double velocity, double acceleration, double max_stop_acceleration, double max_stop_jerk,
  double delay_response_time);

/// Tunables for the amber-signal pass/stop judgement (judgeYellow).
struct YellowJudgeParams
{
  double lamp_period{2.75};            ///< assumed remaining yellow duration [s]
  double max_stop_acceleration{-3.0};  ///< deceleration limit [m/s^2] (negative)
  double max_stop_jerk{-3.0};          ///< jerk limit [m/s^3] (negative)
  double delay_response_time{0.5};     ///< reaction delay before braking [s]
  double stop_velocity{1.0};           ///< below this speed the object always stops [m/s]
};

/// Likely behaviour of an object facing an amber signal.
enum class YellowOutcome {
  STOP,     ///< can comfortably stop -> predict a stop at the line
  PASS,     ///< cannot stop but clears the junction within the yellow -> keep going
  DILEMMA,  ///< can neither stop nor clearly clear -> ambiguous (emit stop AND pass)
};

/// Decide the amber-signal outcome from the object's kinematics and its distance to
/// the stop line.
///
/// NOTE: mirrors the branching of
/// TrafficLightModule::isPassthrough (autoware_behavior_velocity_traffic_light_module)
/// -- stoppable -> STOP, not-stoppable & reachable -> PASS, otherwise DILEMMA --
/// but returns the three-way outcome (instead of a single ego stop/pass decision)
/// so the predictor can emit a multimodal hypothesis in the dilemma zone.
YellowOutcome judgeYellow(
  double velocity, double acceleration, double distance_to_stopline,
  const YellowJudgeParams & params);

/// Decide whether to add a conservative (stop / creep) hypothesis for an object
/// and how to reweight its existing "go" hypotheses, from its PriorityContext.
///
/// Rules:
///  - A FULLY_PRIORITIZED signal (red / crossing arrow) -> STOP, decaying the go
///    confidences by go_probability_decay_on_yield.
///  - A PARTIALLY_PRIORITIZED signal (amber) -> CREEP, leaving the go confidences
///    intact (the object may still proceed).
///  - NOT_PRIORITIZED (green / no signal) -> NONE.
///
/// A conservative path is only added when the path reaches a finite stop line
/// (there is no distance threshold); otherwise the result is NONE.
PriorityCalibration calibratePriority(
  const PriorityContext & context, const PriorityCalibrationParams & params);

/// The raw conservative decision (STOP/CREEP/NONE) before hysteresis, gated by
/// the signal trigger and whether a finite stop line is known. Split out so the
/// node can debounce it across frames before turning it into weights.
ConservativeManeuver decideConservativeManeuver(
  const PriorityContext & context, const PriorityCalibrationParams & params);

/// Map a (possibly hysteresis-held) maneuver to its confidence weights.
PriorityCalibration weightsForManeuver(
  ConservativeManeuver maneuver, const PriorityCalibrationParams & params);

/// Debounce the conservative decision (chattering suppression).
///
/// Updates @p state in place and returns the held maneuver. A new @p raw value
/// only becomes the held output once it has persisted continuously for
/// @p hysteresis_time seconds; flipping back to the held value cancels a pending
/// switch. @p now and @p hysteresis_time share the same time unit (seconds).
ConservativeManeuver updateHysteresis(
  HysteresisState & state, ConservativeManeuver raw, double now, double hysteresis_time);

}  // namespace autoware::map_based_prediction::priority

#endif  // MAP_BASED_PREDICTION__PRIORITY_UTILS_HPP_
