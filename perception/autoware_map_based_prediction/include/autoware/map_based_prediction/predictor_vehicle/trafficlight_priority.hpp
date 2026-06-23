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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__TRAFFICLIGHT_PRIORITY_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__TRAFFICLIGHT_PRIORITY_HPP_

#include "autoware/map_based_prediction/data_structure.hpp"
#include "autoware/map_based_prediction/predictor_vehicle/debug.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_routing/Forward.h>

#include <cstddef>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::map_based_prediction::trafficlight_priority
{
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;

bool hasTrafficLight(const lanelet::ConstLanelet & way_lanelet);

/// Stop line of the lanelet's TrafficLight regulatory element, if tagged.
std::optional<lanelet::ConstLineString3d> getStopLine(const lanelet::ConstLanelet & way_lanelet);

/// Stop line of the lanelet's TrafficLight regulatory element; falls back to the
/// lanelet's entry edge when no stop line is tagged, so a stopping object still
/// has a finite target at the junction entrance.
std::optional<lanelet::ConstLineString3d> getStopLineOrEntryEdge(
  const lanelet::ConstLanelet & way_lanelet);

/// Id of the TrafficLight regulatory element of @p way_lanelet, if any.
std::optional<lanelet::Id> getTrafficSignalId(const lanelet::ConstLanelet & way_lanelet);

/// Latest observation in @p signal_id_map for the traffic light of @p way_lanelet.
std::optional<TrafficLightGroup> getSignalForLanelet(
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & signal_id_map,
  const lanelet::ConstLanelet & way_lanelet);

struct PriorityCalibrationParams
{
  double stop_probability_boost{0.35};
};

/// Arc length [m] along @p ref_path from its start to where it crosses @p stop_line.
/// Without a crossing, falls back to the path end if that is the vertex nearest the
/// stop-line centroid (the path ends just before the line); otherwise the stop line
/// lies behind / beside the path and nullopt is returned.
std::optional<double> arcLengthToStopLine(
  const PosePath & ref_path, const lanelet::ConstLineString3d & stop_line);

/// Whether @p stop_line still lies ahead of an object at @p position travelling
/// along @p ref_path: the on-path arc length to the line must exceed the object's
/// own signed offset along the path.
bool hasStopLineAhead(
  const geometry_msgs::msg::Point & position, const PosePath & ref_path,
  const lanelet::ConstLineString3d & stop_line);

/// Find the first traffic-light-controlled lanelet a predicted path enters
/// (@p lanelet_path is the lane sequence the path follows).
/// @return true when found, with the lanelet written to @p signal_lanelet
bool findTrafficLightLaneletOnPath(
  const lanelet::routing::LaneletPath & lanelet_path, lanelet::ConstLanelet & signal_lanelet);

/// Whether the signal tells an object on @p lanelet to stop, matching the
/// behavior_velocity traffic_light stop/go boundary (isTrafficSignalStop).
/// No observation -> false.
bool evaluateSignalStopRequirement(
  const lanelet::ConstLanelet & lanelet, const std::optional<TrafficLightGroup> & signal);

/// Whether a stop hypothesis should be added: the feature is enabled, the signal
/// demands a stop, and the stop line still lies ahead of the object.
bool shouldAddStopHypothesis(bool signal_requires_stop, bool has_stop_line_ahead);

/// Stop-hypothesis weight after a lane-change penalty: the lane-follow (center)
/// copy keeps the full stop weight, while lane-change copies are attenuated so
/// the center hypothesis is always the strongest.
double weakenConfidenceInLaneChange(const Maneuver & maneuver, const double stop_weight);

struct ObjectPrediction
{
  const TrackedObject & object;
  const std::vector<PredictedRefPath> & ref_paths;
  const std::vector<lanelet::routing::LaneletPath> & lanelet_paths;
  const std::vector<PredictedPath> & predicted_paths;
};

std::vector<PredictedPath> addTrafficSignalStopHypotheses(
  const ObjectPrediction & prediction,
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & traffic_signal_id_map,
  const PriorityCalibrationParams & params, StopHypothesisDebug & debug);

class TrafficSignalStopPredictor
{
public:
  void setParameters(const PriorityCalibrationParams & params, double signal_observation_timeout)
  {
    params_ = params;
    signal_observation_timeout_ = signal_observation_timeout;
  }

  void setTrafficSignal(const TrafficLightGroupArray & traffic_signals, const rclcpp::Time & now);

  /// Drop the previous frame's marker data (stop lines and hypothesis indices);
  /// the cumulative gate counters survive for the throttled log.
  void clearFrameDebug();

  std::vector<PredictedPath> addStopHypotheses(
    const ObjectPrediction & prediction, const rclcpp::Time & now);

  const StopHypothesisDebug & getDebugInfo() const { return debug_; }

private:
  std::unordered_map<lanelet::Id, TrafficLightGroup> traffic_signal_id_map_;
  std::optional<rclcpp::Time> latest_traffic_signal_time_;
  PriorityCalibrationParams params_;
  double signal_observation_timeout_{0.0};
  StopHypothesisDebug debug_;
};

}  // namespace autoware::map_based_prediction::trafficlight_priority

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__TRAFFICLIGHT_PRIORITY_HPP_
