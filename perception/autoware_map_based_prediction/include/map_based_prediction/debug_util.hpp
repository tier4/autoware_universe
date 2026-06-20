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

#ifndef MAP_BASED_PREDICTION__DEBUG_UTIL_HPP_
#define MAP_BASED_PREDICTION__DEBUG_UTIL_HPP_

#include "map_based_prediction/data_structure.hpp"

#include <autoware_utils/ros/transform_listener.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction::debug_util
{

/// Cumulative gate-fire counters for the throttled debug log.
struct DebugLogCounter
{
  size_t vehicles = 0;
  size_t signal_stop = 0;
  size_t stopline_found = 0;
  size_t stop_hypothesis_added = 0;
  size_t should_add = 0;
  size_t clip_failed = 0;
  size_t skipped_mismatch = 0;
  size_t tl_missing = 0;
  size_t red_no_stopline = 0;
  size_t red_behind_straight = 0;
  size_t red_behind_turn = 0;
};

/// object id -> set of predicted_path indices holding a stop hypothesis.
using StopHypothesisIndexMap = std::unordered_map<std::string, std::unordered_set<size_t>>;

/// Debug-only outputs of the stop-hypothesis calibration. Never feeds back into
/// the prediction; consumed only by the debug markers / throttled log.
struct StopHypothesisDebug
{
  /// Cumulative gate-fire counters for the throttled log.
  DebugLogCounter counter;
  /// Stop lines that drove a hypothesis this frame (magenta markers).
  std::vector<lanelet::ConstLineString3d> stop_lines;
  /// object id -> predicted_path indices to colour as stop hypotheses.
  StopHypothesisIndexMap stop_hypothesis_path_indices;
  std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> stop_signal_links;
};

void recordStopSignalLink(
  const lanelet::ConstLanelet & signal_lanelet, const geometry_msgs::msg::Point & vehicle_position,
  StopHypothesisDebug & debug);

/// Per-frame priority-prediction debug markers: vehicle boxes, predicted-path
/// lines coloured go/stop, the stop lines that drove a stop hypothesis, the
/// stabilized signals the decision actually used (ns "used_signals", marker id =
/// traffic-light-group id, colour = signal colour), and the ego box. Starts with
/// a DELETEALL marker.
visualization_msgs::msg::MarkerArray createPriorityObjectMarkers(
  const autoware_perception_msgs::msg::PredictedObjects & output,
  const StopHypothesisIndexMap & stop_hypothesis_indices,
  const std::vector<lanelet::ConstLineString3d> & stop_lines,
  const std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> &
    stop_signal_links,
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & stabilized_signals,
  const std::optional<geometry_msgs::msg::Pose> & ego_pose, const rclcpp::Time & now);

/// Resolve the ego pose from TF (map -> base_link at @p objects_stamp) and publish
/// the priority debug markers on @p publisher.
void publishPriorityObjectMarkers(
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray> & publisher,
  autoware_utils::TransformListener & transform_listener,
  const autoware_perception_msgs::msg::PredictedObjects & output,
  const rclcpp::Time & objects_stamp, const StopHypothesisIndexMap & stop_hypothesis_indices,
  const std::vector<lanelet::ConstLineString3d> & stop_lines,
  const std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> &
    stop_signal_links,
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & stabilized_signals,
  const rclcpp::Time & now);

visualization_msgs::msg::Marker createManeuverMarker(
  const autoware_perception_msgs::msg::TrackedObject & object, Maneuver maneuver,
  std::size_t marker_id);

void logPriorityCounters(
  const rclcpp::Logger & logger, rclcpp::Clock & clock, const DebugLogCounter & counter);

}  // namespace autoware::map_based_prediction::debug_util

#endif  // MAP_BASED_PREDICTION__DEBUG_UTIL_HPP_
