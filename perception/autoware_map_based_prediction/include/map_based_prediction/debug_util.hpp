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

#include <autoware_utils/ros/transform_listener.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/primitives/LineString.h>

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::map_based_prediction::debug_util
{

/// Gate-fire counters for the priority calibration, logged throttled.
struct PriorityDebugCounters
{
  size_t vehicles = 0;
  size_t signal_stop = 0;
  size_t stopline_found = 0;
  size_t conservative_added = 0;
};

/// object id -> set of predicted_path indices holding a conservative (stop) path.
using ConservativePathIndexMap = std::unordered_map<std::string, std::unordered_set<size_t>>;

/// Per-frame priority-prediction debug markers: vehicle boxes, predicted-path
/// lines coloured go/stop, the stop lines that drove a conservative
/// hypothesis, and the ego box. Starts with a DELETEALL marker.
visualization_msgs::msg::MarkerArray createPriorityObjectMarkers(
  const autoware_perception_msgs::msg::PredictedObjects & output,
  const ConservativePathIndexMap & conservative_path_indices,
  const std::vector<lanelet::ConstLineString3d> & stop_lines,
  const std::optional<geometry_msgs::msg::Pose> & ego_pose, const rclcpp::Time & now);

/// Resolve the ego pose from TF (map -> base_link at @p objects_stamp) and publish
/// the priority debug markers on @p publisher.
void publishPriorityObjectMarkers(
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray> & publisher,
  autoware_utils::TransformListener & transform_listener,
  const autoware_perception_msgs::msg::PredictedObjects & output,
  const rclcpp::Time & objects_stamp, const ConservativePathIndexMap & conservative_path_indices,
  const std::vector<lanelet::ConstLineString3d> & stop_lines, const rclcpp::Time & now);

}  // namespace autoware::map_based_prediction::debug_util

#endif  // MAP_BASED_PREDICTION__DEBUG_UTIL_HPP_
