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

#include "autoware/map_based_prediction/predictor_vru/vegetation_debug.hpp"

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>

#include <array>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::map_based_prediction::debug
{

namespace
{

std::string markerText(const VegetationPathEvent & event)
{
  return event.object_id + ":" + (event.path_debug.cut ? "1" : "0");
}

std::array<double, 3> boxScale(uint8_t label)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  if (label == ObjectClassification::PEDESTRIAN) {
    return {0.6, 0.6, 1.7};
  }
  if (label == ObjectClassification::BICYCLE) {
    return {1.8, 0.6, 1.5};
  }
  if (label == ObjectClassification::MOTORCYCLE) {
    return {2.2, 0.8, 1.5};
  }
  return {1.0, 1.0, 1.7};
}

std_msgs::msg::ColorRGBA boxColor(uint8_t label)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  if (label == ObjectClassification::PEDESTRIAN) {
    return autoware_utils::create_marker_color(1.0, 0.67, 0.27, 0.75);
  }
  if (label == ObjectClassification::BICYCLE) {
    return autoware_utils::create_marker_color(0.38, 0.75, 0.38, 0.75);
  }
  if (label == ObjectClassification::MOTORCYCLE) {
    return autoware_utils::create_marker_color(0.65, 0.45, 0.85, 0.75);
  }
  return autoware_utils::create_marker_color(0.5, 0.5, 0.5, 0.75);
}

visualization_msgs::msg::Marker makeLineStrip(
  const rclcpp::Time & stamp, const std::string & ns, int32_t id, const std::string & text,
  const std::vector<geometry_msgs::msg::Point> & points, const std_msgs::msg::ColorRGBA & color)
{
  auto line = autoware_utils::create_default_marker(
    "map", stamp, ns, id, visualization_msgs::msg::Marker::LINE_STRIP,
    autoware_utils::create_marker_scale(0.5, 0.0, 0.0), color);
  line.text = text;
  line.lifetime = rclcpp::Duration::from_seconds(0.3);
  line.points = points;
  return line;
}

void pushMarker(
  visualization_msgs::msg::MarkerArray & markers, visualization_msgs::msg::Marker marker)
{
  marker.id = static_cast<int32_t>(markers.markers.size());
  markers.markers.push_back(std::move(marker));
}

std::vector<geometry_msgs::msg::Point> keptLinePoints(const VegetationPathEvent & event)
{
  auto kept = event.path_debug.kept_points;
  if (kept.size() >= 2) {
    return kept;
  }
  if (kept.size() == 1) {
    kept.push_back(event.cut_pose);
    return kept;
  }
  return {event.object_pose.position, event.cut_pose};
}

VegetationPathDebug buildPathDebug(
  const PredictedPath & predicted_path, const PredictedPath & cut_path)
{
  VegetationPathDebug path_debug;
  path_debug.cut = cut_path.path.size() < predicted_path.path.size();
  if (!path_debug.cut) {
    for (const auto & pose : predicted_path.path) {
      path_debug.display_points.push_back(pose.position);
    }
    return path_debug;
  }

  const auto crossing_index = cut_path.path.size();
  for (size_t i = 0; i < crossing_index; ++i) {
    path_debug.kept_points.push_back(predicted_path.path.at(i).position);
  }
  for (size_t i = crossing_index; i < predicted_path.path.size(); ++i) {
    path_debug.display_points.push_back(predicted_path.path.at(i).position);
  }
  return path_debug;
}

geometry_msgs::msg::Point cutPose(
  const PredictedPath & predicted_path, const PredictedPath & cut_path)
{
  geometry_msgs::msg::Point cut_pose;
  if (cut_path.path.size() >= predicted_path.path.size()) {
    return cut_pose;
  }
  if (!cut_path.path.empty()) {
    return cut_path.path.back().position;
  }
  if (!predicted_path.path.empty()) {
    return predicted_path.path.front().position;
  }
  return cut_pose;
}

}  // namespace

VegetationPathEvent createVegetationPathEvent(
  const PredictedPath & predicted_path, const PredictedPath & cut_path,
  const autoware_perception_msgs::msg::TrackedObject & object)
{
  VegetationPathEvent event;
  event.object_id = autoware_utils::to_hex_string(object.object_id);
  event.label = autoware::object_recognition_utils::getHighestProbLabel(object.classification);
  event.object_pose = object.kinematics.pose_with_covariance.pose;
  event.path_debug = buildPathDebug(predicted_path, cut_path);
  event.cut_pose = cutPose(predicted_path, cut_path);
  return event;
}

void appendVegetationEventMarkers(
  visualization_msgs::msg::MarkerArray & markers, const VegetationPathEvent & event,
  const rclcpp::Time & stamp, std::unordered_set<std::string> & boxed_objects)
{
  const auto text = markerText(event);

  if (boxed_objects.insert(event.object_id).second) {
    const auto scale = boxScale(event.label);
    auto box = autoware_utils::create_default_marker(
      "map", stamp, "vegetation_vru_box", 0, visualization_msgs::msg::Marker::CUBE,
      autoware_utils::create_marker_scale(scale[0], scale[1], scale[2]), boxColor(event.label));
    box.pose = event.object_pose;
    box.text = event.object_id + ":" + std::to_string(event.label);
    box.lifetime = rclcpp::Duration::from_seconds(0.3);
    pushMarker(markers, std::move(box));
  }

  if (event.path_debug.cut) {
    if (event.path_debug.display_points.size() >= 2) {
      auto line = makeLineStrip(
        stamp, "vegetation_vru_path", 0, text, event.path_debug.display_points,
        autoware_utils::create_marker_color(1.0, 0.53, 0.0, 0.9));
      pushMarker(markers, std::move(line));
    }

    const auto kept_line = keptLinePoints(event);
    if (kept_line.size() >= 2) {
      auto line = makeLineStrip(
        stamp, "vegetation_vru_kept", 0, text, kept_line,
        autoware_utils::create_marker_color(0.53, 0.8, 0.53, 0.9));
      pushMarker(markers, std::move(line));
    }

    auto sphere = autoware_utils::create_default_marker(
      "map", stamp, "vegetation_cut", 0, visualization_msgs::msg::Marker::SPHERE,
      autoware_utils::create_marker_scale(0.8, 0.8, 0.8),
      autoware_utils::create_marker_color(0.0, 0.67, 0.0, 0.9));
    sphere.pose.position = event.cut_pose;
    sphere.text = text;
    sphere.lifetime = rclcpp::Duration::from_seconds(0.3);
    pushMarker(markers, std::move(sphere));
  } else if (event.path_debug.display_points.size() >= 2) {
    auto line = makeLineStrip(
      stamp, "vegetation_vru_path", 0, text, event.path_debug.display_points,
      autoware_utils::create_marker_color(0.53, 0.8, 0.53, 0.9));
    pushMarker(markers, std::move(line));
  }
}

}  // namespace autoware::map_based_prediction::debug
