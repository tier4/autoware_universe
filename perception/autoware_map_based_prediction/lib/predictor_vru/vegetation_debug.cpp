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
#include <rclcpp/duration.hpp>

#include <geometry_msgs/msg/vector3.hpp>

#include <cstdint>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction::debug
{

namespace
{

constexpr double kMarkerLifetimeSec = 0.3;
constexpr double kPathLineWidth = 0.5;
constexpr double kCutPointDiameter = 0.8;

constexpr const char * kNsObjectBox = "vegetation_vru_box";
constexpr const char * kNsPathLine = "vegetation_vru_path";
constexpr const char * kNsKeptLine = "vegetation_vru_kept";
constexpr const char * kNsCutPoint = "vegetation_cut";

visualization_msgs::msg::Marker make_marker(
  const rclcpp::Time & stamp, const std::string & ns, int32_t type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color)
{
  auto marker = autoware_utils::create_default_marker("map", stamp, ns, 0, type, scale, color);
  marker.lifetime = rclcpp::Duration::from_seconds(kMarkerLifetimeSec);
  return marker;
}

void push_marker(
  visualization_msgs::msg::MarkerArray & markers, visualization_msgs::msg::Marker marker)
{
  marker.id = static_cast<int32_t>(markers.markers.size());
  markers.markers.push_back(std::move(marker));
}

void append_line_strip(
  visualization_msgs::msg::MarkerArray & markers, const rclcpp::Time & stamp,
  const std::string & ns, const std::string & text,
  const std::vector<geometry_msgs::msg::Point> & points, const std_msgs::msg::ColorRGBA & color)
{
  if (points.size() < 2) {
    return;
  }
  auto line = make_marker(
    stamp, ns, visualization_msgs::msg::Marker::LINE_STRIP,
    autoware_utils::create_marker_scale(kPathLineWidth, 0.0, 0.0), color);
  line.text = text;
  line.points = points;
  push_marker(markers, std::move(line));
}

std_msgs::msg::ColorRGBA cut_segment_color()
{
  return autoware_utils::create_marker_color(1.0, 0.53, 0.0, 0.9);
}

std_msgs::msg::ColorRGBA safe_segment_color()
{
  return autoware_utils::create_marker_color(0.53, 0.8, 0.53, 0.9);
}

std_msgs::msg::ColorRGBA cut_point_color()
{
  return autoware_utils::create_marker_color(0.0, 0.67, 0.0, 0.9);
}

struct BoxStyle
{
  geometry_msgs::msg::Vector3 scale;
  std_msgs::msg::ColorRGBA color;
};

BoxStyle box_style_for_label(uint8_t label)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  using autoware_utils::create_marker_color;
  using autoware_utils::create_marker_scale;
  switch (label) {
    case ObjectClassification::PEDESTRIAN:
      return {create_marker_scale(0.6, 0.6, 1.7), create_marker_color(1.0, 0.67, 0.27, 0.75)};
    case ObjectClassification::BICYCLE:
      return {create_marker_scale(1.8, 0.6, 1.5), create_marker_color(0.38, 0.75, 0.38, 0.75)};
    case ObjectClassification::MOTORCYCLE:
      return {create_marker_scale(2.2, 0.8, 1.5), create_marker_color(0.65, 0.45, 0.85, 0.75)};
    default:
      return {create_marker_scale(1.0, 1.0, 1.7), create_marker_color(0.5, 0.5, 0.5, 0.75)};
  }
}

std::string marker_text(const VegetationPathEvent & event)
{
  return event.object_id + ":" + (event.path_debug.cut ? "1" : "0");
}

std::vector<geometry_msgs::msg::Point> collect_positions(
  const PredictedPath & path, size_t begin, size_t end)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(end - begin);
  for (size_t i = begin; i < end; ++i) {
    points.push_back(path.path.at(i).position);
  }
  return points;
}

std::vector<geometry_msgs::msg::Point> kept_line_points(const VegetationPathEvent & event)
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

VegetationPathDebug build_path_debug(
  const PredictedPath & predicted_path, const PredictedPath & cut_path)
{
  const auto total_size = predicted_path.path.size();
  const auto kept_size = cut_path.path.size();

  VegetationPathDebug path_debug;
  path_debug.cut = kept_size < total_size;
  if (!path_debug.cut) {
    path_debug.display_points = collect_positions(predicted_path, 0, total_size);
    return path_debug;
  }
  path_debug.kept_points = collect_positions(predicted_path, 0, kept_size);
  path_debug.display_points = collect_positions(predicted_path, kept_size, total_size);
  return path_debug;
}

geometry_msgs::msg::Point cut_pose(
  const PredictedPath & predicted_path, const PredictedPath & cut_path)
{
  if (cut_path.path.size() >= predicted_path.path.size()) {
    return geometry_msgs::msg::Point{};
  }
  if (!cut_path.path.empty()) {
    return cut_path.path.back().position;
  }
  if (!predicted_path.path.empty()) {
    return predicted_path.path.front().position;
  }
  return geometry_msgs::msg::Point{};
}

void append_object_box(
  visualization_msgs::msg::MarkerArray & markers, const VegetationPathEvent & event,
  const rclcpp::Time & stamp, std::unordered_set<std::string> & boxed_objects)
{
  if (!boxed_objects.insert(event.object_id).second) {
    return;
  }
  const auto style = box_style_for_label(event.label);
  auto box = make_marker(
    stamp, kNsObjectBox, visualization_msgs::msg::Marker::CUBE, style.scale, style.color);
  box.pose = event.object_pose;
  box.text = event.object_id + ":" + std::to_string(event.label);
  push_marker(markers, std::move(box));
}

void append_cut_point(
  visualization_msgs::msg::MarkerArray & markers, const VegetationPathEvent & event,
  const rclcpp::Time & stamp, const std::string & text)
{
  auto sphere = make_marker(
    stamp, kNsCutPoint, visualization_msgs::msg::Marker::SPHERE,
    autoware_utils::create_marker_scale(kCutPointDiameter, kCutPointDiameter, kCutPointDiameter),
    cut_point_color());
  sphere.pose.position = event.cut_pose;
  sphere.text = text;
  push_marker(markers, std::move(sphere));
}

void append_cut_markers(
  visualization_msgs::msg::MarkerArray & markers, const VegetationPathEvent & event,
  const rclcpp::Time & stamp)
{
  const auto text = marker_text(event);
  append_line_strip(
    markers, stamp, kNsPathLine, text, event.path_debug.display_points, cut_segment_color());
  append_line_strip(
    markers, stamp, kNsKeptLine, text, kept_line_points(event), safe_segment_color());
  append_cut_point(markers, event, stamp, text);
}

void append_uncut_markers(
  visualization_msgs::msg::MarkerArray & markers, const VegetationPathEvent & event,
  const rclcpp::Time & stamp)
{
  append_line_strip(
    markers, stamp, kNsPathLine, marker_text(event), event.path_debug.display_points,
    safe_segment_color());
}

}  // namespace

VegetationPathEvent create_vegetation_path_event(
  const PredictedPath & predicted_path, const PredictedPath & cut_path,
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  VegetationPathEvent event;
  event.object_id = autoware_utils::to_hex_string(object.object_id);
  event.label = autoware::object_recognition_utils::getHighestProbLabel(object.classification);
  event.object_pose = object.kinematics.initial_pose_with_covariance.pose;
  event.path_debug = build_path_debug(predicted_path, cut_path);
  event.cut_pose = cut_pose(predicted_path, cut_path);
  return event;
}

void append_vegetation_event_markers(
  visualization_msgs::msg::MarkerArray & markers, const VegetationPathEvent & event,
  const rclcpp::Time & stamp, std::unordered_set<std::string> & boxed_objects)
{
  append_object_box(markers, event, stamp, boxed_objects);
  if (event.path_debug.cut) {
    append_cut_markers(markers, event, stamp);
  } else {
    append_uncut_markers(markers, event, stamp);
  }
}

}  // namespace autoware::map_based_prediction::debug
