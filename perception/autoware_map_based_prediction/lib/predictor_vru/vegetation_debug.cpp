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

// All markers live in the "map" frame and fade out after a short lifetime so a path that stops
// crossing vegetation is not left behind. Each marker kind has its own namespace.
constexpr double kMarkerLifetimeSec = 0.3;
constexpr double kPathLineWidth = 0.5;
constexpr double kCutPointDiameter = 0.8;

constexpr const char * kNsObjectBox = "vegetation_vru_box";
constexpr const char * kNsPathLine = "vegetation_vru_path";
constexpr const char * kNsKeptLine = "vegetation_vru_kept";
constexpr const char * kNsCutPoint = "vegetation_cut";

// ---------------------------------------------------------------------------
// Low-level marker helpers
// ---------------------------------------------------------------------------

// The id is left at 0; pushMarker() assigns the final id when the marker is stored.
visualization_msgs::msg::Marker makeMarker(
  const rclcpp::Time & stamp, const std::string & ns, int32_t type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color)
{
  auto marker = autoware_utils::create_default_marker("map", stamp, ns, 0, type, scale, color);
  marker.lifetime = rclcpp::Duration::from_seconds(kMarkerLifetimeSec);
  return marker;
}

void pushMarker(
  visualization_msgs::msg::MarkerArray & markers, visualization_msgs::msg::Marker marker)
{
  marker.id = static_cast<int32_t>(markers.markers.size());
  markers.markers.push_back(std::move(marker));
}

// A LINE_STRIP needs at least two points, so shorter inputs are skipped.
void appendLineStrip(
  visualization_msgs::msg::MarkerArray & markers, const rclcpp::Time & stamp,
  const std::string & ns, const std::string & text,
  const std::vector<geometry_msgs::msg::Point> & points, const std_msgs::msg::ColorRGBA & color)
{
  if (points.size() < 2) {
    return;
  }
  auto line = makeMarker(
    stamp, ns, visualization_msgs::msg::Marker::LINE_STRIP,
    autoware_utils::create_marker_scale(kPathLineWidth, 0.0, 0.0), color);
  line.text = text;
  line.points = points;
  pushMarker(markers, std::move(line));
}

// ---------------------------------------------------------------------------
// Styling
// ---------------------------------------------------------------------------

// Orange: the segment that enters vegetation and is trimmed away.
std_msgs::msg::ColorRGBA cutSegmentColor()
{
  return autoware_utils::create_marker_color(1.0, 0.53, 0.0, 0.9);
}

// Green: a segment that is kept, or a whole path that is not cut.
std_msgs::msg::ColorRGBA safeSegmentColor()
{
  return autoware_utils::create_marker_color(0.53, 0.8, 0.53, 0.9);
}

// Green sphere: the point where the path is cut.
std_msgs::msg::ColorRGBA cutPointColor()
{
  return autoware_utils::create_marker_color(0.0, 0.67, 0.0, 0.9);
}

struct BoxStyle
{
  geometry_msgs::msg::Vector3 scale;
  std_msgs::msg::ColorRGBA color;
};

BoxStyle boxStyleForLabel(uint8_t label)
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

// Short label drawn with the path markers: "<object id>:<1 if cut else 0>".
std::string markerText(const VegetationPathEvent & event)
{
  return event.object_id + ":" + (event.path_debug.cut ? "1" : "0");
}

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

// Positions of the path poses in the half-open index range [begin, end).
std::vector<geometry_msgs::msg::Point> collectPositions(
  const PredictedPath & path, size_t begin, size_t end)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(end - begin);
  for (size_t i = begin; i < end; ++i) {
    points.push_back(path.path.at(i).position);
  }
  return points;
}

// At least two points for the kept-segment line, falling back to the object and cut positions when
// the kept segment alone is too short to draw.
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
  const auto total_size = predicted_path.path.size();
  const auto kept_size = cut_path.path.size();

  VegetationPathDebug path_debug;
  path_debug.cut = kept_size < total_size;
  if (!path_debug.cut) {
    path_debug.display_points = collectPositions(predicted_path, 0, total_size);
    return path_debug;
  }
  // Trimmed at kept_size: [0, kept_size) is kept, [kept_size, total_size) enters vegetation.
  path_debug.kept_points = collectPositions(predicted_path, 0, kept_size);
  path_debug.display_points = collectPositions(predicted_path, kept_size, total_size);
  return path_debug;
}

// The position where the path is cut: the last kept pose, or the path start when fully cut.
geometry_msgs::msg::Point cutPose(
  const PredictedPath & predicted_path, const PredictedPath & cut_path)
{
  if (cut_path.path.size() >= predicted_path.path.size()) {
    return geometry_msgs::msg::Point{};  // not cut
  }
  if (!cut_path.path.empty()) {
    return cut_path.path.back().position;
  }
  if (!predicted_path.path.empty()) {
    return predicted_path.path.front().position;
  }
  return geometry_msgs::msg::Point{};
}

// ---------------------------------------------------------------------------
// Per-event marker appenders
// ---------------------------------------------------------------------------

// Box each object only once per frame (the first event for the object wins).
void appendObjectBox(
  visualization_msgs::msg::MarkerArray & markers, const VegetationPathEvent & event,
  const rclcpp::Time & stamp, std::unordered_set<std::string> & boxed_objects)
{
  if (!boxed_objects.insert(event.object_id).second) {
    return;  // already drawn for this object
  }
  const auto style = boxStyleForLabel(event.label);
  auto box = makeMarker(
    stamp, kNsObjectBox, visualization_msgs::msg::Marker::CUBE, style.scale, style.color);
  box.pose = event.object_pose;
  box.text = event.object_id + ":" + std::to_string(event.label);
  pushMarker(markers, std::move(box));
}

void appendCutPoint(
  visualization_msgs::msg::MarkerArray & markers, const VegetationPathEvent & event,
  const rclcpp::Time & stamp, const std::string & text)
{
  auto sphere = makeMarker(
    stamp, kNsCutPoint, visualization_msgs::msg::Marker::SPHERE,
    autoware_utils::create_marker_scale(kCutPointDiameter, kCutPointDiameter, kCutPointDiameter),
    cutPointColor());
  sphere.pose.position = event.cut_pose;
  sphere.text = text;
  pushMarker(markers, std::move(sphere));
}

void appendCutMarkers(
  visualization_msgs::msg::MarkerArray & markers, const VegetationPathEvent & event,
  const rclcpp::Time & stamp)
{
  const auto text = markerText(event);
  appendLineStrip(
    markers, stamp, kNsPathLine, text, event.path_debug.display_points, cutSegmentColor());
  appendLineStrip(markers, stamp, kNsKeptLine, text, keptLinePoints(event), safeSegmentColor());
  appendCutPoint(markers, event, stamp, text);
}

void appendUncutMarkers(
  visualization_msgs::msg::MarkerArray & markers, const VegetationPathEvent & event,
  const rclcpp::Time & stamp)
{
  appendLineStrip(
    markers, stamp, kNsPathLine, markerText(event), event.path_debug.display_points,
    safeSegmentColor());
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
  appendObjectBox(markers, event, stamp, boxed_objects);
  if (event.path_debug.cut) {
    appendCutMarkers(markers, event, stamp);
  } else {
    appendUncutMarkers(markers, event, stamp);
  }
}

}  // namespace autoware::map_based_prediction::debug
