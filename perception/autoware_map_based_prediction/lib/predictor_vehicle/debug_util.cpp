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

#include "autoware/map_based_prediction/predictor_vehicle/debug_util.hpp"

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>

#include <string>
#include <vector>

namespace autoware::map_based_prediction::debug_util
{
using autoware_perception_msgs::msg::ObjectClassification;

visualization_msgs::msg::MarkerArray createPriorityObjectMarkers(
  const autoware_perception_msgs::msg::PredictedObjects & output,
  const StopHypothesisIndexMap & stop_hypothesis_indices,
  const std::vector<lanelet::ConstLineString3d> & stop_lines,
  const std::optional<geometry_msgs::msg::Pose> & ego_pose, const rclcpp::Time & now)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = "map";
  clear_marker.header.stamp = now;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear_marker);

  int32_t path_id = 0;
  const auto go_color = autoware_utils::create_marker_color(0.6, 1.0, 0.6, 0.95);
  const auto stop_color = autoware_utils::create_marker_color(1.0, 0.0, 0.0, 1.0);
  for (const auto & obj : output.objects) {
    const auto label =
      obj.classification.empty()
        ? ObjectClassification::UNKNOWN
        : autoware::object_recognition_utils::getHighestProbLabel(obj.classification);
    if (
      label != ObjectClassification::CAR && label != ObjectClassification::BUS &&
      label != ObjectClassification::TRAILER && label != ObjectClassification::MOTORCYCLE &&
      label != ObjectClassification::TRUCK) {
      continue;
    }
    const auto & paths = obj.kinematics.predicted_paths;
    const std::string oid = autoware_utils::to_hex_string(obj.object_id);
    const auto indices_it = stop_hypothesis_indices.find(oid);
    const bool has_stop_hypothesis = indices_it != stop_hypothesis_indices.end();

    const auto & dims = obj.shape.dimensions;
    auto box = autoware_utils::create_default_marker(
      "map", now, "vehicle_boxes", path_id++, visualization_msgs::msg::Marker::CUBE,
      autoware_utils::create_marker_scale(
        dims.x > 0.1 ? dims.x : 4.5, dims.y > 0.1 ? dims.y : 2.0, dims.z > 0.1 ? dims.z : 1.7),
      autoware_utils::create_marker_color(0.75, 0.85, 1.0, 0.45));
    box.pose = obj.kinematics.initial_pose_with_covariance.pose;
    box.lifetime = rclcpp::Duration::from_seconds(0.3);
    markers.markers.push_back(box);

    size_t max_conf_pi = paths.size();
    float max_conf = -1.0f;
    for (size_t pi = 0; pi < paths.size(); ++pi) {
      if (paths.at(pi).path.size() >= 2 && paths.at(pi).confidence > max_conf) {
        max_conf = paths.at(pi).confidence;
        max_conf_pi = pi;
      }
    }
    for (size_t pi = 0; pi < paths.size(); ++pi) {
      if (paths.at(pi).path.size() < 2) {
        continue;
      }
      const bool is_stop_hypothesis = has_stop_hypothesis && indices_it->second.count(pi) > 0;
      auto color = is_stop_hypothesis ? stop_color : go_color;
      color.a = (pi == max_conf_pi) ? 0.7f : 0.1f;
      constexpr double width = 0.6;
      constexpr double z_off = 0.8;
      auto line = autoware_utils::create_default_marker(
        "map", now, "step3_paths", path_id++, visualization_msgs::msg::Marker::LINE_STRIP,
        autoware_utils::create_marker_scale(width, 0.0, 0.0), color);
      line.lifetime = rclcpp::Duration::from_seconds(0.3);
      for (const auto & pose : paths.at(pi).path) {
        auto pt = pose.position;
        pt.z += z_off;
        line.points.push_back(pt);
      }
      markers.markers.push_back(line);
    }
  }

  for (const auto & stop_line : stop_lines) {
    auto sl = autoware_utils::create_default_marker(
      "map", now, "stop_lines", path_id++, visualization_msgs::msg::Marker::LINE_STRIP,
      autoware_utils::create_marker_scale(0.4, 0.0, 0.0),
      autoware_utils::create_marker_color(1.0, 0.0, 1.0, 0.9));  // magenta
    sl.lifetime = rclcpp::Duration::from_seconds(0.3);
    for (const auto & p : stop_line) {
      geometry_msgs::msg::Point pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z() + 0.5;
      sl.points.push_back(pt);
    }
    markers.markers.push_back(sl);
  }

  if (ego_pose) {
    const auto ego_color = autoware_utils::create_marker_color(0.1, 0.9, 1.0, 0.95);  // cyan
    auto ego_box = autoware_utils::create_default_marker(
      "map", now, "ego", 0, visualization_msgs::msg::Marker::CUBE,
      autoware_utils::create_marker_scale(5.0, 2.2, 1.8), ego_color);
    ego_box.pose = *ego_pose;
    ego_box.pose.position.z += 1.5;
    ego_box.lifetime = rclcpp::Duration::from_seconds(0.3);
    markers.markers.push_back(ego_box);

    auto ego_text = autoware_utils::create_default_marker(
      "map", now, "ego_text", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      autoware_utils::create_marker_scale(0.0, 0.0, 2.5),
      autoware_utils::create_marker_color(0.6, 1.0, 1.0, 1.0));
    ego_text.pose = *ego_pose;
    ego_text.pose.position.z += 4.0;
    ego_text.text = "EGO";
    ego_text.lifetime = rclcpp::Duration::from_seconds(0.3);
    markers.markers.push_back(ego_text);
  }

  return markers;
}

void publishPriorityObjectMarkers(
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray> & publisher,
  autoware_utils::TransformListener & transform_listener,
  const autoware_perception_msgs::msg::PredictedObjects & output,
  const rclcpp::Time & objects_stamp, const StopHypothesisIndexMap & stop_hypothesis_indices,
  const std::vector<lanelet::ConstLineString3d> & stop_lines, const rclcpp::Time & now)
{
  std::optional<geometry_msgs::msg::Pose> ego_pose;
  const auto map_to_base = transform_listener.get_transform(
    "map", "base_link", objects_stamp, rclcpp::Duration::from_seconds(0.1));
  if (map_to_base) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = map_to_base->transform.translation.x;
    pose.position.y = map_to_base->transform.translation.y;
    pose.position.z = map_to_base->transform.translation.z;
    pose.orientation = map_to_base->transform.rotation;
    ego_pose = pose;
  }
  publisher.publish(
    createPriorityObjectMarkers(output, stop_hypothesis_indices, stop_lines, ego_pose, now));
}

}  // namespace autoware::map_based_prediction::debug_util
