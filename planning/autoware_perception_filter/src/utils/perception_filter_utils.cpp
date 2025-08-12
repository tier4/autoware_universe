// Copyright 2025 TIER IV, Inc.
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

#include "autoware/perception_filter/perception_filter_utils.hpp"

#include "autoware/perception_filter/perception_filter_core.hpp"

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::perception_filter
{

double getDistanceAlongPath(
  const geometry_msgs::msg::Point & point,
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & planning_trajectory,
  const geometry_msgs::msg::Pose & ego_pose)
{
  if (!planning_trajectory || planning_trajectory->points.empty()) {
    return 0.0;
  }

  // Find the closest point on the trajectory to the ego vehicle
  double min_ego_distance = std::numeric_limits<double>::max();
  size_t ego_closest_index = 0;

  for (size_t i = 0; i < planning_trajectory->points.size(); ++i) {
    const auto & path_point = planning_trajectory->points[i].pose.position;
    const double dx = ego_pose.position.x - path_point.x;
    const double dy = ego_pose.position.y - path_point.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < min_ego_distance) {
      min_ego_distance = distance;
      ego_closest_index = i;
    }
  }

  // Find the closest point on the trajectory to the given point
  double min_point_distance = std::numeric_limits<double>::max();
  size_t point_closest_index = 0;

  for (size_t i = 0; i < planning_trajectory->points.size(); ++i) {
    const auto & path_point = planning_trajectory->points[i].pose.position;
    const double dx = point.x - path_point.x;
    const double dy = point.y - path_point.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < min_point_distance) {
      min_point_distance = distance;
      point_closest_index = i;
    }
  }

  // Calculate cumulative distance along path
  double cumulative_distance = 0.0;
  const size_t start_index = std::min(ego_closest_index, point_closest_index);
  const size_t end_index = std::max(ego_closest_index, point_closest_index);
  const bool is_point_ahead = (point_closest_index >= ego_closest_index);

  for (size_t i = start_index; i < end_index; ++i) {
    const auto & current_point = planning_trajectory->points[i].pose.position;
    const auto & next_point = planning_trajectory->points[i + 1].pose.position;
    const double dx = next_point.x - current_point.x;
    const double dy = next_point.y - current_point.y;
    const double segment_distance = std::sqrt(dx * dx + dy * dy);
    cumulative_distance += segment_distance;
  }

  return is_point_ahead ? cumulative_distance : -cumulative_distance;
}

autoware_internal_planning_msgs::msg::PlanningFactorArray createPlanningFactors(
  const ObjectClassification & classification,
  const std::vector<FilteredPointInfo> & would_be_filtered_points,
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & planning_trajectory)
{
  autoware_internal_planning_msgs::msg::PlanningFactorArray planning_factors;

  // Set header with current timestamp
  planning_factors.header.stamp = rclcpp::Clock().now();
  planning_factors.header.frame_id = "map";

  // Get UUIDs of objects that pass through now but would be filtered if RTC approved
  std::vector<unique_identifier_msgs::msg::UUID> objects_to_be_filtered;
  for (const auto & object : classification.pass_through_would_filter) {
    objects_to_be_filtered.push_back(object.object_id);
  }

  // Create PlanningFactor when there are objects that would be filtered when RTC is approved
  // OR when there are points that would be filtered when RTC is approved
  if (!objects_to_be_filtered.empty() || !would_be_filtered_points.empty()) {
    autoware_internal_planning_msgs::msg::PlanningFactor factor;
    factor.module = "supervised_perception_filter";
    factor.behavior = autoware_internal_planning_msgs::msg::PlanningFactor::STOP;

    // Set detail message based on what is being filtered
    const bool has_objects = !objects_to_be_filtered.empty();
    const bool has_points = !would_be_filtered_points.empty();

    if (has_objects && has_points) {
      factor.detail = "Objects and pointcloud that would be filtered when RTC is approved";
    } else if (has_objects) {
      factor.detail = "Objects that would be filtered when RTC is approved";
    } else if (has_points) {
      factor.detail = "Pointcloud that would be filtered when RTC is approved";
    }

    // Add control point (nearest point on path)
    if (planning_trajectory && !planning_trajectory->points.empty()) {
      autoware_internal_planning_msgs::msg::ControlPoint control_point;
      control_point.pose = planning_trajectory->points.front().pose;
      control_point.distance = 0.0;
      factor.control_points.push_back(control_point);
    }

    // Add safety factors
    factor.safety_factors.is_safe = false;

    // Add object safety factors
    for (const auto & uuid : objects_to_be_filtered) {
      autoware_internal_planning_msgs::msg::SafetyFactor safety_factor;
      safety_factor.type = autoware_internal_planning_msgs::msg::SafetyFactor::OBJECT;
      safety_factor.is_safe = false;
      safety_factor.object_id = uuid;

      // Add object position
      for (const auto & object : classification.pass_through_would_filter) {
        if (object.object_id.uuid == uuid.uuid) {
          geometry_msgs::msg::Point point;
          point.x = object.kinematics.initial_pose_with_covariance.pose.position.x;
          point.y = object.kinematics.initial_pose_with_covariance.pose.position.y;
          point.z = object.kinematics.initial_pose_with_covariance.pose.position.z;
          safety_factor.points.push_back(point);
          break;
        }
      }

      factor.safety_factors.factors.push_back(safety_factor);
    }

    // Add pointcloud safety factors (only points that would be filtered when RTC is approved)
    if (!would_be_filtered_points.empty()) {
      autoware_internal_planning_msgs::msg::SafetyFactor pointcloud_safety_factor;
      pointcloud_safety_factor.type =
        autoware_internal_planning_msgs::msg::SafetyFactor::POINTCLOUD;
      pointcloud_safety_factor.is_safe = false;

      // Add would-be-filtered points positions
      for (const auto & filtered_info : would_be_filtered_points) {
        pointcloud_safety_factor.points.push_back(filtered_info.point);
      }

      factor.safety_factors.factors.push_back(pointcloud_safety_factor);
    }

    planning_factors.factors.push_back(factor);
  }

  return planning_factors;
}

visualization_msgs::msg::MarkerArray createDebugMarkers(
  const autoware_perception_msgs::msg::PredictedObjects & input_objects,
  const ObjectClassification & classification, bool rtc_activated,
  const geometry_msgs::msg::Pose & ego_pose,
  const autoware::universe_utils::Polygon2d & filtering_polygon, bool filtering_polygon_created)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // First, delete all previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header.frame_id = "map";
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  int marker_id = 0;

  // Helper lambda to create object markers with text
  auto create_object_markers =
    [&](
      const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects,
      const std::string & ns_prefix, const std::string & text,
      const std::array<double, 4> & color) {
      if (objects.empty()) return;

      autoware_perception_msgs::msg::PredictedObjects objects_msg;
      objects_msg.header = input_objects.header;
      objects_msg.objects = objects;

      auto object_marker = createObjectMarker(objects_msg, "map", marker_id++, color);
      object_marker.ns = ns_prefix + "_objects";
      marker_array.markers.push_back(object_marker);

      // Add text markers
      for (size_t i = 0; i < objects.size(); ++i) {
        const auto & object = objects[i];
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.ns = ns_prefix + "_text";
        text_marker.id = static_cast<int>(i);
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position = object.kinematics.initial_pose_with_covariance.pose.position;
        text_marker.pose.position.z += 2.0;  // Display above object
        text_marker.scale.z = 0.5;
        text_marker.color.r = color[0];
        text_marker.color.g = color[1];
        text_marker.color.b = color[2];
        text_marker.color.a = 1.0;
        text_marker.text = text;
        marker_array.markers.push_back(text_marker);
      }
    };

  // Create markers for different object categories
  create_object_markers(
    classification.pass_through_always, "always_pass", "ALWAYS PASS", {0.0, 0.0, 1.0, 0.8});
  create_object_markers(
    classification.pass_through_would_filter, "would_filter", "WOULD FILTER", {1.0, 1.0, 0.0, 0.8});
  create_object_markers(
    classification.currently_filtered, "filtered", "FILTERED", {1.0, 0.0, 0.0, 0.8});

  // Create filtering polygon marker if available
  if (filtering_polygon_created && !filtering_polygon.outer().empty()) {
    visualization_msgs::msg::Marker polygon_marker;
    polygon_marker.header.frame_id = "map";
    polygon_marker.ns = "filtering_polygon";
    polygon_marker.id = 0;
    polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    polygon_marker.action = visualization_msgs::msg::Marker::ADD;
    polygon_marker.scale.x = 0.3;
    polygon_marker.color.r = 0.0;
    polygon_marker.color.g = 1.0;
    polygon_marker.color.b = 0.0;
    polygon_marker.color.a = 0.8;

    // Add polygon points to create a closed polygon
    for (const auto & point : filtering_polygon.outer()) {
      geometry_msgs::msg::Point ros_point;
      ros_point.x = point.x();
      ros_point.y = point.y();
      ros_point.z = 0.1;
      polygon_marker.points.push_back(ros_point);
    }

    // Close the polygon by adding the first point again
    if (!filtering_polygon.outer().empty()) {
      const auto & first_point = filtering_polygon.outer().front();
      geometry_msgs::msg::Point ros_point;
      ros_point.x = first_point.x();
      ros_point.y = first_point.y();
      ros_point.z = 0.1;
      polygon_marker.points.push_back(ros_point);
    }

    marker_array.markers.push_back(polygon_marker);
  }

  // Create status text marker above ego vehicle
  visualization_msgs::msg::Marker status_marker;
  status_marker.header.frame_id = "map";
  status_marker.ns = "rtc_status";
  status_marker.id = 0;
  status_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  status_marker.action = visualization_msgs::msg::Marker::ADD;
  status_marker.pose.position.x = ego_pose.position.x;
  status_marker.pose.position.y = ego_pose.position.y;
  status_marker.pose.position.z = ego_pose.position.z + 3.0;  // 3m above ego vehicle
  status_marker.scale.z = 1.0;
  status_marker.color.r = rtc_activated ? 0.0 : 1.0;
  status_marker.color.g = rtc_activated ? 1.0 : 0.0;
  status_marker.color.b = 0.0;
  status_marker.color.a = 1.0;

  std::string status_text = rtc_activated ? "RTC: ACTIVATED" : "RTC: NOT ACTIVATED";
  status_text += "\nAlways Pass: " + std::to_string(classification.pass_through_always.size());
  status_text +=
    "\nWould Filter: " + std::to_string(classification.pass_through_would_filter.size());
  status_text += "\nFiltered: " + std::to_string(classification.currently_filtered.size());

  status_marker.text = status_text;
  marker_array.markers.push_back(status_marker);

  return marker_array;
}

visualization_msgs::msg::Marker createObjectMarker(
  const autoware_perception_msgs::msg::PredictedObjects & objects, const std::string & frame_id,
  int id, const std::array<double, 4> & color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "perception_filter_debug";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = color[3];
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];

  for (const auto & object : objects.objects) {
    geometry_msgs::msg::Point point;
    point.x = object.kinematics.initial_pose_with_covariance.pose.position.x;
    point.y = object.kinematics.initial_pose_with_covariance.pose.position.y;
    point.z = object.kinematics.initial_pose_with_covariance.pose.position.z;
    marker.points.push_back(point);
  }

  return marker;
}

}  // namespace autoware::perception_filter
