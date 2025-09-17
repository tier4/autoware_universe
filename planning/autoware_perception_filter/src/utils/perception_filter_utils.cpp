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

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/transform_listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::perception_filter
{

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
  for (const auto & object : classification.would_be_removed) {
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
      for (const auto & object : classification.would_be_removed) {
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
  const std::vector<autoware::universe_utils::Polygon2d> & filtering_polygon)
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
    classification.kept_objects, "always_pass", "ALWAYS PASS", {0.0, 0.0, 1.0, 0.8});
  create_object_markers(
    classification.would_be_removed, "would_filter", "WOULD FILTER", {1.0, 1.0, 0.0, 0.8});
  create_object_markers(
    classification.removed_objects, "filtered", "FILTERED", {1.0, 0.0, 0.0, 0.8});

  // Create filtering polygon marker if available
  if (!filtering_polygon.empty()) {
    for (size_t i = 0; i < filtering_polygon.size(); ++i) {
      const auto & polygon = filtering_polygon[i];
      if (polygon.outer().empty()) continue;

      visualization_msgs::msg::Marker polygon_marker;
      polygon_marker.header.frame_id = "map";
      polygon_marker.ns = "filtering_polygon";
      polygon_marker.id = static_cast<int>(i);
      polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      polygon_marker.action = visualization_msgs::msg::Marker::ADD;
      polygon_marker.scale.x = 0.3;
      polygon_marker.color.r = 0.0;
      polygon_marker.color.g = 1.0;
      polygon_marker.color.b = 0.0;
      polygon_marker.color.a = 0.8;

      // Add polygon points to create a closed polygon
      for (const auto & point : polygon.outer()) {
        geometry_msgs::msg::Point ros_point;
        ros_point.x = point.x();
        ros_point.y = point.y();
        ros_point.z = 0.1;
        polygon_marker.points.push_back(ros_point);
      }

      // Close the polygon by adding the first point again
      if (!polygon.outer().empty()) {
        const auto & first_point = polygon.outer().front();
        geometry_msgs::msg::Point ros_point;
        ros_point.x = first_point.x();
        ros_point.y = first_point.y();
        ros_point.z = 0.1;
        polygon_marker.points.push_back(ros_point);
      }

      marker_array.markers.push_back(polygon_marker);
    }
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
  status_text += "\nAlways Pass: " + std::to_string(classification.kept_objects.size());
  status_text += "\nWould Filter: " + std::to_string(classification.would_be_removed.size());
  status_text += "\nFiltered: " + std::to_string(classification.removed_objects.size());

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

double getMinDistanceToPath(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const autoware_planning_msgs::msg::Trajectory & path)
{
  if (path.points.empty()) {
    return std::numeric_limits<double>::max();
  }

  // Convert object to polygon using autoware_universe_utils
  const auto object_polygon = autoware::universe_utils::toPolygon2d(object);

  // Convert trajectory to linestring for distance calculation
  autoware::universe_utils::LineString2d trajectory_line;
  for (const auto & point : path.points) {
    autoware::universe_utils::Point2d bg_point;
    bg_point.x() = point.pose.position.x;
    bg_point.y() = point.pose.position.y;
    trajectory_line.push_back(bg_point);
  }

  // Calculate minimum distance between object polygon and trajectory line using boost::geometry
  double min_distance = std::numeric_limits<double>::max();
  for (const auto & vertex : object_polygon.outer()) {
    const double distance = boost::geometry::distance(vertex, trajectory_line);
    min_distance = std::min(min_distance, distance);
  }

  return min_distance;
}

bool shouldIgnoreObject(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const std::vector<std::string> & ignore_object_classes)
{
  if (ignore_object_classes.empty()) {
    return false;  // No classes to ignore
  }

  const uint8_t object_label = getMostProbableLabel(object);
  const std::string label_string = labelToString(object_label);

  // Check if the object's label is in the ignore list
  return std::find(ignore_object_classes.begin(), ignore_object_classes.end(), label_string) !=
         ignore_object_classes.end();
}

uint8_t getMostProbableLabel(const autoware_perception_msgs::msg::PredictedObject & object)
{
  if (object.classification.empty()) {
    return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
  }

  // Find the classification with the highest probability
  double highest_probability = 0.0;
  uint8_t most_probable_label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;

  for (const auto & classification : object.classification) {
    if (classification.probability > highest_probability) {
      highest_probability = classification.probability;
      most_probable_label = classification.label;
    }
  }

  return most_probable_label;
}

std::string labelToString(uint8_t label)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  switch (label) {
    case ObjectClassification::UNKNOWN:
      return "UNKNOWN";
    case ObjectClassification::CAR:
      return "CAR";
    case ObjectClassification::TRUCK:
      return "TRUCK";
    case ObjectClassification::BUS:
      return "BUS";
    case ObjectClassification::TRAILER:
      return "TRAILER";
    case ObjectClassification::MOTORCYCLE:
      return "MOTORCYCLE";
    case ObjectClassification::BICYCLE:
      return "BICYCLE";
    case ObjectClassification::PEDESTRIAN:
      return "PEDESTRIAN";
    default:
      return "UNKNOWN";
  }
}

uint8_t stringToLabel(const std::string & label_string)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  if (label_string == "UNKNOWN") return ObjectClassification::UNKNOWN;
  if (label_string == "CAR") return ObjectClassification::CAR;
  if (label_string == "TRUCK") return ObjectClassification::TRUCK;
  if (label_string == "BUS") return ObjectClassification::BUS;
  if (label_string == "TRAILER") return ObjectClassification::TRAILER;
  if (label_string == "MOTORCYCLE") return ObjectClassification::MOTORCYCLE;
  if (label_string == "BICYCLE") return ObjectClassification::BICYCLE;
  if (label_string == "PEDESTRIAN") return ObjectClassification::PEDESTRIAN;
  return ObjectClassification::UNKNOWN;
}

std::optional<geometry_msgs::msg::Pose> getEgoPose(const tf2_ros::Buffer & tf_buffer)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer.lookupTransform(
      "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    return std::optional<geometry_msgs::msg::Pose>{};
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.transform.translation.x;
  pose.position.y = transform.transform.translation.y;
  pose.position.z = transform.transform.translation.z;
  pose.orientation = transform.transform.rotation;

  return std::optional<geometry_msgs::msg::Pose>{pose};
}

std::vector<autoware::universe_utils::Polygon2d> createTrajectoryPolygons(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & traj_points,
  const double lat_margin)
{
  std::vector<autoware::universe_utils::Polygon2d> output_polygons;

  if (traj_points.empty()) {
    return output_polygons;
  }

  autoware::universe_utils::Polygon2d accumulated_polygon;

  for (size_t i = 0; i < traj_points.size(); ++i) {
    const auto & pose = traj_points.at(i).pose;

    // Create left and right offset points from trajectory point
    const auto left_point =
      autoware::universe_utils::calcOffsetPose(pose, 0.0, lat_margin, 0.0).position;
    const auto right_point =
      autoware::universe_utils::calcOffsetPose(pose, 0.0, -lat_margin, 0.0).position;

    // Create current step polygon (rectangle-like shape)
    autoware::universe_utils::Polygon2d current_polygon;
    boost::geometry::append(
      current_polygon, autoware::universe_utils::Point2d(left_point.x, left_point.y));
    boost::geometry::append(
      current_polygon, autoware::universe_utils::Point2d(right_point.x, right_point.y));
    boost::geometry::append(
      current_polygon,
      autoware::universe_utils::Point2d(left_point.x, left_point.y));  // Close polygon

    // Add current polygon points to accumulated polygon
    for (const auto & point : current_polygon.outer()) {
      boost::geometry::append(accumulated_polygon, point);
    }

    // Calculate convex hull of accumulated polygon
    autoware::universe_utils::Polygon2d hull_polygon;
    boost::geometry::convex_hull(accumulated_polygon, hull_polygon);
    boost::geometry::correct(hull_polygon);

    output_polygons.push_back(hull_polygon);

    // Update accumulated polygon for next iteration (keep only current polygon for next step)
    accumulated_polygon = std::move(current_polygon);
  }

  return output_polygons;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filterByTrajectoryPolygonsCropBox(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr,
  const std::vector<autoware::universe_utils::Polygon2d> & traj_polygons,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points,
  const std::vector<autoware::universe_utils::Polygon2d> & crop_box_polygons,
  const double height_margin, const bool keep_inside)
{
  if (traj_polygons.empty() || trajectory_points.empty() || crop_box_polygons.empty()) {
    return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  }

  // Use pre-calculated crop box polygon for XY bounds
  const auto & crop_box = crop_box_polygons.front();
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  for (const auto & point : crop_box.outer()) {
    x_min = std::min(x_min, point.x());
    x_max = std::max(x_max, point.x());
    y_min = std::min(y_min, point.y());
    y_max = std::max(y_max, point.y());
  }

  // Calculate Z bounds from trajectory points
  double lowest_traj_height = std::numeric_limits<double>::max();
  double highest_traj_height = std::numeric_limits<double>::lowest();
  for (const auto & trajectory_point : trajectory_points) {
    lowest_traj_height = std::min(lowest_traj_height, trajectory_point.pose.position.z);
    highest_traj_height = std::max(highest_traj_height, trajectory_point.pose.position.z);
  }

  // Apply filter with keep_inside control
  auto filtered_pointcloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (const auto & point : input_pointcloud_ptr->points) {
    // Check if point is inside crop box bounds
    bool is_inside_crop_box =
      (point.x >= x_min && point.x <= x_max && point.y >= y_min && point.y <= y_max &&
       point.z >= (lowest_traj_height - height_margin) &&
       point.z <= (highest_traj_height + height_margin));

    // Keep points based on keep_inside parameter
    if ((keep_inside && is_inside_crop_box) || (!keep_inside && !is_inside_crop_box)) {
      filtered_pointcloud_ptr->points.push_back(point);
    }
  }

  filtered_pointcloud_ptr->width = filtered_pointcloud_ptr->points.size();
  filtered_pointcloud_ptr->height = 1;
  filtered_pointcloud_ptr->is_dense = true;

  return filtered_pointcloud_ptr;
}

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> transformTrajectoryToBaseLink(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & map_trajectory_points,
  const std::shared_ptr<autoware::universe_utils::TransformListener> & transform_listener)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> base_link_trajectory_points;

  if (map_trajectory_points.empty()) {
    return base_link_trajectory_points;
  }

  // Get transform from map to base_link
  const auto transform_opt = transform_listener->getTransform(
    "base_link", "map", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  if (!transform_opt) {
    std::cerr << "Failed to get transform from map to base_link" << std::endl;
    return base_link_trajectory_points;
  }

  const auto eigen_transform = tf2::transformToEigen(transform_opt->transform);

  // Transform trajectory points to base_link frame
  for (const auto & map_point : map_trajectory_points) {
    autoware_planning_msgs::msg::TrajectoryPoint base_link_point = map_point;

    // Transform position
    Eigen::Vector3d map_pos_3d(
      map_point.pose.position.x, map_point.pose.position.y, map_point.pose.position.z);
    Eigen::Vector3d base_link_pos_3d = eigen_transform * map_pos_3d;

    base_link_point.pose.position.x = base_link_pos_3d.x();
    base_link_point.pose.position.y = base_link_pos_3d.y();
    base_link_point.pose.position.z = base_link_pos_3d.z();

    // Transform orientation (quaternion)
    Eigen::Quaterniond map_quat(
      map_point.pose.orientation.w, map_point.pose.orientation.x, map_point.pose.orientation.y,
      map_point.pose.orientation.z);

    // Extract rotation part from transform
    Eigen::Matrix3d rotation_matrix = eigen_transform.rotation();
    Eigen::Quaterniond base_link_quat(rotation_matrix * map_quat.toRotationMatrix());

    base_link_point.pose.orientation.w = base_link_quat.w();
    base_link_point.pose.orientation.x = base_link_quat.x();
    base_link_point.pose.orientation.y = base_link_quat.y();
    base_link_point.pose.orientation.z = base_link_quat.z();

    base_link_trajectory_points.push_back(base_link_point);
  }

  return base_link_trajectory_points;
}

std::vector<autoware::universe_utils::Polygon2d> generateTrajectoryPolygons(
  const autoware_planning_msgs::msg::Trajectory & planning_trajectory, double max_filter_distance,
  const std::shared_ptr<autoware::universe_utils::TransformListener> & transform_listener)
{
  std::vector<autoware::universe_utils::Polygon2d> traj_polygons;

  if (planning_trajectory.points.empty()) {
    return traj_polygons;
  }

  // Transform trajectory points to base_link frame
  const auto base_link_trajectory_points =
    transformTrajectoryToBaseLink(planning_trajectory.points, transform_listener);

  if (base_link_trajectory_points.empty()) {
    return traj_polygons;
  }

  // Create polygons in base_link frame
  traj_polygons = createTrajectoryPolygons(base_link_trajectory_points, max_filter_distance);

  return traj_polygons;
}

std::vector<autoware::universe_utils::Polygon2d> generateCropBoxPolygons(
  const std::vector<autoware::universe_utils::Polygon2d> & traj_polygons)
{
  std::vector<autoware::universe_utils::Polygon2d> crop_box_polygons;

  if (traj_polygons.empty()) {
    return crop_box_polygons;
  }

  // Calculate XY bounding box from trajectory polygons
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  for (const auto & poly : traj_polygons) {
    for (const auto & point : poly.outer()) {
      x_min = std::min(x_min, point.x());
      x_max = std::max(x_max, point.x());
      y_min = std::min(y_min, point.y());
      y_max = std::max(y_max, point.y());
    }
  }

  // Create bounding box polygon
  autoware::universe_utils::Polygon2d bounding_polygon;
  boost::geometry::append(bounding_polygon, autoware::universe_utils::Point2d(x_min, y_min));
  boost::geometry::append(bounding_polygon, autoware::universe_utils::Point2d(x_max, y_min));
  boost::geometry::append(bounding_polygon, autoware::universe_utils::Point2d(x_max, y_max));
  boost::geometry::append(bounding_polygon, autoware::universe_utils::Point2d(x_min, y_max));
  boost::geometry::append(
    bounding_polygon, autoware::universe_utils::Point2d(x_min, y_min));  // Close polygon
  boost::geometry::correct(bounding_polygon);

  crop_box_polygons.push_back(bounding_polygon);

  return crop_box_polygons;
}

autoware::universe_utils::Polygon2d combineTrajectoryPolygons(
  const std::vector<autoware::universe_utils::Polygon2d> & polygons)
{
  autoware::universe_utils::Polygon2d combined_polygon;

  if (polygons.empty()) {
    return combined_polygon;
  }

  if (polygons.size() == 1) {
    return polygons[0];
  }

  // Use boost::geometry::union_ to combine all polygons
  autoware::universe_utils::MultiPolygon2d result_polygons;

  // Start with the first polygon
  autoware::universe_utils::MultiPolygon2d temp_polygons;
  temp_polygons.push_back(polygons[0]);

  // Union with each subsequent polygon
  for (size_t i = 1; i < polygons.size(); ++i) {
    autoware::universe_utils::MultiPolygon2d current_result;
    boost::geometry::union_(temp_polygons, polygons[i], current_result);
    temp_polygons = current_result;
  }

  // If we have a result, take the first (largest) polygon
  if (!temp_polygons.empty()) {
    combined_polygon = temp_polygons[0];
    boost::geometry::correct(combined_polygon);
  }

  return combined_polygon;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filterByMultiTrajectoryPolygon(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr,
  const std::vector<autoware::universe_utils::Polygon2d> & traj_polygons,
  autoware::universe_utils::TimeKeeper * time_keeper)
{
  auto ret_pointcloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  ret_pointcloud_ptr->header = input_pointcloud_ptr->header;

  // Define types for Boost.Geometry
  namespace bg = boost::geometry;
  namespace bgi = boost::geometry::index;
  using BoostPoint2D = bg::model::point<double, 2, bg::cs::cartesian>;
  using BoostValue = std::pair<BoostPoint2D, size_t>;  // point + index

  // Build R-tree from input points
  std::vector<BoostValue> rtree_data;
  rtree_data.reserve(input_pointcloud_ptr->points.size());

  {
    std::unique_ptr<autoware::universe_utils::ScopedTimeTrack> st_rtree_build;
    if (time_keeper) {
      st_rtree_build =
        std::make_unique<autoware::universe_utils::ScopedTimeTrack>("rtree_build", *time_keeper);
    }
    std::transform(
      input_pointcloud_ptr->points.begin(), input_pointcloud_ptr->points.end(),
      std::back_inserter(rtree_data), [i = 0](const pcl::PointXYZ & pt) mutable {
        return std::make_pair(BoostPoint2D(pt.x, pt.y), i++);
      });
  }

  bgi::rtree<BoostValue, bgi::quadratic<16>> rtree(rtree_data.begin(), rtree_data.end());

  std::unordered_set<size_t> selected_indices;

  {
    std::unique_ptr<autoware::universe_utils::ScopedTimeTrack> st_polygon_query;
    if (time_keeper) {
      st_polygon_query =
        std::make_unique<autoware::universe_utils::ScopedTimeTrack>("polygon_query", *time_keeper);
    }
    std::for_each(
      traj_polygons.begin(), traj_polygons.end(),
      [&](const autoware::universe_utils::Polygon2d & one_step_polygon) {
        bg::model::box<BoostPoint2D> bbox;
        bg::envelope(one_step_polygon, bbox);

        std::vector<BoostValue> result_s;
        rtree.query(bgi::intersects(bbox), std::back_inserter(result_s));

        for (const auto & val : result_s) {
          const BoostPoint2D & pt = val.first;
          if (bg::within(pt, one_step_polygon)) {
            selected_indices.insert(val.second);
          }
        }
      });
  }

  {
    std::unique_ptr<autoware::universe_utils::ScopedTimeTrack> st_result_build;
    if (time_keeper) {
      st_result_build =
        std::make_unique<autoware::universe_utils::ScopedTimeTrack>("result_build", *time_keeper);
    }
    ret_pointcloud_ptr->points.reserve(selected_indices.size());
    std::transform(
      selected_indices.begin(), selected_indices.end(),
      std::back_inserter(ret_pointcloud_ptr->points),
      [&](const size_t idx) { return input_pointcloud_ptr->points[idx]; });

    ret_pointcloud_ptr->width = ret_pointcloud_ptr->points.size();
    ret_pointcloud_ptr->height = 1;
    ret_pointcloud_ptr->is_dense = true;
  }

  return ret_pointcloud_ptr;
}

std::vector<autoware::universe_utils::Polygon2d> createDifferencePolygons(
  const std::vector<autoware::universe_utils::Polygon2d> & max_polygons,
  const std::vector<autoware::universe_utils::Polygon2d> & min_polygons)
{
  std::vector<autoware::universe_utils::Polygon2d> difference_polygons;

  if (max_polygons.empty()) {
    return difference_polygons;
  }

  // If min_polygons is empty, return max_polygons as is
  if (min_polygons.empty()) {
    return max_polygons;
  }

  // Combine all max_polygons into a single polygon using union
  autoware::universe_utils::Polygon2d combined_max_polygon;
  if (max_polygons.size() == 1) {
    combined_max_polygon = max_polygons[0];
  } else {
    combined_max_polygon = combineTrajectoryPolygons(max_polygons);
  }

  // Combine all min_polygons into a single polygon using union
  autoware::universe_utils::Polygon2d combined_min_polygon;
  if (min_polygons.size() == 1) {
    combined_min_polygon = min_polygons[0];
  } else {
    combined_min_polygon = combineTrajectoryPolygons(min_polygons);
  }

  // Perform difference operation using boost::geometry::difference
  autoware::universe_utils::MultiPolygon2d result_polygons;
  boost::geometry::difference(combined_max_polygon, combined_min_polygon, result_polygons);

  // Convert MultiPolygon2d to vector of Polygon2d
  for (const auto & polygon : result_polygons) {
    autoware::universe_utils::Polygon2d corrected_polygon = polygon;
    boost::geometry::correct(corrected_polygon);
    difference_polygons.push_back(corrected_polygon);
  }

  return difference_polygons;
}

autoware_planning_msgs::msg::Trajectory cutTrajectoryByFilteringDistance(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & ego_pose, const double filtering_start_distance,
  const double filtering_end_distance)
{
  // Initialize cut trajectory with input trajectory
  auto cut_trajectory = trajectory;

  // Find nearest segment index to ego pose
  const auto nearest_segment_idx =
    autoware::motion_utils::findNearestSegmentIndex(trajectory.points, ego_pose.position);

  if (!nearest_segment_idx) {
    // If nearest segment is not found, return original trajectory
    return cut_trajectory;
  }

  // Calculate distance from trajectory start to ego pose
  const double ego_distance_from_start =
    autoware::motion_utils::calcSignedArcLength(trajectory.points, 0, nearest_segment_idx);

  // Calculate start and end distances for cutting
  const double start_distance = ego_distance_from_start + filtering_start_distance;
  const double end_distance = ego_distance_from_start + filtering_end_distance;

  // Find indices for cutting by converting distance to pose first
  const auto start_pose =
    autoware::motion_utils::calcInterpolatedPose(trajectory.points, start_distance);
  const auto end_pose =
    autoware::motion_utils::calcInterpolatedPose(trajectory.points, end_distance);

  const auto start_idx =
    autoware::motion_utils::findNearestIndex(trajectory.points, start_pose.position);
  const auto end_idx =
    autoware::motion_utils::findNearestIndex(trajectory.points, end_pose.position);

  if (start_idx < end_idx) {
    // Create cut trajectory
    cut_trajectory.points.clear();
    cut_trajectory.points.insert(
      cut_trajectory.points.end(), trajectory.points.begin() + start_idx,
      trajectory.points.begin() + end_idx + 1);
  }

  return cut_trajectory;
}

}  // namespace autoware::perception_filter
