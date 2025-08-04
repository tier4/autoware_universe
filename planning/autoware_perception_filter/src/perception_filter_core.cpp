// Copyright 2024 TIER IV, Inc.
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

#include "autoware/perception_filter/perception_filter_core.hpp"

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/detail/envelope/interface.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace autoware::perception_filter
{

ObjectClassification classifyObjectsWithinRadius(
  const autoware_perception_msgs::msg::PredictedObjects & input_objects,
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & planning_trajectory,
  const geometry_msgs::msg::Pose & ego_pose, bool rtc_is_registered,
  const std::set<std::array<uint8_t, 16>> & frozen_filter_object_ids, double max_filter_distance,
  double object_classification_radius, const std::vector<std::string> & ignore_object_classes)
{
  ObjectClassification classification;

  // If frozen list is empty and RTC not registered, pass through all objects
  if (frozen_filter_object_ids.empty() && !rtc_is_registered) {
    classification.pass_through_always = input_objects.objects;
    return classification;
  }

  // If frozen list is not empty and RTC not registered, keep filtered objects
  if (!frozen_filter_object_ids.empty() && !rtc_is_registered) {
    for (const auto & object : input_objects.objects) {
      std::array<uint8_t, 16> uuid_array;
      std::copy(object.object_id.uuid.begin(), object.object_id.uuid.end(), uuid_array.begin());

      if (frozen_filter_object_ids.find(uuid_array) != frozen_filter_object_ids.end()) {
        classification.currently_filtered.push_back(object);
      } else {
        classification.pass_through_always.push_back(object);
      }
    }
    return classification;
  }

  if (rtc_is_registered && planning_trajectory) {
    for (const auto & object : input_objects.objects) {
      std::array<uint8_t, 16> uuid_array;
      std::copy(object.object_id.uuid.begin(), object.object_id.uuid.end(), uuid_array.begin());

      // Check if object is in frozen filter list
      if (frozen_filter_object_ids.find(uuid_array) != frozen_filter_object_ids.end()) {
        classification.currently_filtered.push_back(object);
        continue;
      }

      // Check if object should be ignored
      if (!shouldIgnoreObject(object, ignore_object_classes)) {
        classification.pass_through_always.push_back(object);
        continue;
      }

      // Check if object is within classification radius
      const double distance_from_ego = getDistanceFromEgo(object, ego_pose);
      if (distance_from_ego <= object_classification_radius) {
        const double distance_to_path = getMinDistanceToPath(object, *planning_trajectory);
        const bool would_be_filtered = distance_to_path <= max_filter_distance;

        if (would_be_filtered) {
          classification.pass_through_would_filter.push_back(object);
        } else {
          classification.pass_through_always.push_back(object);
        }
      } else {
        classification.pass_through_always.push_back(object);
      }
    }
  }

  return classification;
}

autoware::universe_utils::Polygon2d createPathPolygon(
  const autoware_planning_msgs::msg::Trajectory & trajectory, double start_distance,
  double end_distance, double width)
{
  autoware::universe_utils::Polygon2d polygon;

  if (trajectory.points.empty()) {
    return polygon;
  }

  // Calculate cumulative distances to find start and end indices
  double cumulative_distance = 0.0;
  size_t start_index = 0;
  size_t end_index = trajectory.points.size() - 1;

  for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
    const auto & current_point = trajectory.points[i].pose.position;
    const auto & next_point = trajectory.points[i + 1].pose.position;

    const double dx = next_point.x - current_point.x;
    const double dy = next_point.y - current_point.y;
    const double segment_distance = std::sqrt(dx * dx + dy * dy);

    if (
      cumulative_distance <= start_distance &&
      cumulative_distance + segment_distance > start_distance) {
      start_index = i;
    }

    if (
      cumulative_distance <= end_distance &&
      cumulative_distance + segment_distance > end_distance) {
      end_index = i + 1;
      break;
    }

    cumulative_distance += segment_distance;
  }

  // Create left and right offset points for the polygon
  autoware::universe_utils::LineString2d left_points;
  autoware::universe_utils::LineString2d right_points;

  for (size_t i = start_index; i <= end_index && i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points[i].pose.position;

    // Calculate perpendicular direction (normal to trajectory)
    autoware::universe_utils::Point2d normal_dir(0.0, 0.0);

    if (i > 0 && i < trajectory.points.size() - 1) {
      // Use direction from previous to next point
      const auto & prev_point = trajectory.points[i - 1].pose.position;
      const auto & next_point = trajectory.points[i + 1].pose.position;

      const double dx = next_point.x - prev_point.x;
      const double dy = next_point.y - prev_point.y;
      const double length = std::sqrt(dx * dx + dy * dy);

      if (length > 1e-6) {
        // Normal direction (perpendicular to trajectory)
        normal_dir.x() = -dy / length;
        normal_dir.y() = dx / length;
      }
    } else if (i > 0) {
      // Use direction from previous point
      const auto & prev_point = trajectory.points[i - 1].pose.position;
      const double dx = point.x - prev_point.x;
      const double dy = point.y - prev_point.y;
      const double length = std::sqrt(dx * dx + dy * dy);

      if (length > 1e-6) {
        normal_dir.x() = -dy / length;
        normal_dir.y() = dx / length;
      }
    } else if (i < trajectory.points.size() - 1) {
      // Use direction to next point
      const auto & next_point = trajectory.points[i + 1].pose.position;
      const double dx = next_point.x - point.x;
      const double dy = next_point.y - point.y;
      const double length = std::sqrt(dx * dx + dy * dy);

      if (length > 1e-6) {
        normal_dir.x() = -dy / length;
        normal_dir.y() = dx / length;
      }
    }

    // Create left and right offset points
    autoware::universe_utils::Point2d left_point(
      point.x + normal_dir.x() * width, point.y + normal_dir.y() * width);
    autoware::universe_utils::Point2d right_point(
      point.x - normal_dir.x() * width, point.y - normal_dir.y() * width);

    left_points.push_back(left_point);
    right_points.push_back(right_point);
  }

  // Build polygon from left and right points
  for (const auto & point : left_points) {
    polygon.outer().push_back(point);
  }

  // Add right points in reverse order to close the polygon
  for (auto it = right_points.rbegin(); it != right_points.rend(); ++it) {
    polygon.outer().push_back(*it);
  }

  // Close polygon if not already closed
  if (
    !polygon.outer().empty() && (polygon.outer().front().x() != polygon.outer().back().x() ||
                                 polygon.outer().front().y() != polygon.outer().back().y())) {
    polygon.outer().push_back(polygon.outer().front());
  }

  return polygon;
}

PointCloudProcessingResult processPointCloudCommon(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud,
  const autoware::universe_utils::Polygon2d & filtering_polygon,
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & planning_trajectory,
  const tf2_ros::Buffer & tf_buffer)
{
  PointCloudProcessingResult result;

  if (input_pointcloud.data.empty() || !planning_trajectory) {
    return result;
  }

  // Convert ROS PointCloud2 to PCL format
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr polygon_points(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(input_pointcloud, *input_cloud);

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer.lookupTransform(
      "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    // Return failure result if transform lookup fails
    return result;
  }

  // Transform the pointcloud to the map frame
  const auto eigen_transform = tf2::transformToEigen(transform.transform).cast<float>();
  pcl::transformPointCloud(*input_cloud, *result.transformed_cloud, eigen_transform);

  const auto bbox =
    boost::geometry::return_envelope<autoware::universe_utils::Box2d>(filtering_polygon.outer());

  // Rough filter on the X/Y axis using indices-based approach
  pcl::IndicesPtr x_filtered_indices(new pcl::Indices());
  pcl::PassThrough<pcl::PointXYZ> passthrough_filter;
  passthrough_filter.setInputCloud(result.transformed_cloud);
  passthrough_filter.setFilterFieldName("x");
  passthrough_filter.setFilterLimits(
    static_cast<float>(bbox.min_corner().x()), static_cast<float>(bbox.max_corner().x()));
  passthrough_filter.filter(*x_filtered_indices);

  pcl::IndicesPtr xy_filtered_indices(new pcl::Indices());
  passthrough_filter.setInputCloud(result.transformed_cloud);
  passthrough_filter.setFilterFieldName("y");
  passthrough_filter.setIndices(x_filtered_indices);
  passthrough_filter.setFilterLimits(
    static_cast<float>(bbox.min_corner().y()), static_cast<float>(bbox.max_corner().y()));
  passthrough_filter.filter(*xy_filtered_indices);

  // Prepare polygon hull for CropHull filter
  std::vector<pcl::Vertices> polygon_vertices;
  polygon_vertices.emplace_back();
  auto polygon_i = 0;
  for (const auto & p : filtering_polygon.outer()) {
    polygon_vertices.back().vertices.push_back(polygon_i++);
    polygon_points->push_back({static_cast<float>(p.x()), static_cast<float>(p.y()), 0.0});
  }

  // Use CropHull for polygon-based filtering
  pcl::CropHull<pcl::PointXYZ> crop_hull;
  crop_hull.setInputCloud(result.transformed_cloud);
  crop_hull.setIndices(xy_filtered_indices);
  crop_hull.setHullCloud(polygon_points);
  crop_hull.setHullIndices(polygon_vertices);
  crop_hull.setDim(2);  // 2D polygon filtering
  crop_hull.filter(result.polygon_inside_indices->indices);

  // Calculate distances to path for all points inside polygon
  result.distances_to_path.reserve(result.polygon_inside_indices->indices.size());
  for (const auto i : result.polygon_inside_indices->indices) {
    const auto & point = result.transformed_cloud->points[i];
    geometry_msgs::msg::Point ros_point;
    ros_point.x = point.x;
    ros_point.y = point.y;
    ros_point.z = point.z;

    const double distance_to_path = getMinDistanceToPath(ros_point, *planning_trajectory);
    result.distances_to_path.push_back(distance_to_path);
  }

  result.success = true;
  return result;
}

double getMinDistanceToPath(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const autoware_planning_msgs::msg::Trajectory & path)
{
  if (path.points.empty()) {
    return std::numeric_limits<double>::max();
  }

  const auto object_polygon = autoware::universe_utils::toPolygon2d(object);
  double min_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path.points.size() - 1; ++i) {
    const auto & current_pose = path.points[i].pose;
    const auto & next_pose = path.points[i + 1].pose;

    autoware::universe_utils::LineString2d line_segment;
    line_segment.push_back(
      autoware::universe_utils::Point2d(current_pose.position.x, current_pose.position.y));
    line_segment.push_back(
      autoware::universe_utils::Point2d(next_pose.position.x, next_pose.position.y));

    double distance;
    if (boost::geometry::distance(line_segment[0], line_segment[1]) < 1e-6) {
      // Calculate distance to point
      distance = boost::geometry::distance(
        object_polygon,
        autoware::universe_utils::Point2d(current_pose.position.x, current_pose.position.y));
    } else {
      // Calculate distance to line segment
      distance = boost::geometry::distance(object_polygon, line_segment);
    }
    min_distance = std::min(min_distance, distance);
  }

  // Also check distance to the last point
  if (!path.points.empty()) {
    const auto & last_pose = path.points.back().pose;
    const double distance = boost::geometry::distance(
      object_polygon,
      autoware::universe_utils::Point2d(last_pose.position.x, last_pose.position.y));
    min_distance = std::min(min_distance, distance);
  }

  return min_distance;
}

double getMinDistanceToPath(
  const geometry_msgs::msg::Point & point, const autoware_planning_msgs::msg::Trajectory & path)
{
  double min_distance = std::numeric_limits<double>::max();

  for (const auto & path_point : path.points) {
    const double dx = point.x - path_point.pose.position.x;
    const double dy = point.y - path_point.pose.position.y;
    const double distance = std::sqrt(dx * dx + dy * dy);
    min_distance = std::min(min_distance, distance);
  }

  return min_distance;
}

double getDistanceFromEgo(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const geometry_msgs::msg::Pose & ego_pose)
{
  const auto & object_pos = object.kinematics.initial_pose_with_covariance.pose.position;

  const double dx = object_pos.x - ego_pose.position.x;
  const double dy = object_pos.y - ego_pose.position.y;

  return std::sqrt(dx * dx + dy * dy);
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

}  // namespace autoware::perception_filter
