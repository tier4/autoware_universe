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

#ifndef AUTOWARE__PERCEPTION_FILTER__PERCEPTION_FILTER_CORE_HPP_
#define AUTOWARE__PERCEPTION_FILTER__PERCEPTION_FILTER_CORE_HPP_

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/ros/transform_listener.hpp>
#include <autoware/universe_utils/system/time_keeper.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <boost/geometry.hpp>

// PCL includes for point cloud processing
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

// Forward declaration for TF2
namespace tf2_ros
{
class Buffer;
}

namespace autoware::perception_filter
{

/**
 * @brief RTC approval filtering range polygon structure
 * @details Defines a polygon-based filtering area that is activated when RTC is approved
 */
struct FilteringPolygon
{
  std::vector<autoware::universe_utils::Polygon2d> max_polygon;  ///< Filtering range polygons
  std::vector<autoware::universe_utils::Polygon2d> min_polygon;  ///< Safety range polygons
  autoware_planning_msgs::msg::Trajectory trajectory;            ///< Associated trajectory
  geometry_msgs::msg::Pose ego_pose;    ///< Ego vehicle pose at polygon creation
  geometry_msgs::msg::Pose start_pose;  ///< Start pose at filtering_start_distance behind ego
  geometry_msgs::msg::Pose end_pose;    ///< End pose at filtering_end_distance ahead of ego
  bool is_active;                       ///< Whether the polygon is currently active
};

/**
 * @brief Object classification structure for filtering decisions
 * @details Categorizes objects based on filtering behavior and RTC status
 */
struct ObjectClassification
{
  std::vector<autoware_perception_msgs::msg::PredictedObject>
    kept_objects;  ///< Objects that are always kept (never filtered)
  std::vector<autoware_perception_msgs::msg::PredictedObject>
    would_be_removed;  ///< Objects that would be removed if RTC is approved
  std::vector<autoware_perception_msgs::msg::PredictedObject>
    removed_objects;  ///< Objects that are currently removed/filtered
};

/**
 * @brief Information about filtered points for planning factors
 * @details Stores point data for filtered pointcloud points
 */
struct FilteredPointInfo
{
  geometry_msgs::msg::Point point;  ///< Filtered point coordinates
};

/**
 * @brief Structure to hold common point cloud processing results
 * @details Contains transformed point cloud, polygon-filtered indices, and distance calculations
 */
struct PointCloudProcessingResult
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud;  ///< Point cloud transformed to map frame
  pcl::PointIndices::Ptr polygon_inside_indices;  ///< Indices of points inside filtering polygon
  std::vector<double>
    distances_to_path;  ///< Distance to planning path for each point inside polygon
  bool success;         ///< Whether processing completed successfully

  PointCloudProcessingResult() : success(false)
  {
    transformed_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    polygon_inside_indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
  }
};

// ========== Core Filtering Functions ==========

/**
 * @brief Classify objects within radius for filtering decisions
 * @param input_objects Objects to classify
 * @param planning_trajectory Planning trajectory for distance calculation
 * @param ego_pose Current ego vehicle pose
 * @param rtc_is_registered Whether RTC interface is registered
 * @param frozen_filter_object_ids Object IDs frozen at RTC approval time
 * @param max_filter_distance Maximum distance from path to filter objects [m]
 * @param object_classification_radius Radius for object classification [m]
 * @param ignore_object_classes Object classes to ignore during filtering
 * @return Object classification result
 */
ObjectClassification classifyObjectsWithinRadius(
  const autoware_perception_msgs::msg::PredictedObjects & input_objects,
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & planning_trajectory,
  const geometry_msgs::msg::Pose & ego_pose, bool rtc_is_registered,
  const std::set<std::array<uint8_t, 16>> & frozen_filter_object_ids, double max_filter_distance,
  double object_classification_radius, const std::vector<std::string> & ignore_object_classes);

/**
 * @brief Create a polygon around trajectory segment
 * @param trajectory Input trajectory
 * @param start_distance Start distance along path [m]
 * @param end_distance End distance along path [m]
 * @param width Half-width of polygon [m]
 * @return Generated polygon
 */
autoware::universe_utils::Polygon2d createPathPolygon(
  const autoware_planning_msgs::msg::Trajectory & trajectory, double start_distance,
  double end_distance, double width);

/**
 * @brief Process point cloud with common filtering operations
 * @param input_pointcloud Input point cloud to process
 * @param filtering_polygon Polygon to filter points within
 * @param planning_trajectory Planning trajectory for distance calculation
 * @param tf_buffer TF buffer for coordinate transformation
 * @param time_keeper Time keeper for detailed performance measurement
 * @return PointCloudProcessingResult containing transformed cloud, filtered indices, and distances
 * @details Performs coordinate transformation, bounding box filtering, polygon filtering, and
 * distance calculation
 */
PointCloudProcessingResult processPointCloudCommon(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud,
  const autoware::universe_utils::Polygon2d & filtering_polygon,
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & planning_trajectory,
  const std::shared_ptr<autoware::universe_utils::TransformListener> & transform_listener,
  autoware::universe_utils::TimeKeeper & time_keeper);

}  // namespace autoware::perception_filter

#endif  // AUTOWARE__PERCEPTION_FILTER__PERCEPTION_FILTER_CORE_HPP_
