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

#ifndef AUTOWARE__PERCEPTION_FILTER__PERCEPTION_FILTER_UTILS_HPP_
#define AUTOWARE__PERCEPTION_FILTER__PERCEPTION_FILTER_UTILS_HPP_

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/transform_listener.hpp>
#include <autoware/universe_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::perception_filter
{

// Forward declarations
struct FilteredPointInfo;
struct ObjectClassification;

/**
 * @brief Create planning factors for filtered objects and points
 * @param classification Object classification result
 * @param would_be_filtered_point_cloud Point cloud that would be filtered when RTC is approved
 * @param planning_trajectory Planning trajectory for control points
 * @return Planning factor array
 */
autoware_internal_planning_msgs::msg::PlanningFactorArray createPlanningFactors(
  const ObjectClassification & classification,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & would_be_filtered_point_cloud,
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & planning_trajectory);

/**
 * @brief Calculate minimum distance from predicted object polygon to trajectory
 * @param object Predicted object to calculate distance from
 * @param path Trajectory to calculate distance to
 * @return Minimum distance from object polygon to trajectory [m]
 */
double getMinDistanceToPath(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const autoware_planning_msgs::msg::Trajectory & path);

// ========== Object Classification Helper Functions ==========

/**
 * @brief Check if object should be ignored based on class
 * @param object Object to check
 * @param ignore_object_classes Object classes to ignore during filtering
 * @return True if object should be ignored, false otherwise
 */
bool shouldIgnoreObject(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const std::vector<std::string> & ignore_object_classes);

/**
 * @brief Get most probable classification label for object
 * @param object Object to get label for
 * @return Most probable classification label
 */
uint8_t getMostProbableLabel(const autoware_perception_msgs::msg::PredictedObject & object);

/**
 * @brief Convert classification label to string
 * @param label Classification label
 * @return String representation of label
 */
std::string labelToString(uint8_t label);

/**
 * @brief Convert string to classification label
 * @param label_string String representation of label
 * @return Classification label
 */
uint8_t stringToLabel(const std::string & label_string);

/**
 * @brief Get ego vehicle pose from TF buffer
 * @param tf_buffer TF buffer for getting transform
 * @return Optional ego pose, empty if transform is not available
 */
std::optional<geometry_msgs::msg::Pose> getEgoPose(const tf2_ros::Buffer & tf_buffer);

/**
 * @brief Create filtering polygons from trajectory points with lateral margin
 * @param traj_points Trajectory points to create polygons from
 * @param lat_margin Lateral margin for polygon creation [m]
 * @return Vector of filtering polygons for each trajectory point
 */
std::vector<autoware::universe_utils::Polygon2d> createTrajectoryPolygons(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & traj_points,
  const double lat_margin);

/**
 * @brief Filter pointcloud using trajectory polygons with crop box filtering
 * @param input_pointcloud_ptr Input pointcloud to filter
 * @param traj_polygons Trajectory polygons for filtering
 * @param trajectory_points Trajectory points for height calculation
 * @param crop_box_polygons Pre-calculated crop box polygons for XY bounds
 * @param height_margin Height margin for Z-axis filtering [m]
 * @param keep_inside If true, keep points inside crop box; if false, keep points outside crop box
 * @return Filtered pointcloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr filterByTrajectoryPolygonsCropBox(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr,
  const std::vector<autoware::universe_utils::Polygon2d> & traj_polygons,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points,
  const std::vector<autoware::universe_utils::Polygon2d> & crop_box_polygons,
  const double height_margin = 0.0, const bool keep_inside = true);

/**
 * @brief Transform trajectory points from map frame to base_link frame
 * @param map_trajectory_points Trajectory points in map frame
 * @param transform_listener Transform listener for coordinate transformation
 * @return Transformed trajectory points in base_link frame
 */
std::vector<autoware_planning_msgs::msg::TrajectoryPoint> transformTrajectoryToBaseLink(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & map_trajectory_points,
  const std::shared_ptr<autoware::universe_utils::TransformListener> & transform_listener);

/**
 * @brief Generate trajectory polygons in base_link frame for planning factors
 * @param planning_trajectory Planning trajectory in map frame
 * @param max_filter_distance Maximum filter distance for polygon creation
 * @param transform_listener Transform listener for coordinate transformation
 * @return Vector of trajectory polygons in base_link frame
 */
std::vector<autoware::universe_utils::Polygon2d> generateTrajectoryPolygons(
  const autoware_planning_msgs::msg::Trajectory & planning_trajectory, double max_filter_distance,
  const std::shared_ptr<autoware::universe_utils::TransformListener> & transform_listener);

/**
 * @brief Generate crop box polygons from trajectory polygons
 * @param traj_polygons Trajectory polygons to create crop box from
 * @return Vector of crop box polygons (bounding boxes)
 */
std::vector<autoware::universe_utils::Polygon2d> generateCropBoxPolygons(
  const std::vector<autoware::universe_utils::Polygon2d> & traj_polygons);

/**
 * @brief Combine multiple trajectory polygons into a single polygon using union operation
 * @param polygons Vector of polygons to combine
 * @return Combined polygon (union of all input polygons)
 */
autoware::universe_utils::Polygon2d combineTrajectoryPolygons(
  const std::vector<autoware::universe_utils::Polygon2d> & polygons);

/**
 * @brief Filter pointcloud using multiple trajectory polygons with R-tree optimization
 * @param input_pointcloud_ptr Input pointcloud to filter
 * @param traj_polygons Vector of trajectory polygons for filtering
 * @param time_keeper Optional time keeper for performance measurement
 * @return Pair of filtered pointcloud containing points inside any of the polygons and their
 * indices
 */
std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointIndices::Ptr>
filterByMultiTrajectoryPolygon(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr,
  const std::vector<autoware::universe_utils::Polygon2d> & traj_polygons,
  autoware::universe_utils::TimeKeeper * time_keeper = nullptr);

/**
 * @brief Create difference polygon by subtracting min_polygons from max_polygons
 * @param max_polygons Vector of larger polygons (outer boundary)
 * @param min_polygons Vector of smaller polygons to subtract (inner boundary)
 * @return Vector of difference polygons (max_polygons - min_polygons)
 */
std::vector<autoware::universe_utils::Polygon2d> createDifferencePolygons(
  const std::vector<autoware::universe_utils::Polygon2d> & max_polygons,
  const std::vector<autoware::universe_utils::Polygon2d> & min_polygons);

/**
 * @brief Cut trajectory by filtering distances from ego pose
 * @param trajectory Input trajectory to cut
 * @param ego_pose Ego vehicle pose in map frame
 * @param filtering_start_distance Start distance from ego pose for cutting [m]
 * @param filtering_end_distance End distance from ego pose for cutting [m]
 * @return Cut trajectory containing only the specified distance range
 */
autoware_planning_msgs::msg::Trajectory cutTrajectoryByFilteringDistance(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & ego_pose, const double filtering_start_distance,
  const double filtering_end_distance);

/**
 * @brief Cut trajectory between specified start and end poses
 * @param trajectory Input trajectory to cut
 * @param start_pose Start pose for cutting
 * @param end_pose End pose for cutting
 * @return Cut trajectory containing only the points between start_pose and end_pose
 */
autoware_planning_msgs::msg::Trajectory cutTrajectoryByPoses(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & end_pose);

/**
 * @brief Transform polygons from base_link frame to map frame
 * @param base_link_polygons Vector of polygons in base_link frame
 * @param transform_listener Transform listener for coordinate transformation
 * @return Vector of transformed polygons in map frame
 */
std::vector<autoware::universe_utils::Polygon2d> transformPolygonsToMap(
  const std::vector<autoware::universe_utils::Polygon2d> & base_link_polygons,
  const std::shared_ptr<autoware::universe_utils::TransformListener> & transform_listener);

/**
 * @brief Get Eigen transform matrix between two frames
 * @param transform_listener Transform listener for coordinate transformation
 * @param target_frame Target frame name
 * @param source_frame Source frame name
 * @return Optional Eigen transform matrix, empty if transform is not available
 */
std::optional<Eigen::Matrix4d> getEigenTransform(
  const std::shared_ptr<autoware::universe_utils::TransformListener> & transform_listener,
  const std::string & target_frame, const std::string & source_frame);

}  // namespace autoware::perception_filter

#endif  // AUTOWARE__PERCEPTION_FILTER__PERCEPTION_FILTER_UTILS_HPP_
