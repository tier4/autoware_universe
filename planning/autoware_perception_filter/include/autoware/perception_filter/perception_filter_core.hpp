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

#ifndef AUTOWARE__PERCEPTION_FILTER__PERCEPTION_FILTER_CORE_HPP_
#define AUTOWARE__PERCEPTION_FILTER__PERCEPTION_FILTER_CORE_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <boost/geometry.hpp>

#include <array>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace autoware::perception_filter
{

/**
 * @brief RTC approval filtering range polygon structure
 * @details Defines a polygon-based filtering area that is activated when RTC is approved
 */
struct FilteringPolygon
{
  autoware_utils_geometry::Polygon2d polygon;  ///< Filtering range polygon
  double start_distance_along_path;            ///< Start distance along the path [m]
  double end_distance_along_path;              ///< End distance along the path [m]
  bool is_active;                              ///< Whether the polygon is currently active
};

/**
 * @brief Object classification structure for filtering decisions
 * @details Categorizes objects based on filtering behavior and RTC status
 */
struct ObjectClassification
{
  std::vector<autoware_perception_msgs::msg::PredictedObject>
    pass_through_always;  ///< Objects that always pass through
  std::vector<autoware_perception_msgs::msg::PredictedObject>
    pass_through_would_filter;  ///< Objects passing now but would be filtered if RTC approved
  std::vector<autoware_perception_msgs::msg::PredictedObject>
    currently_filtered;  ///< Objects currently being filtered
};

/**
 * @brief Information about filtered points for planning factors
 * @details Stores point data and distance information for filtered pointcloud points
 */
struct FilteredPointInfo
{
  geometry_msgs::msg::Point point;  ///< Filtered point coordinates
  double distance_to_path;          ///< Distance from point to path [m]
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
autoware_utils_geometry::Polygon2d createPathPolygon(
  const autoware_planning_msgs::msg::Trajectory & trajectory, double start_distance,
  double end_distance, double width);

// ========== Distance and Proximity Calculation Functions ==========

/**
 * @brief Calculate minimum distance from predicted object to trajectory
 * @param object Predicted object to calculate distance from
 * @param path Trajectory to calculate distance to
 * @return Minimum distance from object boundary to path [m]
 */
double getMinDistanceToPath(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const autoware_planning_msgs::msg::Trajectory & path);

/**
 * @brief Calculate minimum distance from point to trajectory
 * @param point Point to calculate distance from
 * @param path Trajectory to calculate distance to
 * @return Minimum distance from point to path [m]
 */
double getMinDistanceToPath(
  const geometry_msgs::msg::Point & point, const autoware_planning_msgs::msg::Trajectory & path);

/**
 * @brief Calculate distance from ego vehicle to object
 * @param object Object to calculate distance to
 * @param ego_pose Current ego vehicle pose
 * @return Distance from ego to object [m]
 */
double getDistanceFromEgo(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const geometry_msgs::msg::Pose & ego_pose);

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

}  // namespace autoware::perception_filter

#endif  // AUTOWARE__PERCEPTION_FILTER__PERCEPTION_FILTER_CORE_HPP_
