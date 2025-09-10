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

#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace autoware::perception_filter
{

// Forward declarations
struct FilteredPointInfo;
struct ObjectClassification;


/**
 * @brief Calculate distance along the path from ego vehicle to given point
 * @param point Point to calculate distance along path for
 * @param planning_trajectory Planning trajectory
 * @param ego_pose Current ego vehicle pose
 * @return Signed distance along the path from ego to the point [m]
 * @details Positive values indicate points ahead of ego, negative values behind
 */
double getDistanceAlongPath(
  const geometry_msgs::msg::Point & point,
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & planning_trajectory,
  const geometry_msgs::msg::Pose & ego_pose);

/**
 * @brief Create planning factors for filtered objects and points
 * @param classification Object classification result
 * @param would_be_filtered_points Points that would be filtered when RTC is approved
 * @param planning_trajectory Planning trajectory for control points
 * @return Planning factor array
 */
autoware_internal_planning_msgs::msg::PlanningFactorArray createPlanningFactors(
  const ObjectClassification & classification,
  const std::vector<FilteredPointInfo> & would_be_filtered_points,
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & planning_trajectory);

/**
 * @brief Create debug visualization markers
 * @param input_objects Input objects for visualization
 * @param classification Object classification result
 * @param rtc_activated Whether RTC is currently activated
 * @param ego_pose Current ego vehicle pose
 * @param filtering_polygon Filtering polygon for visualization
 * @param filtering_polygon_created Whether filtering polygon is created
 * @return Visualization marker array
 */
visualization_msgs::msg::MarkerArray createDebugMarkers(
  const autoware_perception_msgs::msg::PredictedObjects & input_objects,
  const ObjectClassification & classification, bool rtc_activated,
  const geometry_msgs::msg::Pose & ego_pose,
  const autoware::universe_utils::Polygon2d & filtering_polygon, bool filtering_polygon_created);

/**
 * @brief Create visualization marker for objects
 * @param objects Objects to visualize
 * @param frame_id Frame ID for marker
 * @param id Marker ID
 * @param color RGBA color array
 * @return Visualization marker
 */
visualization_msgs::msg::Marker createObjectMarker(
  const autoware_perception_msgs::msg::PredictedObjects & objects, const std::string & frame_id,
  int id, const std::array<double, 4> & color);

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

#endif  // AUTOWARE__PERCEPTION_FILTER__PERCEPTION_FILTER_UTILS_HPP_
