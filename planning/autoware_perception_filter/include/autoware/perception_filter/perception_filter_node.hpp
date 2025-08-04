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

#ifndef AUTOWARE__PERCEPTION_FILTER__PERCEPTION_FILTER_NODE_HPP_
#define AUTOWARE__PERCEPTION_FILTER__PERCEPTION_FILTER_NODE_HPP_

#include "autoware/perception_filter/perception_filter_core.hpp"
#include "autoware/perception_filter/perception_filter_utils.hpp"

#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <autoware/rtc_interface/rtc_interface.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/universe_utils/ros/parameter.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/control_point.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor_array.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/geometry.hpp>

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace autoware::perception_filter
{

/**
 * @brief Perception filter node for supervised object and pointcloud filtering
 * @details Filters perception data based on proximity to planned trajectory with RTC approval
 */
class PerceptionFilterNode : public rclcpp::Node
{
public:
  explicit PerceptionFilterNode(const rclcpp::NodeOptions & node_options);

  /**
   * @brief Check if required data is ready for object filtering
   * @return True if data is ready, false otherwise
   */
  bool isDataReadyForObjects();

  /**
   * @brief Check if required data is ready for pointcloud filtering
   * @return True if data is ready, false otherwise
   */
  bool isDataReadyForPointCloud();

private:
  // ========== Callback Functions ==========

  /**
   * @brief Callback for predicted objects
   * @param msg Predicted objects message
   */
  void onObjects(const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg);

  /**
   * @brief Callback for pointcloud data
   * @param msg Pointcloud message
   */
  void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  /**
   * @brief Callback for planning trajectory
   * @param msg Trajectory message
   */
  void onPlanningTrajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);

  // ========== RTC Interface Functions ==========

  /**
   * @brief Initialize RTC interface
   */
  void initializeRTCInterface();

  /**
   * @brief Update RTC status based on vehicle state
   * @param is_currently_stopped Whether vehicle is currently stopped
   * @param is_just_stopped Whether vehicle just stopped (transition)
   */
  void updateRTCStatus(const bool is_currently_stopped, const bool is_just_stopped);

  // ========== Polygon-based Filtering Functions ==========

  /**
   * @brief Create filtering polygon when RTC is activated
   */
  void createFilteringPolygon();

  /**
   * @brief Update filtering polygon activation status
   * @details Deactivates polygon when ego vehicle passes through it
   */
  void updateFilteringPolygonStatus();

  /**
   * @brief Classify pointcloud points for planning factors with proper coordinate transformation
   * @param input_pointcloud Input pointcloud to classify
   * @param rtc_is_registered Whether RTC interface is registered
   * @return Vector of filtered point information for planning factors
   */
  std::vector<FilteredPointInfo> classifyPointCloudForPlanningFactors(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud, bool rtc_is_registered);

  /**
   * @brief Filter point cloud data based on trajectory proximity with proper coordinate
   * transformation
   * @param input_pointcloud Input point cloud to be filtered
   * @param planning_trajectory Planning trajectory for distance calculation
   * @param filtering_polygon Filtering polygon for RTC-based filtering
   * @param filtering_polygon_created Whether filtering polygon is created
   * @param max_filter_distance Maximum distance from path to filter objects [m]
   * @param pointcloud_safety_distance Minimum safety distance for pointcloud filtering [m]
   * @return Filtered point cloud
   */
  sensor_msgs::msg::PointCloud2 filterPointCloud(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud,
    const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & planning_trajectory,
    const FilteringPolygon & filtering_polygon, bool filtering_polygon_created,
    double max_filter_distance, double pointcloud_safety_distance);

  // ========== Utility Functions ==========

  /**
   * @brief Get current ego vehicle pose
   * @return Ego pose in map frame
   */
  geometry_msgs::msg::Pose getCurrentEgoPose() const;

  // ========== ROS Communication Members ==========

  // Subscribers
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr objects_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr planning_trajectory_sub_;

  // Publishers
  rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr
    filtered_objects_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pointcloud_pub_;
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::PlanningFactorArray>::SharedPtr
    planning_factors_pub_;

  // Debug visualization publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;

  // Processing time publishers
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    objects_processing_time_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pointcloud_processing_time_pub_;

  // Published time publisher
  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_publisher_;

  // ========== Core System Members ==========

  // TF buffer and listener for coordinate transformations
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // RTC interface for approval-based filtering
  std::unique_ptr<autoware::rtc_interface::RTCInterface> rtc_interface_;
  unique_identifier_msgs::msg::UUID rtc_uuid_;

  // Vehicle state monitoring
  autoware::motion_utils::VehicleStopChecker vehicle_stop_checker_;
  bool ego_previously_stopped_{};

  // ========== State Variables ==========

  // Latest received data
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr planning_trajectory_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_pointcloud_;
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr latest_objects_;

  // Classification and filtering state
  ObjectClassification latest_classification_;
  std::vector<FilteredPointInfo> would_be_filtered_points_;

  // RTC state management
  bool previous_rtc_activated_objects_{};  ///< Previous RTC activation state for object processing
  bool previous_rtc_activated_pointcloud_{};  ///< Previous RTC activation state for pointcloud
                                              ///< processing
  std::set<std::array<uint8_t, 16>>
    frozen_filter_object_ids_;  ///< Object IDs frozen at RTC approval time
  bool filtering_active_{};     ///< Whether filtering is currently active

  // Polygon-based filtering management
  FilteringPolygon filtering_polygon_;  ///< Filtering polygon created at RTC approval
  bool filtering_polygon_created_{};    ///< Whether the filtering polygon has been created

  // ========== Configuration Parameters ==========

  // Feature enable/disable flags
  bool enable_object_filtering_;      ///< Enable object filtering functionality
  bool enable_pointcloud_filtering_;  ///< Enable pointcloud filtering functionality

  // Distance and safety parameters
  double max_filter_distance_;           ///< Maximum distance from path to filter objects [m]
  double pointcloud_safety_distance_;    ///< Minimum safety distance for pointcloud filtering [m]
  double object_classification_radius_;  ///< Radius for object classification [m]
  double stop_velocity_threshold_;       ///< Velocity threshold for stop detection [m/s]

  // Object classification parameters
  std::vector<std::string> ignore_object_classes_;  ///< Object classes to ignore during filtering
};

}  // namespace autoware::perception_filter

#endif  // AUTOWARE__PERCEPTION_FILTER__PERCEPTION_FILTER_NODE_HPP_
