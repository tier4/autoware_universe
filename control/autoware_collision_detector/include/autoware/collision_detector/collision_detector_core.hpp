// Copyright 2024-2025 TIER IV, Inc.
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

#ifndef AUTOWARE__COLLISION_DETECTOR__COLLISION_DETECTOR_CORE_HPP_
#define AUTOWARE__COLLISION_DETECTOR__COLLISION_DETECTOR_CORE_HPP_

#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::collision_detector
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_perception_msgs::msg::PredictedObjects;

using Obstacle = std::pair<double, geometry_msgs::msg::Point>;

/// Core collision detection logic shared by the composable node and safety monitor plugins.
class CollisionDetectorCore
{
public:
  struct NearbyObjectTypeFilters
  {
    bool filter_car{false};
    bool filter_truck{false};
    bool filter_bus{false};
    bool filter_trailer{false};
    bool filter_unknown{false};
    bool filter_bicycle{false};
    bool filter_motorcycle{false};
    bool filter_pedestrian{false};
  };

  struct NodeParam
  {
    bool use_pointcloud{};
    bool use_dynamic_object{};
    double collision_distance{};
    double nearby_filter_radius{};
    double keep_ignoring_time{};
    NearbyObjectTypeFilters nearby_object_type_filters;
    bool ignore_behind_rear_axle{};
    struct
    {
      double on{};
      double off{};
      double off_distance_hysteresis{};
    } time_buffer;
  };

  struct TimestampedObject
  {
    unique_identifier_msgs::msg::UUID object_id;
    rclcpp::Time timestamp;
  };

  /// @param param_prefix Prefix for ROS parameters (e.g. "" or "collision_detector.")
  CollisionDetectorCore(
    rclcpp::Node * node, const std::string & param_prefix = "",
    const std::string & odom_topic = "~/input/odometry",
    const std::string & pointcloud_topic = "~/input/pointcloud",
    const std::string & objects_topic = "~/input/objects");

  void declare_parameters();
  void setup_diagnostics();
  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

private:
  PredictedObjects filterObjects(const PredictedObjects & objects);

  void removeOldObjects(
    std::vector<TimestampedObject> & container, const rclcpp::Time & current_time,
    const rclcpp::Duration & duration_sec);

  bool shouldBeExcluded(
    const autoware_perception_msgs::msg::ObjectClassification::_label_type & classification) const;

  std::optional<Obstacle> getNearestObstacle(
    const autoware_utils_geometry::Polygon2d & ego_polygon) const;

  std::optional<Obstacle> getNearestObstacleByPointCloud(
    const autoware_utils_geometry::Polygon2d & ego_polygon) const;

  std::optional<Obstacle> getNearestObstacleByDynamicObject(
    const autoware_utils_geometry::Polygon2d & ego_polygon) const;

  std::optional<geometry_msgs::msg::TransformStamped> getTransform(
    const std::string & source, const std::string & target, const rclcpp::Time & stamp,
    double duration_sec) const;

  std::string param_name(const std::string & name) const;

  rclcpp::Node * node_;
  std::string param_prefix_;

  mutable tf2_ros::Buffer tf_buffer_;
  mutable tf2_ros::TransformListener tf_listener_;

  autoware_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry> sub_odometry_;
  autoware_utils::InterProcessPollingSubscriber<sensor_msgs::msg::PointCloud2> sub_pointcloud_;
  autoware_utils::InterProcessPollingSubscriber<PredictedObjects> sub_dynamic_objects_;
  autoware_utils::InterProcessPollingSubscriber<OperationModeState> sub_operation_mode_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_;

  NodeParam node_param_;
  VehicleInfo vehicle_info_;

  nav_msgs::msg::Odometry::ConstSharedPtr odometry_ptr_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_ptr_;
  PredictedObjects::ConstSharedPtr object_ptr_;
  OperationModeState::ConstSharedPtr operation_mode_ptr_;
  std::optional<rclcpp::Time> start_of_consecutive_collision_stamp_;
  std::optional<rclcpp::Time> most_recent_collision_stamp_;
  bool is_error_diag_{false};
  std::shared_ptr<PredictedObjects> filtered_object_ptr_;
  std::vector<TimestampedObject> observed_objects_;
  std::vector<TimestampedObject> ignored_objects_;

  std::unique_ptr<autoware::motion_utils::VehicleStopChecker> vehicle_stop_checker_;
};

}  // namespace autoware::collision_detector

#endif  // AUTOWARE__COLLISION_DETECTOR__COLLISION_DETECTOR_CORE_HPP_
