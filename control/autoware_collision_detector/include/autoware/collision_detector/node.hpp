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

#ifndef AUTOWARE__COLLISION_DETECTOR__NODE_HPP_
#define AUTOWARE__COLLISION_DETECTOR__NODE_HPP_

#include <agnocast/agnocast.hpp>
#include <agnocast/node/tf2/buffer.hpp>
#include <agnocast/node/tf2/transform_listener.hpp>
#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/optional.hpp>

#include <tf2/utils.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::collision_detector
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;

using Obstacle = std::pair<double /* distance */, geometry_msgs::msg::Point>;

class CollisionDetectorNode : public agnocast::Node
{
public:
  explicit CollisionDetectorNode(const rclcpp::NodeOptions & node_options);

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

private:
  PredictedObjects filterObjects(const PredictedObjects & objects);

  void removeOldObjects(
    std::vector<TimestampedObject> & container, const rclcpp::Time & current_time,
    const rclcpp::Duration & duration_sec);

  bool shouldBeExcluded(
    const autoware_perception_msgs::msg::ObjectClassification::_label_type & classification) const;

  void checkCollision();

  std::optional<Obstacle> getNearestObstacle(
    const autoware_utils_geometry::Polygon2d & ego_polygon) const;

  std::optional<Obstacle> getNearestObstacleByPointCloud(
    const autoware_utils_geometry::Polygon2d & ego_polygon) const;

  std::optional<Obstacle> getNearestObstacleByDynamicObject(
    const autoware_utils_geometry::Polygon2d & ego_polygon) const;

  std::optional<geometry_msgs::msg::TransformStamped> getTransform(
    const std::string & source, const std::string & target, const rclcpp::Time & stamp,
    double duration_sec) const;

  // ros
  mutable agnocast::Buffer tf_buffer_{get_clock()};
  mutable agnocast::TransformListener tf_listener_{tf_buffer_, *this};
  agnocast::TimerBase::SharedPtr timer_;

  // publisher and subscriber
  agnocast::PollingSubscriber<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  agnocast::PollingSubscriber<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
  agnocast::PollingSubscriber<PredictedObjects>::SharedPtr sub_dynamic_objects_;
  agnocast::PollingSubscriber<OperationModeState>::SharedPtr sub_operation_mode_;
  agnocast::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_;

  // parameter
  NodeParam node_param_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  // data
  agnocast::ipc_shared_ptr<const nav_msgs::msg::Odometry> odometry_ptr_;
  agnocast::ipc_shared_ptr<const sensor_msgs::msg::PointCloud2> pointcloud_ptr_;
  agnocast::ipc_shared_ptr<const PredictedObjects> object_ptr_;
  agnocast::ipc_shared_ptr<const OperationModeState> operation_mode_ptr_;
  std::optional<rclcpp::Time> start_of_consecutive_collision_stamp_;
  std::optional<rclcpp::Time> most_recent_collision_stamp_;
  bool is_error_diag_ = false;
  std::shared_ptr<PredictedObjects> filtered_object_ptr_;
  std::vector<TimestampedObject> observed_objects_;
  std::vector<TimestampedObject> ignored_objects_;

  // diagnostic_updater disabled for agnocast::Node

  std::unique_ptr<autoware::motion_utils::VehicleStopCheckerTemplate<agnocast::Node>>
    vehicle_stop_checker_;
};
}  // namespace autoware::collision_detector

#endif  // AUTOWARE__COLLISION_DETECTOR__NODE_HPP_
