// Copyright 2020 Tier IV, Inc.
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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "autoware_utils/ros/logger_level_configure.hpp"
#include "autoware_utils/ros/polling_subscriber.hpp"
#include "debug_marker.hpp"
#include "surround_obstacle_checker_node_parameters.hpp"

#include <agnocast/agnocast.hpp>
#include <agnocast/node/tf2/tf2.hpp>
#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit_clear_command.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/utils.h>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::surround_obstacle_checker
{

using autoware::motion_utils::VehicleStopCheckerTemplate;
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_internal_planning_msgs::msg::VelocityLimit;
using autoware_internal_planning_msgs::msg::VelocityLimitClearCommand;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;

using Obstacle = std::pair<double /* distance */, geometry_msgs::msg::Point>;

enum class State { PASS, STOP };

class SurroundObstacleCheckerNode : public agnocast::Node
{
public:
  explicit SurroundObstacleCheckerNode(const rclcpp::NodeOptions & node_options);

private:
  std::array<double, 3> getCheckDistances(const std::string & str_label) const;

  bool getUseDynamicObject() const;

  void onTimer();

  std::optional<Obstacle> getNearestObstacle() const;

  std::optional<Obstacle> getNearestObstacleByPointCloud() const;

  std::optional<Obstacle> getNearestObstacleByDynamicObject() const;

  std::optional<geometry_msgs::msg::TransformStamped> getTransform(
    const std::string & source, const std::string & target, const rclcpp::Time & stamp,
    double duration_sec) const;

  auto isStopRequired(
    const bool is_obstacle_found, const bool is_vehicle_stopped, const State & state,
    const std::optional<rclcpp::Time> & last_obstacle_found_time, const double time_threshold) const
    -> std::pair<bool, std::optional<rclcpp::Time>>;

  // ros
  mutable agnocast::Buffer tf_buffer_;
  std::unique_ptr<agnocast::TransformListener> tf_listener_;
  agnocast::TimerBase::SharedPtr timer_;

  // publisher and subscriber
  agnocast::PollingSubscriber<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  agnocast::PollingSubscriber<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
  agnocast::PollingSubscriber<PredictedObjects>::SharedPtr sub_dynamic_objects_;
  agnocast::Publisher<VelocityLimitClearCommand>::SharedPtr pub_clear_velocity_limit_;
  agnocast::Publisher<VelocityLimit>::SharedPtr pub_velocity_limit_;
  agnocast::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_processing_time_;

  // stop checker
  std::unique_ptr<VehicleStopCheckerTemplate<agnocast::Node>> vehicle_stop_checker_;

  // debug
  std::shared_ptr<SurroundObstacleCheckerDebugNode> debug_ptr_;

  // parameter
  std::shared_ptr<surround_obstacle_checker_node::ParamListener> param_listener_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  // data
  agnocast::ipc_shared_ptr<const nav_msgs::msg::Odometry> odometry_ptr_;
  agnocast::ipc_shared_ptr<const sensor_msgs::msg::PointCloud2> pointcloud_ptr_;
  agnocast::ipc_shared_ptr<const PredictedObjects> object_ptr_;

  // State Machine
  State state_ = State::PASS;
  std::optional<rclcpp::Time> last_obstacle_found_time_;

  std::unique_ptr<autoware_utils::BasicLoggerLevelConfigure<agnocast::Node>> logger_configure_;

  std::unordered_map<int, std::string> label_map_;

public:
  friend class SurroundObstacleCheckerNodeTest;
};
}  // namespace autoware::surround_obstacle_checker

#endif  // NODE_HPP_
