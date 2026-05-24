// Copyright 2025 Autoware Foundation
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

#ifndef AUTOWARE__CONTROL_SAFETY_MONITOR_HOST__PLUGIN__TRAJECTORY_OBSTACLE_COLLISION_PLUGIN_HPP_
#define AUTOWARE__CONTROL_SAFETY_MONITOR_HOST__PLUGIN__TRAJECTORY_OBSTACLE_COLLISION_PLUGIN_HPP_

#include "autoware/control_safety_monitor_host/safety_monitor_plugin_base.hpp"

#include <autoware/obstacle_collision_checker/obstacle_collision_checker.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/processing_time_publisher.hpp>
#include <autoware_utils/ros/self_pose_listener.hpp>
#include <autoware_utils/ros/transform_listener.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <memory>

namespace autoware::control_safety_monitor_host::plugin
{

class TrajectoryObstacleCollisionPlugin : public SafetyMonitorPluginBase
{
public:
  void on_initialize() override;
  void set_up_params() override;
  void on_parameter(const std::vector<rclcpp::Parameter> & parameters) override;
  void update() override;
private:
  void on_obstacle_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void on_reference_trajectory(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);
  void on_predicted_trajectory(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_timer();
  bool is_data_ready();
  bool is_data_timeout();
  void check_collision(diagnostic_updater::DiagnosticStatusWrapper & stat);

  std::string param_name(const std::string & name) const;

  std::string param_prefix_{"obstacle_collision_checker."};

  double update_rate_{10.0};
  autoware::obstacle_collision_checker::Input input_{};
  autoware::obstacle_collision_checker::Output output_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  std::shared_ptr<autoware_utils::SelfPoseListener> self_pose_listener_;
  std::shared_ptr<autoware_utils::TransformListener> transform_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_obstacle_pointcloud_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_reference_trajectory_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_predicted_trajectory_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_;
  geometry_msgs::msg::Twist::ConstSharedPtr current_twist_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr obstacle_pointcloud_;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr obstacle_transform_;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr reference_trajectory_;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr predicted_trajectory_;

  std::shared_ptr<autoware_utils::DebugPublisher> debug_publisher_;
  std::shared_ptr<autoware_utils::ProcessingTimePublisher> time_publisher_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;
};

}  // namespace autoware::control_safety_monitor_host::plugin

#endif  // AUTOWARE__CONTROL_SAFETY_MONITOR_HOST__PLUGIN__TRAJECTORY_OBSTACLE_COLLISION_PLUGIN_HPP_
