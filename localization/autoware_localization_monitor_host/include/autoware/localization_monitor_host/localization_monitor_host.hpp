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

#ifndef AUTOWARE__LOCALIZATION_MONITOR_HOST__LOCALIZATION_MONITOR_HOST_HPP_
#define AUTOWARE__LOCALIZATION_MONITOR_HOST__LOCALIZATION_MONITOR_HOST_HPP_

#include "autoware/localization_monitor_host/localization_monitor_plugin_base.hpp"
#include "autoware/localization_monitor_host/monitor_data.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::localization_monitor_host
{

class LocalizationMonitorHost : public rclcpp::Node
{
public:
  explicit LocalizationMonitorHost(const rclcpp::NodeOptions & options);

private:
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void on_twist(geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg);
  void on_timer();

  void set_up_params();
  void load_plugins();
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters);

  std::unique_ptr<pluginlib::ClassLoader<plugin::LocalizationMonitorPluginBase>> plugin_loader_;
  std::vector<std::shared_ptr<plugin::LocalizationMonitorPluginBase>> plugins_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  diagnostic_updater::Updater updater_{this};

  MonitorData monitor_data_;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  bool use_localization_error_monitor_{true};
  bool use_pose_instability_detector_{true};
  double timer_period_{0.1};
};

}  // namespace autoware::localization_monitor_host

#endif  // AUTOWARE__LOCALIZATION_MONITOR_HOST__LOCALIZATION_MONITOR_HOST_HPP_
