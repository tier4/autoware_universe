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

#ifndef AUTOWARE__SYSTEM_MONITOR_HOST__SYSTEM_MONITOR_HOST_HPP_
#define AUTOWARE__SYSTEM_MONITOR_HOST__SYSTEM_MONITOR_HOST_HPP_

#include "autoware/system_monitor_host/plugins/monitor_plugin_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::system_monitor_host
{

class SystemMonitorHost : public rclcpp::Node
{
public:
  explicit SystemMonitorHost(const rclcpp::NodeOptions & options);

private:
  void load_plugin(const std::string & plugin_name);
  void initialize_plugins();
  void setup_params();
  void on_timer();
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters);

  std::unique_ptr<pluginlib::ClassLoader<plugin::MonitorPluginBase>> plugin_loader_;
  std::vector<std::shared_ptr<plugin::MonitorPluginBase>> plugins_;
  std::shared_ptr<diagnostic_updater::Updater> updater_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  bool initialized_{false};
  double update_rate_{1.0};
};

}  // namespace autoware::system_monitor_host

#endif  // AUTOWARE__SYSTEM_MONITOR_HOST__SYSTEM_MONITOR_HOST_HPP_
