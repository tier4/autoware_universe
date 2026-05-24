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

#include "autoware/localization_monitor_host/localization_monitor_host.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace autoware::localization_monitor_host
{

LocalizationMonitorHost::LocalizationMonitorHost(const rclcpp::NodeOptions & options)
: Node("localization_monitor_host", options),
  plugin_loader_(
    std::make_unique<pluginlib::ClassLoader<plugin::LocalizationMonitorPluginBase>>(
      "autoware_localization_monitor_host",
      "autoware::localization_monitor_host::plugin::LocalizationMonitorPluginBase"))
{
  set_up_params();
  load_plugins();

  set_param_res_ = add_on_set_parameters_callback(
    std::bind(&LocalizationMonitorHost::on_parameter, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "input/odom", 1,
    std::bind(&LocalizationMonitorHost::on_odom, this, std::placeholders::_1));

  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "input/twist", 10,
    std::bind(&LocalizationMonitorHost::on_twist, this, std::placeholders::_1));

  timer_ = rclcpp::create_timer(
    this, this->get_clock(), std::chrono::duration<double>(timer_period_),
    std::bind(&LocalizationMonitorHost::on_timer, this));
}

void LocalizationMonitorHost::set_up_params()
{
  const std::vector<std::string> default_plugins = {
    "autoware::localization_monitor_host::plugin::LocalizationErrorPlugin",
    "autoware::localization_monitor_host::plugin::PoseInstabilityPlugin",
  };
  declare_parameter("plugin_names", default_plugins);

  use_localization_error_monitor_ =
    declare_parameter<bool>("use_localization_error_monitor", true);
  use_pose_instability_detector_ =
    declare_parameter<bool>("use_pose_instability_detector", true);
  timer_period_ = declare_parameter<double>("timer_period", 0.1);
}

void LocalizationMonitorHost::load_plugins()
{
  const auto plugin_names = get_parameter("plugin_names").as_string_array();

  for (const auto & plugin_name : plugin_names) {
    try {
      auto plugin = plugin_loader_->createSharedInstance(plugin_name);

      for (const auto & p : plugins_) {
        if (plugin->get_name() == p->get_name()) {
          RCLCPP_WARN_STREAM(
            get_logger(), "Plugin '" << plugin_name << "' is already loaded.");
          continue;
        }
      }

      plugin->initialize(plugin_name, this);
      plugins_.push_back(plugin);
      RCLCPP_INFO_STREAM(get_logger(), "Loaded plugin: " << plugin_name);
    } catch (const pluginlib::CreateClassException & e) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Failed to load plugin '" << plugin_name << "': " << e.what());
    }
  }
}

rcl_interfaces::msg::SetParametersResult LocalizationMonitorHost::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & param : parameters) {
    if (param.get_name() == "use_localization_error_monitor") {
      use_localization_error_monitor_ = param.as_bool();
    } else if (param.get_name() == "use_pose_instability_detector") {
      use_pose_instability_detector_ = param.as_bool();
    } else if (param.get_name() == "timer_period") {
      timer_period_ = param.as_double();
      timer_->cancel();
      timer_ = rclcpp::create_timer(
        this, this->get_clock(), std::chrono::duration<double>(timer_period_),
        std::bind(&LocalizationMonitorHost::on_timer, this));
    }
  }

  for (auto & plugin : plugins_) {
    plugin->on_parameter(parameters);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void LocalizationMonitorHost::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  monitor_data_.latest_odom = *msg;
  monitor_data_.odom_received = true;
}

void LocalizationMonitorHost::on_twist(
  geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
{
  monitor_data_.twist_buffer.push_back(*msg);
}

void LocalizationMonitorHost::on_timer()
{
  if (!monitor_data_.odom_received) {
    return;
  }

  updater_.force_update();

  for (auto & plugin : plugins_) {
    const auto & name = plugin->get_name();
    if (name.find("LocalizationError") != std::string::npos &&
        !use_localization_error_monitor_) {
      continue;
    }
    if (name.find("PoseInstability") != std::string::npos &&
        !use_pose_instability_detector_) {
      continue;
    }
    plugin->evaluate(monitor_data_, updater_);
  }
}

}  // namespace autoware::localization_monitor_host

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::localization_monitor_host::LocalizationMonitorHost)
