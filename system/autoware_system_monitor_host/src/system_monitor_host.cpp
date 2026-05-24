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

#include "autoware/system_monitor_host/system_monitor_host.hpp"

#include <algorithm>
#include <functional>

namespace autoware::system_monitor_host
{

SystemMonitorHost::SystemMonitorHost(const rclcpp::NodeOptions & options)
: Node("monitor_host", options),
  plugin_loader_(
    std::make_unique<pluginlib::ClassLoader<plugin::MonitorPluginBase>>(
      "autoware_system_monitor_host",
      "autoware::system_monitor_host::plugin::MonitorPluginBase"))
{
  updater_ = std::make_shared<diagnostic_updater::Updater>(this);
  updater_->setHardwareID(get_name());

  setup_params();
  initialize_plugins();

  set_param_res_ = add_on_set_parameters_callback(
    std::bind(&SystemMonitorHost::on_parameter, this, std::placeholders::_1));

  const auto period_ns = rclcpp::Rate(update_rate_).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&SystemMonitorHost::on_timer, this));

  RCLCPP_INFO(get_logger(), "SystemMonitorHost initialized with %zu plugins", plugins_.size());
}

void SystemMonitorHost::setup_params()
{
  declare_parameter("update_rate", 1.0);
  declare_parameter("plugin_names", std::vector<std::string>());

  update_rate_ = get_parameter("update_rate").as_double();
}

void SystemMonitorHost::initialize_plugins()
{
  if (initialized_) return;

  const auto plugin_names = get_parameter("plugin_names").as_string_array();

  for (const auto & plugin_name : plugin_names) {
    load_plugin(plugin_name);
  }
  initialized_ = true;
}

void SystemMonitorHost::load_plugin(const std::string & plugin_name)
{
  try {
    auto plugin = plugin_loader_->createSharedInstance(plugin_name);
    plugin->initialize(plugin_name, this, updater_);
    plugins_.push_back(plugin);
    RCLCPP_INFO_STREAM(get_logger(), "Loaded monitor plugin: " << plugin_name);
  } catch (const pluginlib::CreateClassException & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to load plugin " << plugin_name << ": " << e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to load plugin " << plugin_name << ": " << e.what());
  }
}

void SystemMonitorHost::on_timer()
{
  for (auto & plugin : plugins_) {
    plugin->evaluate();
  }
}

rcl_interfaces::msg::SetParametersResult SystemMonitorHost::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (auto & param : parameters) {
    if (param.get_name() == "update_rate") {
      update_rate_ = param.as_double();
      const auto period_ns = rclcpp::Rate(update_rate_).period();
      timer_ = rclcpp::create_timer(
        this, get_clock(), period_ns, std::bind(&SystemMonitorHost::on_timer, this));
    }
    if (param.get_name() == "plugin_names") {
      auto names = param.as_string_array();
      std::vector<std::string> current_names;
      std::transform(
        plugins_.begin(), plugins_.end(), std::back_inserter(current_names),
        [](const auto & p) { return p->get_name(); });
      if (names != current_names) {
        RCLCPP_WARN(get_logger(), "plugin_names changed at runtime; reloading plugins");
        plugins_.clear();
        for (const auto & name : names) {
          load_plugin(name);
        }
      }
    }
  }

  for (auto & plugin : plugins_) {
    auto plugin_result = plugin->on_parameter(parameters);
    if (!plugin_result.successful) {
      result.successful = false;
      result.reason += plugin->get_name() + ": " + plugin_result.reason + "; ";
    }
  }
  return result;
}

}  // namespace autoware::system_monitor_host

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::system_monitor_host::SystemMonitorHost)
