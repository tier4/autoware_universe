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

#include "autoware/control_safety_monitor_host/safety_monitor_host.hpp"

#include "autoware/control_safety_monitor_host/plugin/surround_collision_plugin.hpp"
#include "autoware/control_safety_monitor_host/plugin/trajectory_object_collision_plugin.hpp"
#include "autoware/control_safety_monitor_host/plugin/trajectory_obstacle_collision_plugin.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace autoware::control_safety_monitor_host
{

SafetyMonitorHost::SafetyMonitorHost(const rclcpp::NodeOptions & options)
: Node("safety_monitor_host", options)
{
  set_up_params();
  load_plugins();
  apply_enable_flags();

  set_param_res_ = add_on_set_parameters_callback(
    std::bind(&SafetyMonitorHost::on_parameter, this, std::placeholders::_1));

  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<double>(timer_period_),
    std::bind(&SafetyMonitorHost::on_timer, this));
}

void SafetyMonitorHost::set_up_params()
{
  const std::vector<std::string> default_plugins = {
    "autoware::control_safety_monitor_host::plugin::SurroundCollisionPlugin",
    "autoware::control_safety_monitor_host::plugin::TrajectoryObstacleCollisionPlugin",
    "autoware::control_safety_monitor_host::plugin::TrajectoryObjectCollisionPlugin",
  };
  declare_parameter("plugin_names", default_plugins);

  use_collision_detector_ = declare_parameter<bool>("use_collision_detector", true);
  use_obstacle_collision_checker_ =
    declare_parameter<bool>("use_obstacle_collision_checker", false);
  use_predicted_path_checker_ = declare_parameter<bool>("use_predicted_path_checker", false);
  timer_period_ = declare_parameter<double>("timer_period", 0.1);
}

namespace
{

std::shared_ptr<plugin::SafetyMonitorPluginBase> create_plugin(const std::string & plugin_name)
{
  if (plugin_name.find("SurroundCollisionPlugin") != std::string::npos) {
    return std::make_shared<plugin::SurroundCollisionPlugin>();
  }
  if (plugin_name.find("TrajectoryObstacleCollisionPlugin") != std::string::npos) {
    return std::make_shared<plugin::TrajectoryObstacleCollisionPlugin>();
  }
  if (plugin_name.find("TrajectoryObjectCollisionPlugin") != std::string::npos) {
    return std::make_shared<plugin::TrajectoryObjectCollisionPlugin>();
  }
  return nullptr;
}

}  // namespace

void SafetyMonitorHost::load_plugins()
{
  const auto plugin_names = get_parameter("plugin_names").as_string_array();

  for (const auto & plugin_name : plugin_names) {
    auto plugin = create_plugin(plugin_name);
    if (!plugin) {
      RCLCPP_ERROR_STREAM(get_logger(), "Unknown safety monitor plugin: " << plugin_name);
      continue;
    }

    bool already_loaded = false;
    for (const auto & p : plugins_) {
      if (plugin->get_name() == p->get_name()) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Plugin '" << plugin_name << "' is already loaded.");
        already_loaded = true;
        break;
      }
    }
    if (already_loaded) {
      continue;
    }

    plugin->initialize(plugin_name, this);
    plugins_.push_back(plugin);
    RCLCPP_INFO_STREAM(get_logger(), "Loaded plugin: " << plugin_name);
  }
}

void SafetyMonitorHost::apply_enable_flags()
{
  if (use_obstacle_collision_checker_ && use_predicted_path_checker_) {
    RCLCPP_WARN(
      get_logger(),
      "Both obstacle_collision_checker and predicted_path_checker are enabled; only "
      "obstacle_collision_checker will run (trajectory checker XOR).");
    use_predicted_path_checker_ = false;
  }

  for (auto & plugin : plugins_) {
    const auto & name = plugin->get_name();
    if (name.find("SurroundCollisionPlugin") != std::string::npos) {
      plugin->set_enabled(use_collision_detector_);
    } else if (name.find("TrajectoryObstacleCollisionPlugin") != std::string::npos) {
      plugin->set_enabled(use_obstacle_collision_checker_);
    } else if (name.find("TrajectoryObjectCollisionPlugin") != std::string::npos) {
      plugin->set_enabled(use_predicted_path_checker_);
    }
  }
}

rcl_interfaces::msg::SetParametersResult SafetyMonitorHost::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & param : parameters) {
    if (param.get_name() == "use_collision_detector") {
      use_collision_detector_ = param.as_bool();
    } else if (param.get_name() == "use_obstacle_collision_checker") {
      use_obstacle_collision_checker_ = param.as_bool();
    } else if (param.get_name() == "use_predicted_path_checker") {
      use_predicted_path_checker_ = param.as_bool();
    } else if (param.get_name() == "timer_period") {
      timer_period_ = param.as_double();
      timer_->cancel();
      timer_ = rclcpp::create_timer(
        this, get_clock(), std::chrono::duration<double>(timer_period_),
        std::bind(&SafetyMonitorHost::on_timer, this));
    }
  }

  apply_enable_flags();

  for (auto & plugin : plugins_) {
    plugin->on_parameter(parameters);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void SafetyMonitorHost::on_timer()
{
  for (auto & plugin : plugins_) {
    if (plugin->is_enabled()) {
      plugin->update();
    }
  }
}

}  // namespace autoware::control_safety_monitor_host

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::control_safety_monitor_host::SafetyMonitorHost)
