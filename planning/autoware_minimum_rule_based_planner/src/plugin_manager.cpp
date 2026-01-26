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

#include "plugin_manager.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

PlannerModuleManager::PlannerModuleManager()
: plugin_loader_(
    "autoware_minimum_rule_based_planner", "autoware::rule_based_planner::PluginModuleInterface")
{
}

void PlannerModuleManager::load_module_plugin(
  rclcpp::Node & node, const std::string & name,
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Check if plugin is already loaded
  for (const auto & plugin : loaded_plugins_) {
    if (plugin->get_module_name() == name) {
      RCLCPP_WARN_STREAM(node.get_logger(), "Plugin '" << name << "' is already loaded.");
      return;
    }
  }

  // Check if plugin class is available
  if (!plugin_loader_.isClassAvailable(name)) {
    RCLCPP_ERROR_STREAM(node.get_logger(), "Plugin '" << name << "' is not available.");
    RCLCPP_ERROR_STREAM(node.get_logger(), "Available plugins:");
    for (const auto & available_name : plugin_loader_.getDeclaredClasses()) {
      RCLCPP_ERROR_STREAM(node.get_logger(), "  - " << available_name);
    }
    return;
  }

  try {
    // Create plugin instance
    auto plugin = plugin_loader_.createSharedInstance(name);

    // Initialize the plugin
    plugin->init(node, name);

    // Set time keeper for performance tracking
    plugin->set_time_keeper(time_keeper);

    // Add to loaded plugins
    loaded_plugins_.push_back(plugin);

    RCLCPP_INFO_STREAM(node.get_logger(), "Loaded plugin: " << name);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR_STREAM(node.get_logger(), "Failed to load plugin '" << name << "': " << ex.what());
  }
}

void PlannerModuleManager::unload_module_plugin(rclcpp::Node & node, const std::string & name)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = std::remove_if(
    loaded_plugins_.begin(), loaded_plugins_.end(),
    [&name](const auto & plugin) { return plugin->get_module_name() == name; });

  if (it != loaded_plugins_.end()) {
    loaded_plugins_.erase(it, loaded_plugins_.end());
    RCLCPP_INFO_STREAM(node.get_logger(), "Unloaded plugin: " << name);
  } else {
    RCLCPP_WARN_STREAM(node.get_logger(), "Plugin '" << name << "' was not loaded.");
  }
}

void PlannerModuleManager::update_module_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (auto & plugin : loaded_plugins_) {
    plugin->update_parameters(parameters);
  }
}

std::vector<PlanningResult> PlannerModuleManager::plan(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<PlanningResult> results;
  results.reserve(loaded_plugins_.size());

  for (auto & plugin : loaded_plugins_) {
    auto result = plugin->plan(trajectory_points, planner_data);
    results.push_back(result);
  }

  return results;
}

std::vector<std::string> PlannerModuleManager::get_loaded_module_names() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<std::string> names;
  names.reserve(loaded_plugins_.size());

  for (const auto & plugin : loaded_plugins_) {
    names.push_back(plugin->get_module_name());
  }

  return names;
}

}  // namespace autoware::minimum_rule_based_planner
