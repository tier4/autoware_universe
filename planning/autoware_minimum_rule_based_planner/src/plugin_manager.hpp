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

#ifndef PLUGIN_MANAGER_HPP_
#define PLUGIN_MANAGER_HPP_

#include <autoware/rule_based_planner_common/module_interface.hpp>
#include <autoware/rule_based_planner_common/planner_data.hpp>
#include <autoware/rule_based_planner_common/planning_result.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

using rule_based_planner::PlannerData;
using rule_based_planner::PlanningResult;
using rule_based_planner::PluginModuleInterface;

/**
 * @brief Manager class for loading and executing planner module plugins
 *
 * This class handles dynamic loading of planner modules via pluginlib,
 * and coordinates the execution of all loaded modules on input trajectories.
 */
class PlannerModuleManager
{
public:
  PlannerModuleManager();

  /**
   * @brief Load a module plugin by name
   * @param node ROS node reference for plugin initialization
   * @param name Fully qualified plugin class name
   * @param time_keeper Time keeper for performance tracking
   */
  void load_module_plugin(
    rclcpp::Node & node, const std::string & name,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper);

  /**
   * @brief Unload a module plugin by name
   * @param node ROS node reference for logging
   * @param name Module name to unload
   */
  void unload_module_plugin(rclcpp::Node & node, const std::string & name);

  /**
   * @brief Update parameters for all loaded modules
   * @param parameters Vector of changed parameters
   */
  void update_module_parameters(const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Execute all loaded modules on the trajectory
   * @param trajectory_points Input trajectory points
   * @param planner_data Shared planner data
   * @return Vector of planning results from all modules
   */
  std::vector<PlanningResult> plan(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data);

  /**
   * @brief Get list of loaded module names
   * @return Vector of module names
   */
  std::vector<std::string> get_loaded_module_names() const;

  /**
   * @brief Check if any modules are loaded
   * @return true if at least one module is loaded
   */
  bool has_loaded_modules() const { return !loaded_plugins_.empty(); }

private:
  /// Plugin loader for PluginModuleInterface
  pluginlib::ClassLoader<PluginModuleInterface> plugin_loader_;

  /// List of loaded plugin instances
  std::vector<std::shared_ptr<PluginModuleInterface>> loaded_plugins_;

  /// Mutex for thread-safe access to loaded_plugins_
  mutable std::mutex mutex_;
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // PLUGIN_MANAGER_HPP_
