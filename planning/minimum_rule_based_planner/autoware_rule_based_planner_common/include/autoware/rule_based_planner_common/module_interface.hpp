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

#ifndef AUTOWARE__RULE_BASED_PLANNER_COMMON__MODULE_INTERFACE_HPP_
#define AUTOWARE__RULE_BASED_PLANNER_COMMON__MODULE_INTERFACE_HPP_

#include "autoware/rule_based_planner_common/planner_data.hpp"
#include "autoware/rule_based_planner_common/planning_result.hpp"

#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::rule_based_planner
{

/**
 * @brief Base interface class for rule-based planner modules (plugins)
 *
 * This interface defines the contract that all planner modules must implement.
 * Modules can modify trajectory points (e.g., add stop points, modify velocities).
 */
class PluginModuleInterface
{
public:
  virtual ~PluginModuleInterface() = default;

  /**
   * @brief Initialize the module with ROS node and module name
   * @param node Reference to the ROS node for creating publishers/subscribers
   * @param module_name Name identifier for this module
   */
  virtual void init(rclcpp::Node & node, const std::string & module_name) = 0;

  /**
   * @brief Update module parameters from ROS parameter server
   * @param parameters Vector of changed parameters
   */
  virtual void update_parameters(const std::vector<rclcpp::Parameter> & parameters) = 0;

  /**
   * @brief Main planning function - process trajectory and return planning result
   * @param trajectory_points Input trajectory points to process
   * @param planner_data Shared planner data (map, route, ego state, etc.)
   * @return PlanningResult containing stop points, slowdown intervals, etc.
   */
  virtual PlanningResult plan(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data) = 0;

  /**
   * @brief Get the full module name
   * @return Module name string
   */
  virtual std::string get_module_name() const = 0;

  /**
   * @brief Get a short module name for logging/debug purposes
   * @return Short module name string
   */
  virtual std::string get_short_module_name() const { return "module"; }

  /**
   * @brief Set the time keeper for performance tracking
   * @param time_keeper Shared pointer to time keeper
   */
  void set_time_keeper(std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
  {
    time_keeper_ = time_keeper;
  }

protected:
  rclcpp::Logger logger_ = rclcpp::get_logger("");
  rclcpp::Clock::SharedPtr clock_;
  std::string module_name_{};
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;

  // Debug publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr virtual_wall_publisher_;
};

}  // namespace autoware::rule_based_planner

#endif  // AUTOWARE__RULE_BASED_PLANNER_COMMON__MODULE_INTERFACE_HPP_
