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

#ifndef OBSTACLE_STOP_HPP_
#define OBSTACLE_STOP_HPP_

#include <autoware/rule_based_planner_common/module_interface.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::rule_based_planner
{

using TrajectoryPoint = autoware_planning_msgs::msg::TrajectoryPoint;

/**
 * @brief Simple stop obstacle structure for rule-based planner
 */
struct StopObstacle
{
  geometry_msgs::msg::Point collision_point{};  ///< Point where collision would occur
  double dist_to_collision{0.0};                ///< Distance from ego to collision point [m]
};

/**
 * @brief Calculate x offset to vehicle bumper
 * @param is_driving_forward True if driving forward
 * @param vehicle_info Vehicle information
 * @return X offset to bumper [m]
 */
inline double calc_x_offset_to_bumper(
  const bool is_driving_forward, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  if (is_driving_forward) {
    return vehicle_info.max_longitudinal_offset_m;
  }
  return vehicle_info.min_longitudinal_offset_m;
}

/**
 * @brief Obstacle stop module for rule-based planner
 *
 * This module detects obstacles on the trajectory and inserts
 * stop points to avoid collisions.
 */
class ObstacleStopModule : public PluginModuleInterface
{
public:
  /**
   * @brief Initialize the module
   * @param node ROS node reference
   * @param module_name Module name identifier
   */
  void init(rclcpp::Node & node, const std::string & module_name) override;

  /**
   * @brief Update parameters from ROS parameter server
   * @param parameters Vector of changed parameters
   */
  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) override;

  /**
   * @brief Main planning function - detect obstacles and plan stop points
   * @param trajectory_points Input trajectory points
   * @param planner_data Shared planner data
   * @return PlanningResult with stop points
   */
  PlanningResult plan(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data) override;

  /**
   * @brief Get the full module name
   * @return Module name string
   */
  std::string get_module_name() const override { return module_name_; }

  /**
   * @brief Get a short module name for logging
   * @return Short module name
   */
  std::string get_short_module_name() const override { return "obstacle_stop"; }

private:
  /**
   * @brief Plan stop point based on obstacles
   * @param planner_data Shared planner data
   * @param traj_points Trajectory points
   * @param stop_obstacles Detected stop obstacles
   * @param x_offset_to_bumper X offset to vehicle bumper [m]
   * @return Optional stop point
   */
  std::optional<geometry_msgs::msg::Point> plan_stop(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<StopObstacle> & stop_obstacles, const double x_offset_to_bumper);

  // Parameters
  struct Parameters
  {
    double stop_margin{6.0};                  ///< Margin distance before obstacle [m]
    double min_behavior_stop_margin{2.0};     ///< Minimum margin for behavior stop [m]
    double obstacle_check_distance{100.0};    ///< Distance to check for obstacles [m]
    double obstacle_velocity_threshold{0.5};  ///< Velocity threshold for moving obstacle [m/s]
  };
  Parameters params_;
};

}  // namespace autoware::rule_based_planner

#endif  // OBSTACLE_STOP_HPP_
