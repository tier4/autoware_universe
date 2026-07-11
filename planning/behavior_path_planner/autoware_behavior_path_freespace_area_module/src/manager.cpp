// Copyright 2026 TIER IV, Inc.
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

#include "autoware/behavior_path_freespace_area_module/manager.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

void FreespaceAreaModuleManager::init(rclcpp::Node * node)
{
  initInterface(node, {});

  FreespaceAreaParameters p{};

  const std::string ns = "freespace_area.";

  // activation
  p.activation_lookahead_distance =
    node->declare_parameter<double>(ns + "activation.lookahead_distance");

  // planner
  p.planner_algorithm = node->declare_parameter<std::string>(ns + "planner.algorithm");
  p.planner_velocity = node->declare_parameter<double>(ns + "planner.velocity");
  p.planner_vehicle_shape_margin =
    node->declare_parameter<double>(ns + "planner.vehicle_shape_margin");
  p.planner_update_rate = node->declare_parameter<double>(ns + "planner.update_rate");

  // planner.common
  {
    const std::string cns = ns + "planner.common.";
    auto & c = p.common_param;
    c.time_limit = node->declare_parameter<double>(cns + "time_limit");
    c.max_turning_ratio = node->declare_parameter<double>(cns + "max_turning_ratio");
    c.turning_steps = node->declare_parameter<int>(cns + "turning_steps");
    c.theta_size = node->declare_parameter<int>(cns + "theta_size");
    c.angle_goal_range = node->declare_parameter<double>(cns + "angle_goal_range");
    c.lateral_goal_range = node->declare_parameter<double>(cns + "lateral_goal_range");
    c.longitudinal_goal_range = node->declare_parameter<double>(cns + "longitudinal_goal_range");
    c.curve_weight = node->declare_parameter<double>(cns + "curve_weight");
    c.reverse_weight = node->declare_parameter<double>(cns + "reverse_weight");
    c.direction_change_weight = node->declare_parameter<double>(cns + "direction_change_weight");
    c.obstacle_threshold = node->declare_parameter<int>(cns + "obstacle_threshold");
  }

  // planner.astar
  {
    const std::string ans = ns + "planner.astar.";
    auto & a = p.astar_param;
    a.search_method = node->declare_parameter<std::string>(ans + "search_method");
    a.only_behind_solutions = node->declare_parameter<bool>(ans + "only_behind_solutions");
    a.use_back = node->declare_parameter<bool>(ans + "use_back");
    a.adapt_expansion_distance = node->declare_parameter<bool>(ans + "adapt_expansion_distance");
    a.expansion_distance = node->declare_parameter<double>(ans + "expansion_distance");
    a.near_goal_distance = node->declare_parameter<double>(ans + "near_goal_distance");
    a.distance_heuristic_weight =
      node->declare_parameter<double>(ans + "distance_heuristic_weight");
    a.smoothness_weight = node->declare_parameter<double>(ans + "smoothness_weight");
    a.obstacle_distance_weight = node->declare_parameter<double>(ans + "obstacle_distance_weight");
    a.goal_lat_distance_weight = node->declare_parameter<double>(ans + "goal_lat_distance_weight");
  }

  // planner.rrtstar
  {
    const std::string rns = ns + "planner.rrtstar.";
    auto & r = p.rrtstar_param;
    r.enable_update = node->declare_parameter<bool>(rns + "enable_update");
    r.use_informed_sampling = node->declare_parameter<bool>(rns + "use_informed_sampling");
    r.max_planning_time = node->declare_parameter<double>(rns + "max_planning_time");
    r.neighbor_radius = node->declare_parameter<double>(rns + "neighbor_radius");
    r.margin = node->declare_parameter<double>(rns + "margin");
  }

  // latch
  p.replan_lateral_deviation =
    node->declare_parameter<double>(ns + "latch.replan_lateral_deviation");
  p.replan_when_obstacle_found =
    node->declare_parameter<bool>(ns + "latch.replan_when_obstacle_found");
  p.obstacle_check_margin = node->declare_parameter<double>(ns + "latch.obstacle_check_margin");
  p.stuck_time_threshold = node->declare_parameter<double>(ns + "latch.stuck_time_threshold");

  // path
  p.junction_blend_distance = node->declare_parameter<double>(ns + "path.junction_blend_distance");
  p.junction_inset_distance =
    node->declare_parameter<double>(ns + "path.junction_inset_distance", 2.0);
  p.goal_position_tolerance = node->declare_parameter<double>(ns + "path.goal_position_tolerance");
  p.goal_yaw_tolerance_deg = node->declare_parameter<double>(ns + "path.goal_yaw_tolerance_deg");
  p.drivable_area_margin_buffer =
    node->declare_parameter<double>(ns + "path.drivable_area_margin_buffer", 1.0);

  parameters_ = std::make_shared<FreespaceAreaParameters>(p);
}

void FreespaceAreaModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  [[maybe_unused]] auto p = parameters_;

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::FreespaceAreaModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
