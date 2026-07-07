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

#ifndef AUTOWARE__BEHAVIOR_PATH_FREESPACE_AREA_MODULE__DATA_STRUCTS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_FREESPACE_AREA_MODULE__DATA_STRUCTS_HPP_

#include <autoware/freespace_planning_algorithms/abstract_algorithm.hpp>
#include <autoware/freespace_planning_algorithms/astar_search.hpp>
#include <autoware/freespace_planning_algorithms/rrtstar.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <atomic>
#include <mutex>
#include <optional>
#include <string>

namespace autoware::behavior_path_planner
{

struct FreespaceAreaParameters
{
  // activation
  double activation_lookahead_distance;

  // planner
  std::string planner_algorithm;
  double planner_velocity;
  double planner_vehicle_shape_margin;
  double planner_update_rate;
  autoware::freespace_planning_algorithms::PlannerCommonParam common_param;
  autoware::freespace_planning_algorithms::AstarParam astar_param;
  autoware::freespace_planning_algorithms::RRTStarParam rrtstar_param;

  // latch & replan
  double replan_lateral_deviation;
  bool replan_when_obstacle_found;
  double obstacle_check_margin;
  double stuck_time_threshold;

  // path composition
  double junction_blend_distance;
  double junction_inset_distance;
  double goal_position_tolerance;
  double goal_yaw_tolerance_deg;
};

// Request handed to the asynchronous planning worker.
struct FreespaceAreaPlanRequest
{
  bool valid{false};
  bool need_plan{false};
  nav_msgs::msg::OccupancyGrid costmap;
  geometry_msgs::msg::Pose start_pose;
  geometry_msgs::msg::Pose goal_pose;
  // Latched trajectory to re-check for obstacles (empty when there is nothing to check).
  geometry_msgs::msg::PoseArray latched_trajectory;
};

// Response produced by the asynchronous planning worker.
struct FreespaceAreaPlanResponse
{
  bool success{false};
  bool obstacle_on_latched{false};
  autoware::freespace_planning_algorithms::PlannerWaypoints waypoints;
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

// State shared between the module (BPP main thread) and the asynchronous planning worker.
// It is owned via std::shared_ptr by BOTH sides: scene module instances are created and
// destroyed every cycle by the planner manager, and the worker's timer callback can still be
// in flight on another executor thread when the module is destroyed. Shared ownership
// guarantees the worker never touches freed memory (dangling mutex/request/response were the
// source of heap corruption crashes).
struct FreespaceAreaWorkerContext
{
  std::mutex mutex;
  std::optional<FreespaceAreaPlanRequest> request;
  FreespaceAreaPlanResponse response;
  std::atomic<bool> is_running{false};
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_FREESPACE_AREA_MODULE__DATA_STRUCTS_HPP_
