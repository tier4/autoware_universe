// Copyright 2022 TIER IV, Inc.
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

#ifndef START_GOAL_PLANNER__START_GOAL_PLANNER_HPP_
#define START_GOAL_PLANNER__START_GOAL_PLANNER_HPP_

#include "../path_planner.hpp"
#include "../type_alias.hpp"

#include <autoware/lanelet2_utils/kind.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/logger.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <memory>
#include <optional>
#include <vector>

namespace
{
std::vector<PathPointWithLaneId> generate_trajectory_from_points(
  std::vector<geometry_msgs::msg::Point> points, PathPointWithLaneId goal);
}

namespace autoware::minimum_rule_based_planner
{
struct StartGoalPlannerParams
{
  struct SmoothGoalConnection
  {
    double search_radius_range = 20.0;
    double pre_goal_offset = 1.0;
    double clothoid_reference_velocity = 4.7;
    int64_t clothoid_steer_angle_trial_count = 7;
    double clothoid_max_steer_angle_rate_deg_per_sec = 30.0;
  } smooth_goal_connection;
};

using Pose = geometry_msgs::msg::Pose;
class StartGoalPlanner
{
public:
  StartGoalPlanner(
    const rclcpp::Logger & logger, std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    const StartGoalPlannerParams & params, const VehicleInfo & vehicle_info);

  std::optional<PathPointTrajectory> plan(
    const PathPointTrajectory & trajectory, const double & s_path_end,
    const RouteContext & planner_data);
  bool start_planner_active() { return start_planner_act; }
  bool goal_planner_active() { return goal_planner_act; }

private:
  struct AvailableArea
  {
    lanelet::ConstLanelets lanelets;
    lanelet::Areas areas;
  };

  AvailableArea get_available_area(const PathPointTrajectory & trajectory);

  void judge_start_planner_act();
  void judge_goal_planner_act(const PathPointTrajectory & trajectory, const double & s_path_end);
  std::optional<std::vector<PathPointWithLaneId>> get_start_pose(
    const PathPointTrajectory & trajectory);
  std::optional<std::vector<PathPointWithLaneId>> get_goal_pose(
    const PathPointTrajectory & trajectory);
  std::optional<std::vector<PathPointTrajectory>> generate_pull_trajectories(
    const std::vector<PathPointWithLaneId> & start_pose_candidates,
    const std::vector<PathPointWithLaneId> & goal_pose_candidates);
  std::optional<PathPointTrajectory> evaluate_trajectory(
    const std::vector<PathPointTrajectory> & candidate_trajectories,
    const AvailableArea & available_area);
  std::optional<PathPointTrajectory> connect_pull_trajectory(
    const PathPointTrajectory & trajectory, const PathPointTrajectory & pull_trajectory);
  std::optional<PathPointTrajectory> connect_start_planner_trajectory(
    const PathPointTrajectory & trajectory, const PathPointTrajectory & pull_trajectory);
  std::optional<PathPointTrajectory> connect_goal_planner_trajectory(
    const PathPointTrajectory & trajectory, const PathPointTrajectory & pull_trajectory);

  bool start_planner_act{false};
  bool goal_planner_act{false};

  RouteContext route_context_;
  rclcpp::Logger logger_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  StartGoalPlannerParams params_;
  VehicleInfo vehicle_info_;
};
}  // namespace autoware::minimum_rule_based_planner

#endif  // START_GOAL_PLANNER__START_GOAL_PLANNER_HPP_
