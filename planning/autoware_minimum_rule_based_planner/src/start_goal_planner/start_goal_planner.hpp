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

#ifndef START_GOAL_PLANNER__START_GOAL_PLANNER_HPP_
#define START_GOAL_PLANNER__START_GOAL_PLANNER_HPP_

#include "../type_alias.hpp"

#include <autoware/lanelet2_utils/kind.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/logger.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
using StartGoalPlannerParams = Params::PathPlanning::StartGoalPlanner;

using Pose = geometry_msgs::msg::Pose;
class StartGoalPlanner
{
public:
  struct RouteData
  {
    geometry_msgs::msg::Pose goal_pose{};
    lanelet::ConstLanelets preferred_lanelets{};
    lanelet::ConstLanelets start_lanelets{};
    lanelet::LaneletMapPtr lanelet_map_ptr{nullptr};
    lanelet::routing::RoutingGraphPtr routing_graph_ptr{nullptr};
  };

  StartGoalPlanner(
    const rclcpp::Logger & logger, std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    const StartGoalPlannerParams & params, const VehicleInfo & vehicle_info);

  // entry point to generate start goal trajectory
  std::optional<PathPointTrajectory> plan(
    const PathPointTrajectory & trajectory, lanelet::ConstLanelet & current_lanelet,
    const double & s_path_end, const geometry_msgs::msg::Pose & ego_pose);

  // Route / map initialisation
  void set_route_data(const RouteData & route_data);
  void update_params(const StartGoalPlannerParams & params);

  // Confirm the start planner has generated the trajectory
  bool start_planner_active() { return start_planner_act_; }

  // Confirm the goal planner has generated the trajectory
  bool goal_planner_active() { return goal_planner_act_ && generated_trajectory_.has_value(); }

private:
  // polygon type which can be used to pull-out and pull-over
  const std::unordered_set<std::string> available_area_type = {"parking_lot"};

  // get polygon of available lanelet and area
  std::vector<lanelet::BasicPolygon2d> get_available_area(const PathPointTrajectory & trajectory);

  // judge condition to activate start planner
  void judge_start_planner_act(
    const lanelet::ConstLanelet & current_lanelet, const geometry_msgs::msg::Pose & ego_pose);

  // judge condition to activate goal planner
  void judge_goal_planner_act(
    const PathPointTrajectory & trajectory, const double & s_path_end,
    const std::vector<lanelet::BasicPolygon2d> & available_area,
    const geometry_msgs::msg::Pose & ego_pose);

  // judge if previous trajectory is valid
  bool is_prev_traj_valid(const geometry_msgs::msg::Pose & ego_pose);

  // get candidates for initial pose pull-trajectory starts from
  std::optional<std::vector<PathPointWithLaneId>> get_start_pose(
    const PathPointTrajectory & trajectory, const geometry_msgs::msg::Pose & ego_pose);

  // get candidates for end pose pull-trajectory try to reach
  std::optional<std::vector<PathPointWithLaneId>> get_goal_pose(
    const PathPointTrajectory & trajectory, const geometry_msgs::msg::Pose & ego_pose);
  std::optional<std::vector<PathPointTrajectory>> generate_pull_trajectories(
    const PathPointWithLaneId & start_point, const PathPointWithLaneId & goal_point,
    const double & first_steer_angle);
  std::optional<double> evaluate_trajectory(
    const PathPointTrajectory & candidate,
    const std::vector<lanelet::BasicPolygon2d> & available_area,
    const geometry_msgs::msg::Pose & ego_pose);
  std::optional<PathPointTrajectory> generate_and_evaluate_trajectory(
    const std::vector<lanelet::BasicPolygon2d> & available_area,
    const geometry_msgs::msg::Pose & ego_pose,
    std::vector<PathPointWithLaneId> start_pose_candidates,
    std::vector<PathPointWithLaneId> goal_pose_candidates);

  // connect input trajectory and generated pull-trajectory
  std::optional<PathPointTrajectory> connect_pull_trajectory(
    const PathPointTrajectory & trajectory, const PathPointTrajectory & pull_trajectory);
  std::optional<PathPointTrajectory> connect_start_planner_trajectory(
    const PathPointTrajectory & trajectory, const PathPointTrajectory & pull_trajectory);
  std::optional<PathPointTrajectory> connect_goal_planner_trajectory(
    const PathPointTrajectory & trajectory, const PathPointTrajectory & pull_trajectory);

  // return fallback_trajectory when trajectory generation is failed
  std::optional<PathPointTrajectory> generate_fallback_trajectory(
    const PathPointTrajectory & trajectory);

  bool start_planner_act_{false};
  bool goal_planner_act_{false};

  // lached trajectory generated at previous time step
  std::optional<PathPointTrajectory> generated_trajectory_{std::nullopt};
  std::optional<Pose> goal_pose_prev_{std::nullopt};

  RouteData route_data_;
  rclcpp::Logger logger_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  StartGoalPlannerParams params_;
  VehicleInfo vehicle_info_;
};
}  // namespace autoware::minimum_rule_based_planner

#endif  // START_GOAL_PLANNER__START_GOAL_PLANNER_HPP_
