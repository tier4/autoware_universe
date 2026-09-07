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

#include "safety_planner.hpp"

#include <autoware/trajectory/threshold.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/trajectory/utils/crop.hpp>
#include <autoware/trajectory/utils/reference_path.hpp>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

SafetyPlanner::SafetyPlanner(const Params & params, std::shared_ptr<TimeKeeper> time_keeper)
: params_(params), time_keeper_(std::move(time_keeper))
{
  load_constraint_generator_plugins();
  load_trajectory_planner_plugin();
}

void SafetyPlanner::load_constraint_generator_plugins()
{
  const auto logger = rclcpp::get_logger("safety_planner");
  constraint_generator_loader_ = std::make_unique<ConstraintGeneratorLoader>(
    "autoware_safety_planner", "autoware::safety_planner::ConstraintGeneratorInterface");

  for (const auto & class_name : params_.constraint_generator_plugins) {
    try {
      auto plugin = constraint_generator_loader_->createSharedInstance(class_name);
      plugin->on_initialize(time_keeper_, params_);
      const auto name = plugin->get_name();
      constraint_generator_plugins_.push_back(std::move(plugin));
      RCLCPP_INFO(
        logger, "Loaded constraint generator plugin: %s (%s)", name.c_str(), class_name.c_str());
    } catch (const pluginlib::PluginlibException & e) {
      RCLCPP_ERROR(
        logger, "Failed to load constraint generator plugin '%s': %s", class_name.c_str(),
        e.what());
    }
  }
}

std::vector<std::string> SafetyPlanner::get_constraint_generator_plugin_names() const
{
  std::vector<std::string> names;
  names.reserve(constraint_generator_plugins_.size());
  for (const auto & plugin : constraint_generator_plugins_) {
    names.push_back(plugin->get_name());
  }
  return names;
}

void SafetyPlanner::load_trajectory_planner_plugin()
{
  const auto logger = rclcpp::get_logger("safety_planner");
  trajectory_planner_loader_ = std::make_unique<TrajectoryPlannerLoader>(
    "autoware_safety_planner", "autoware::safety_planner::TrajectoryPlannerInterface");

  const auto & class_name = params_.trajectory_planner.plugin;
  try {
    trajectory_planner_ = trajectory_planner_loader_->createSharedInstance(class_name);
    trajectory_planner_->on_initialize(time_keeper_, params_);
    RCLCPP_INFO(
      logger, "Loaded trajectory planner plugin: %s (%s)", trajectory_planner_->get_name().c_str(),
      class_name.c_str());
  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_ERROR(
      logger, "Failed to load trajectory planner plugin '%s': %s", class_name.c_str(), e.what());
  }
}

std::string SafetyPlanner::get_trajectory_planner_plugin_name() const
{
  return trajectory_planner_ ? trajectory_planner_->get_name() : std::string{};
}

tl::expected<PathPointTrajectory, std::string> SafetyPlanner::build_reference_path(
  const SafetyPlannerInput & input) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & route_manager = *input.route_manager;
  const double forward_length = params_.reference_path.forward_length_m;
  const double backward_length = params_.reference_path.backward_length_m;

  const auto lane_sequence =
    route_manager.get_lanelet_sequence_on_route(forward_length, backward_length);

  auto reference_path = experimental::trajectory::build_reference_path(
    lane_sequence.as_lanelets(), route_manager.current_lanelet(), input.odometry.pose.pose,
    route_manager.lanelet_map_ptr(), route_manager.routing_graph_ptr(),
    route_manager.traffic_rules_ptr(), forward_length, backward_length);

  if (!reference_path) {
    return tl::unexpected("Failed to build reference path: " + reference_path.error());
  }

  // goal_pose より先だけを crop する。後方 (backward_length_m 分) は残す —
  // 制約の射影が ego 後方の footprint・後方から来る物体を扱うため。
  // goal がまだ前方 (reference_path の終端より先) にある間は終端が最近傍になるので、
  // 実質「後方端から前方終端まで」になる。
  const double s_ego =
    experimental::trajectory::closest(*reference_path, input.odometry.pose.pose.position);
  const double s_goal =
    experimental::trajectory::closest(*reference_path, input.goal_pose.position);

  if (s_goal - s_ego < experimental::trajectory::k_epsilon_distance) {
    return tl::unexpected(
      "goal_pose is behind ego on the reference path (length = " + std::to_string(s_goal - s_ego) +
      ")");
  }

  reference_path->crop(0.0, s_goal);
  return std::move(reference_path.value());
}

tl::expected<SafetyPlannerResult, std::string> SafetyPlanner::plan(const SafetyPlannerInput & input)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  auto reference_path = build_reference_path(input);
  if (!reference_path) {
    return tl::unexpected(reference_path.error());
  }
  const PlannerContext context(input, std::move(reference_path.value()));

  // 制約ジェネレータープラグインを呼び出して制約のリストを生成する
  auto constraints = calculate_constraints(context);

  // certainty ごとに 2 セットへ振り分ける:
  // normal = DEFINITE のみ / cautious = DEFINITE + POSSIBLE
  std::vector<Constraint> normal_list;
  std::vector<Constraint> cautious_list;
  for (const auto & [plugin_name, output] : constraints) {
    for (const auto & constraint : output.constraints) {
      if (constraint.certainty == Certainty::DEFINITE) {
        normal_list.push_back(constraint);
      }
      cautious_list.push_back(constraint);
    }
  }

  // 軌道生成はプラグインの仕事 (制約のコンパイル・rough_planner / optimizer の
  // 呼び出し方は実装詳細)
  SafetyPlannerResult result;
  if (trajectory_planner_) {
    const TrajectoryPlannerInput input{context, normal_list, cautious_list};
    auto planner_result = trajectory_planner_->plan(input);
    result.normal_trajectory = std::move(planner_result.normal_trajectory);
    result.cautious_trajectory = std::move(planner_result.cautious_trajectory);
    result.debug.compiled_constraints = std::move(planner_result.debug.compiled_constraints);
    result.debug.rough_plan_result = std::move(planner_result.debug.rough_plan_result);
    result.debug.trajectory_optimizer_result =
      std::move(planner_result.debug.trajectory_optimizer_result);
  }

  result.debug.constraint_generator_outputs = std::move(constraints);
  result.debug.reference_path = context.reference_path;
  return result;
}

std::map<std::string, ConstraintGeneratorOutput> SafetyPlanner::calculate_constraints(
  const PlannerContext & context)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  std::map<std::string, ConstraintGeneratorOutput> constraints;
  for (const auto & plugin : constraint_generator_plugins_) {
    constraints.emplace(plugin->get_name(), plugin->generate_constraints(context));
  }

  return constraints;
}

}  // namespace autoware::safety_planner
