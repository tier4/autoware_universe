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

#include "utils/reference_path_smoother.hpp"
#include "utils/trajectory_postprocess.hpp"

#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/trajectory/threshold.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/trajectory/utils/crop.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware/trajectory/utils/reference_path.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

std::optional<PathPointTrajectory> connect_reference_path_to_goal(
  const PathPointTrajectory & path, const Pose & goal,
  const lanelet::ConstLanelets & route_lanelets, const lanelet::LaneletMapConstPtr & lanelet_map,
  const double search_radius_m)
{
  using autoware_utils_geometry::calc_distance2d;
  //! [m] where the connecting points are placed in front of the goal, as in goal_planner
  constexpr double PRE_GOAL_DISTANCE_M = 1.0;
  constexpr double PRE_MID_GOAL_DISTANCE_M = 0.5;
  //! [m] spacing at which the new stretch is checked against the lanes
  constexpr double VALIDATION_STEP_M = 1.0;
  //! [m] by how much the radius shrinks per retry
  constexpr double RADIUS_REDUCE_M = 1.0;

  const double s_goal = experimental::trajectory::closest(path, goal.position);
  if (calc_distance2d(path.compute(s_goal).point.pose, goal) > search_radius_m) {
    return std::nullopt;  // the goal is outside the window ahead, or too far off to the side
  }
  const auto points = path.restore();
  const auto bases = path.get_underlying_bases();

  const double goal_yaw = autoware_utils_geometry::get_rpy(goal).z;
  const double goal_z = path.compute(s_goal).point.pose.position.z;
  const auto make_goal_point = [&](const double back_distance) {
    PathPointWithLaneId point = points.back();
    point.point.pose = goal;
    point.point.pose.position.x -= std::cos(goal_yaw) * back_distance;
    point.point.pose.position.y -= std::sin(goal_yaw) * back_distance;
    point.point.pose.position.z = goal_z;
    return point;
  };

  const auto is_inside_map_lane = [&](const geometry_msgs::msg::Pose & pose) {
    const lanelet::BasicPoint2d p{pose.position.x, pose.position.y};
    for (const auto & lanelet : route_lanelets) {
      if (lanelet::geometry::inside(lanelet, p)) {
        return true;
      }
    }
    return !experimental::lanelet2_utils::get_road_lanelets_at(lanelet_map, p.x(), p.y()).empty() ||
           !experimental::lanelet2_utils::get_shoulder_lanelets_at(lanelet_map, p.x(), p.y())
              .empty();
  };

  std::optional<PathPointTrajectory> last_built;
  for (double radius = search_radius_m; radius >= 0.0; radius -= RADIUS_REDUCE_M) {
    std::size_t cut_index = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (bases[i] > s_goal) {
        break;
      }
      cut_index = i;
      if (calc_distance2d(points[i].point.pose, goal) > radius) {
        continue;
      }
      cut_index = i == 0 ? 0 : i - 1;
      break;
    }

    std::vector<PathPointWithLaneId> connected(
      points.begin(), points.begin() + static_cast<std::ptrdiff_t>(cut_index) + 1);
    connected.push_back(make_goal_point(PRE_GOAL_DISTANCE_M));
    connected.push_back(make_goal_point(PRE_MID_GOAL_DISTANCE_M));
    connected.push_back(make_goal_point(0.0));
    for (auto it = connected.end() - 3; it != connected.end(); ++it) {
      it->lane_ids = points[cut_index].lane_ids;
    }

    auto built = experimental::trajectory::pretty_build(connected);
    if (!built) {
      continue;
    }
    last_built = std::move(*built);

    bool valid = true;
    for (double s = bases[cut_index]; s <= last_built->length(); s += VALIDATION_STEP_M) {
      if (!is_inside_map_lane(last_built->compute(s).point.pose)) {
        valid = false;
        break;
      }
    }
    if (valid) {
      break;
    }
  }
  return last_built;
}

}  // namespace

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

  const auto & class_name = params_.trajectory_planner_plugin;
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

  const auto & policy = params_.reference_path.policy;
  if (policy == "goal_connection" || policy == "goal_connection_and_smooth") {
    if (
      auto connected = connect_reference_path_to_goal(
        *reference_path, input.goal_pose, lane_sequence.as_lanelets(),
        route_manager.lanelet_map_ptr(), params_.reference_path.goal_connection.search_radius_m)) {
      reference_path = std::move(*connected);
    }
  }
  if (policy == "goal_connection_and_smooth") {
    if (
      auto smoothed = smooth_reference_path(
        *reference_path, lane_sequence.as_lanelets(), input.vehicle_info.max_lateral_offset_m,
        params_.reference_path.smoother.clearance_m)) {
      reference_path = std::move(*smoothed);
    }
  }

  const double s_ego =
    experimental::trajectory::closest(*reference_path, input.odometry.pose.pose.position);
  const double s_goal =
    experimental::trajectory::closest(*reference_path, input.goal_pose.position);

  constexpr double GOAL_OVERRUN_TOLERANCE_M = 1.0;
  if (s_goal - s_ego < -GOAL_OVERRUN_TOLERANCE_M) {
    return tl::unexpected(
      "goal_pose is behind ego on the reference path (length = " + std::to_string(s_goal - s_ego) +
      ")");
  }

  reference_path->crop(0.0, std::max(s_goal, s_ego));
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

  auto constraints = calculate_constraints(context);

  // Split by certainty: normal = DEFINITE only, cautious = DEFINITE + POSSIBLE
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

  SafetyPlannerResult result;
  if (trajectory_planner_) {
    const TrajectoryPlannerInput input{context, normal_list, cautious_list};
    auto planner_result = trajectory_planner_->plan(input);
    if (params_.engage_velocity.enable) {
      if (planner_result.normal_trajectory) {
        auto & trajectory = planner_result.normal_trajectory->trajectory;
        trajectory = set_engage_speed(trajectory, params_.engage_velocity.velocity_hard_mps);
      }
      if (planner_result.cautious_trajectory) {
        auto & trajectory = planner_result.cautious_trajectory->trajectory;
        trajectory = set_engage_speed(trajectory, params_.engage_velocity.velocity_hard_mps);
      }
    }
    result.normal_trajectory = std::move(planner_result.normal_trajectory);
    result.cautious_trajectory = std::move(planner_result.cautious_trajectory);
    result.debug.normal = std::move(planner_result.normal_debug);
    result.debug.cautious = std::move(planner_result.cautious_debug);
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
