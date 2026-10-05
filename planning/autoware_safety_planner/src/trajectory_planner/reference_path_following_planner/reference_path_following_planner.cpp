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

#include "reference_path_following_planner.hpp"

#include "../../utils/frenet_utils.hpp"
#include "../../utils/velocity_optimizer.hpp"
#include "../frenet_sampling_based_planner/compiled_constraints_utils.hpp"
#include "../frenet_sampling_based_planner/constraints_compiler.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner::experimental
{

namespace
{

rclcpp::Logger logger()
{
  return rclcpp::get_logger("safety_planner.reference_path_following_planner");
}

rclcpp::Clock & steady_clock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

//! Upper bound of the global HARD ScalarBound constraints on quantity; INF when there is none
double global_hard_bound_max(
  const CompiledConstraints & compiled_constraints, const BoundedQuantity quantity)
{
  double max = INF;
  for (const auto & bound : compiled_constraints.scalar_bounds) {
    if (
      bound.quantity == quantity && bound.s0 == -INF && bound.s1 == INF &&
      compiled_constraints.raw_constraints[bound.raw_index].hardness == Hardness::HARD) {
      max = std::min(max, bound.max);
    }
  }
  return max;
}

//! The speed limit baked into path, from s0, drawn along it as a stairstep line at its value
//! [m/s] above the road, with the value at the ego as a text
MarkerArray make_speed_limit_markers(
  const PathPointTrajectory & path, const double s0, const builtin_interfaces::msg::Time & stamp)
{
  using autoware_utils_visualization::create_default_marker;
  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;

  const auto point_at = [&](const double s, const double v) {
    auto point = path.compute(s).point.pose.position;
    point.z += v;
    return point;
  };
  const auto speed_limit_at = [&](const double s) {
    return static_cast<double>(path.compute(s).point.longitudinal_velocity_mps);
  };

  auto line = create_default_marker(
    "map", stamp, "reference_path_following_speed_limit", 0, Marker::LINE_STRIP,
    create_marker_scale(0.1, 0.0, 0.0), create_marker_color(0.0, 1.0, 1.0, 0.8));
  double v = speed_limit_at(s0);
  line.points.push_back(point_at(s0, v));
  for (const double s : path.get_underlying_bases()) {
    if (s <= s0) {
      continue;
    }
    line.points.push_back(point_at(s, v));
    v = speed_limit_at(s);
    line.points.push_back(point_at(s, v));
  }

  auto text = create_default_marker(
    "map", stamp, "reference_path_following_speed_limit_at_ego", 0, Marker::TEXT_VIEW_FACING,
    create_marker_scale(0.0, 0.0, 1.0), create_marker_color(0.0, 1.0, 1.0, 0.999));
  text.pose.position = line.points.front();
  text.pose.position.z += 1.0;
  char buf[32];
  std::snprintf(buf, sizeof(buf), "speed_limit %.2f m/s", speed_limit_at(s0));
  text.text = buf;

  MarkerArray markers;
  markers.markers.push_back(std::move(line));
  markers.markers.push_back(std::move(text));
  return markers;
}

//! (v0, a0) as the VelocitySmoother of autoware_minimum_rule_based_planner. Below engage_mps the
//! profile starts at engage_mps with the acceleration left to the QP: the first grid interval runs
//! at a0 (b' = 2a), so from (0, 0) it takes tens of seconds to leave it. Otherwise a0 is the one
//! planned at the ego in previous_trajectory rather than a_ego, which carries the response of the
//! controller back into the plan, unless the velocity planned there is off v_ego by over 3 m/s
InitialMotion calc_initial_motion(
  const double v_ego, const double a_ego, const std::optional<Trajectory> & previous_trajectory,
  const geometry_msgs::msg::Point & ego_position, const double engage_mps)
{
  if (v_ego < engage_mps) {
    return {engage_mps, std::nullopt};
  }
  InitialMotion initial{v_ego, a_ego};
  if (previous_trajectory && !previous_trajectory->points.empty()) {
    const auto & points = previous_trajectory->points;
    const auto nearest = std::min_element(
      points.begin(), points.end(), [&](const TrajectoryPoint & x, const TrajectoryPoint & y) {
        return autoware_utils_geometry::calc_squared_distance2d(x, ego_position) <
               autoware_utils_geometry::calc_squared_distance2d(y, ego_position);
      });
    constexpr double MAX_VELOCITY_DEVIATION_MPS = 3.0;
    if (std::abs(nearest->longitudinal_velocity_mps - v_ego) <= MAX_VELOCITY_DEVIATION_MPS) {
      initial.a = nearest->acceleration_mps2;
    }
  }
  return initial;
}

}  // namespace

void ReferencePathFollowingPlanner::on_initialize(
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper, const Params & params)
{
  TrajectoryPlannerInterface::on_initialize(time_keeper, params);
  const TurnSignalParams turn_signal_params{
    params.turn_signal.search_distance, params.turn_signal.min_blink_duration,
    params.turn_signal.stopped_velocity_threshold, params.turn_signal.heading_align_threshold};
  normal_turn_indicator_decider_.update_params(turn_signal_params);
  cautious_turn_indicator_decider_.update_params(turn_signal_params);
}

TrajectoryPlannerResult ReferencePathFollowingPlanner::plan_trajectories(
  const TrajectoryPlannerInput & input)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  TrajectoryPlannerResult result;
  {
    autoware_utils_debug::ScopedTimeTrack side_st("plan_normal", *time_keeper_);
    result.normal_trajectory = plan_one_side(
      normal_turn_indicator_decider_, input.context, input.normal_constraints,
      normal_previous_trajectory_, result.normal_debug);
  }
  const bool cautious_differs = std::any_of(
    input.cautious_constraints.begin(), input.cautious_constraints.end(),
    [](const Constraint & constraint) { return constraint.certainty == Certainty::POSSIBLE; });
  if (cautious_differs) {
    autoware_utils_debug::ScopedTimeTrack side_st("plan_cautious", *time_keeper_);
    result.cautious_trajectory = plan_one_side(
      cautious_turn_indicator_decider_, input.context, input.cautious_constraints,
      cautious_previous_trajectory_, result.cautious_debug);
  } else {
    // Not left empty, since the node publishes the cautious candidate every cycle
    result.cautious_trajectory = result.normal_trajectory;
    result.cautious_debug = result.normal_debug;
    cautious_previous_trajectory_ = normal_previous_trajectory_;
  }
  return result;
}

PlannedTrajectory ReferencePathFollowingPlanner::plan_one_side(
  TurnIndicatorDecider & turn_indicator_decider, const PlannerContext & context,
  const std::vector<Constraint> & constraints, std::optional<Trajectory> & previous_trajectory,
  TrajectoryPlannerDebug & debug) const
{
  const auto & p = params_.reference_path_following_planner;
  const auto & vo = p.velocity_optimizer;

  auto phase = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(
    "compile_constraint_list", *time_keeper_);
  const auto compiled_constraints = compile_constraint_list(context, constraints);
  phase.reset();

  phase = std::make_unique<autoware_utils_debug::ScopedTimeTrack>("plan_velocity", *time_keeper_);
  const auto limits = collect_kinematic_limits(compiled_constraints);
  const double nominal_jerk =
    std::min(limits.j_nom, global_hard_bound_max(compiled_constraints, BoundedQuantity::LON_JERK));

  const double s0 = compute_ego_frenet_state(context).s;
  const double s_stop =
    stop_target_s(context, compiled_constraints, params_.trajectory_horizon_s, s0);
  const double v_ego = std::max(0.0, context.odometry.twist.twist.linear.x);

  auto path = context.reference_path;
  auto & speed_limit = path.longitudinal_velocity_mps();
  speed_limit = limits.v_hard;
  for (const auto & bound : compiled_constraints.scalar_bounds) {
    if (bound.quantity != BoundedQuantity::VELOCITY || (bound.s0 == -INF && bound.s1 == INF)) {
      continue;
    }
    const double z0 = std::clamp(bound.s0, 0.0, path.length());
    const double z1 = std::clamp(bound.s1, 0.0, path.length());
    if (z0 >= z1) {
      continue;
    }
    const double after = speed_limit.compute(z1);
    speed_limit.range(z0, z1).clamp(bound.max);
    speed_limit.at(z1).set(after);
  }
  path.set_stopline(std::min(s_stop, path.length()));
  debug.markers["reference_path_following_speed_limit"] =
    make_speed_limit_markers(path, s0, context.odometry.header.stamp);

  // TODO(odashima): apply stopline as speed limit

  VelocityPlanningParams vp_params;
  vp_params.resolution_m = vo.resolution_m;
  vp_params.max_length_m = vo.max_length_m;
  vp_params.lat_accel = std::min(
    limits.a_lat_nom, global_hard_bound_max(compiled_constraints, BoundedQuantity::LAT_ACCEL));
  vp_params.steer_rate = global_hard_bound_max(compiled_constraints, BoundedQuantity::STEER_RATE);
  vp_params.wheel_base_m = context.vehicle_info.wheel_base_m;
  vp_params.optimizer.a_min = limits.a_nom_min;
  vp_params.optimizer.a_max = limits.a_nom_max;
  vp_params.optimizer.j_min = -nominal_jerk;
  vp_params.optimizer.j_max = nominal_jerk;
  vp_params.optimizer.jerk_weight = vo.weights.jerk;
  vp_params.optimizer.over_v_weight = vo.weights.over_velocity;
  vp_params.optimizer.over_a_weight = vo.weights.over_acceleration;
  vp_params.optimizer.over_j_weight = vo.weights.over_jerk;
  const auto initial = calc_initial_motion(
    v_ego, context.acceleration.accel.accel.linear.x, previous_trajectory,
    context.odometry.pose.pose.position, params_.engage_velocity.velocity_hard_mps);
  const auto num_points =
    static_cast<std::size_t>(std::round(params_.trajectory_horizon_s / p.time_step_s)) + 1;
  auto points = plan_velocity(path, s0, initial, vp_params, num_points, p.time_step_s);
  phase.reset();

  phase = std::make_unique<autoware_utils_debug::ScopedTimeTrack>("to_trajectory", *time_keeper_);
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = context.odometry.header.stamp;
  if (points) {
    trajectory.points = std::move(*points);
  } else {
    constexpr auto FAILURE = "velocity QP did not solve";
    RCLCPP_WARN_THROTTLE(logger(), steady_clock(), 5000, "%s", FAILURE);
    // No fallback on purpose, as the MPPI planner: the failure has to be visible downstream, so
    // only the ego point is output and the reason is recorded as a marker
    TrajectoryPoint point;
    point.pose = context.odometry.pose.pose;
    trajectory.points.push_back(point);
    auto marker = autoware_utils_visualization::create_default_marker(
      "map", context.odometry.header.stamp, "reference_path_following_failure", 0,
      Marker::TEXT_VIEW_FACING, autoware_utils_visualization::create_marker_scale(0.0, 0.0, 1.0),
      autoware_utils_visualization::create_marker_color(1.0, 0.0, 0.0, 0.999));
    marker.pose = context.odometry.pose.pose;
    marker.text = FAILURE;
    debug.markers["reference_path_following_failure"].markers.push_back(std::move(marker));
  }
  phase.reset();

  autoware_utils_debug::ScopedTimeTrack turn_st("decide_turn_indicators", *time_keeper_);
  const auto turn_indicators = turn_indicator_decider.decide(context, trajectory);
  previous_trajectory = trajectory;
  return PlannedTrajectory{std::move(trajectory), turn_indicators};
}

}  // namespace autoware::safety_planner::experimental

PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::experimental::ReferencePathFollowingPlanner,
  autoware::safety_planner::TrajectoryPlannerInterface)
