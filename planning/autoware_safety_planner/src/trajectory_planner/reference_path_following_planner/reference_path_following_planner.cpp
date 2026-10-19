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
#include "../../utils/reference_path_smoother.hpp"
#include "../../utils/velocity_optimizer.hpp"
#include "../frenet_sampling_based_planner/compiled_constraints_utils.hpp"
#include "../frenet_sampling_based_planner/constraints_compiler.hpp"

#include <autoware/trajectory/utils/pretty_build.hpp>
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

//! A text at the ego saying why the planner did not do what it was asked
Marker make_failure_marker(
  const PlannerContext & context, const std::string & ns, const char * text)
{
  auto marker = autoware_utils_visualization::create_default_marker(
    "map", context.odometry.header.stamp, ns, 0, Marker::TEXT_VIEW_FACING,
    autoware_utils_visualization::create_marker_scale(0.0, 0.0, 1.0),
    autoware_utils_visualization::create_marker_color(1.0, 0.0, 0.0, 0.999));
  marker.pose = context.odometry.pose.pose;
  marker.text = text;
  return marker;
}

//! The lateral offsets [m] the HARD lateral bounds leave to the reference path at the samples ss
//! ahead of s0: {lower, upper}. At each sample the footprint is laid along the tangent and
//! measured as a rigid rectangle in the frame there (make_boundary_profile), where a shift along
//! the normal is a plain translation in y, so unlike the (s, l) box the bounds hold on a curve.
//! The heading the shift adds is not compensated, as in footprint_sl_box. Behind s0 nothing is
//! bounded
std::pair<std::vector<double>, std::vector<double>> lateral_room_within_hard_bounds(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const std::vector<double> & ss, const double s0, const double clearance_m)
{
  std::vector<const LateralBoundEntry *> hard_bounds;
  for (const auto & bound : compiled_constraints.lateral_bounds) {
    if (compiled_constraints.raw_constraints[bound.raw_index].hardness == Hardness::HARD) {
      hard_bounds.push_back(&bound);
    }
  }
  const auto & vehicle_info = context.vehicle_info;
  const double x_min = vehicle_info.min_longitudinal_offset_m;
  const double x_max = vehicle_info.max_longitudinal_offset_m;
  std::vector<double> lower(ss.size(), -INF);
  std::vector<double> upper(ss.size(), INF);
  for (std::size_t i = 0; i < ss.size(); ++i) {
    if (ss[i] < s0) {
      continue;
    }
    // One bin: only the extreme over the footprint length is needed. The s window is the one of
    // the sampling planner, twice the footprint, which keeps the other leg of a hairpin out
    const auto profile = make_boundary_profile(
      hard_bounds, to_world_pose(context.reference_path, ss[i], 0.0), x_min, x_max,
      ss[i] + 2.0 * x_min, ss[i] + 2.0 * x_max, x_max - x_min);
    upper[i] = profile.left_min - vehicle_info.max_lateral_offset_m - clearance_m;
    lower[i] = profile.right_max - vehicle_info.min_lateral_offset_m + clearance_m;
  }
  return {std::move(lower), std::move(upper)};
}

//! A LINE_LIST of the finite bounds, each side drawn between consecutive samples
MarkerArray make_lateral_room_markers(
  const PathPointTrajectory & path, const std::vector<double> & ss,
  const std::vector<double> & lower, const std::vector<double> & upper,
  const builtin_interfaces::msg::Time & stamp)
{
  auto marker = autoware_utils_visualization::create_default_marker(
    "map", stamp, "reference_path_following_lateral_room", 0, Marker::LINE_LIST,
    autoware_utils_visualization::create_marker_scale(0.1, 0.0, 0.0),
    autoware_utils_visualization::create_marker_color(1.0, 0.5, 0.0, 0.8));
  for (const auto * side : {&lower, &upper}) {
    for (std::size_t i = 0; i + 1 < ss.size(); ++i) {
      if (!std::isfinite((*side)[i]) || !std::isfinite((*side)[i + 1])) {
        continue;
      }
      for (const std::size_t j : {i, i + 1}) {
        const auto pose = to_world_pose(path, ss[j], (*side)[j]);
        geometry_msgs::msg::Point point;
        point.x = pose.position.x();
        point.y = pose.position.y();
        point.z = path.compute(ss[j]).point.pose.position.z;
        marker.points.push_back(point);
      }
    }
  }
  MarkerArray markers;
  markers.markers.push_back(std::move(marker));
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
  auto compiled_constraints = compile_constraint_list(context, constraints);
  phase.reset();

  // The path the velocity is planned on: the reference_path, or where the footprint on it would
  // cross a road_border, the path shifted clear of it. The constraints are then compiled again on
  // the shifted path, since the arc length of every projected view refers to the path it was
  // projected on
  std::optional<PlannerContext> shifted_context;
  if (p.lateral_optimization.enable) {
    phase = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(
      "optimize_lateral_offset", *time_keeper_);
    if (auto shifted = shift_path_within_hard_bounds(context, compiled_constraints, debug)) {
      shifted_context.emplace(context);
      shifted_context->reference_path = std::move(*shifted);
      compiled_constraints = compile_constraint_list(*shifted_context, constraints);
    }
    phase.reset();
  }
  const PlannerContext & ctx = shifted_context ? *shifted_context : context;

  phase = std::make_unique<autoware_utils_debug::ScopedTimeTrack>("plan_velocity", *time_keeper_);
  const auto limits = collect_kinematic_limits(compiled_constraints);
  const double nominal_jerk =
    std::min(limits.j_nom, global_hard_bound_max(compiled_constraints, BoundedQuantity::LON_JERK));

  const double s0 = compute_ego_frenet_state(ctx).s;
  const double s_stop = stop_target_s(ctx, compiled_constraints, params_.trajectory_horizon_s, s0);
  const double v_ego = std::max(0.0, context.odometry.twist.twist.linear.x);

  auto path = ctx.reference_path;
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
    debug.markers["reference_path_following_failure"].markers.push_back(
      make_failure_marker(context, "reference_path_following_failure", FAILURE));
  }
  phase.reset();

  autoware_utils_debug::ScopedTimeTrack turn_st("decide_turn_indicators", *time_keeper_);
  const auto turn_indicators = turn_indicator_decider.decide(context, trajectory);
  previous_trajectory = trajectory;
  return PlannedTrajectory{std::move(trajectory), turn_indicators};
}

std::optional<PathPointTrajectory> ReferencePathFollowingPlanner::shift_path_within_hard_bounds(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  TrajectoryPlannerDebug & debug) const
{
  //! [m] sample spacing, 1 m as in the elastic band; an uneven spacing makes the QP unstable
  constexpr double SAMPLE_INTERVAL_M = 1.0;
  const auto & p = params_.reference_path_following_planner.lateral_optimization;
  const auto & path = context.reference_path;
  const auto & stamp = context.odometry.header.stamp;

  std::vector<double> ss;
  for (double s = 0.0; s < path.length() - 0.5 * SAMPLE_INTERVAL_M; s += SAMPLE_INTERVAL_M) {
    ss.push_back(s);
  }
  ss.push_back(path.length());
  if (ss.size() < 4) {
    return std::nullopt;  // the end conditions of the QP leave nothing to optimize
  }

  const double s0 = compute_ego_frenet_state(context).s;
  auto [lower, upper] =
    lateral_room_within_hard_bounds(context, compiled_constraints, ss, s0, p.clearance_m);
  // The end of the path is held on the centerline (it is the goal when the goal is in the window),
  // so the run of samples up to it whose bounds exclude the centerline admits it instead: a goal
  // within the clearance of a curb (on a shoulder) would otherwise make the QP infeasible and take
  // the shift everywhere else with it. The path is followed as it is there, as before
  for (std::size_t i = ss.size(); i-- > 0 && (lower[i] > 0.0 || upper[i] < 0.0);) {
    lower[i] = std::min(lower[i], 0.0);
    upper[i] = std::max(upper[i], 0.0);
  }
  debug.markers["reference_path_following_lateral_room"] =
    make_lateral_room_markers(path, ss, lower, upper, stamp);
  bool centerline_is_clear = true;
  std::vector<double> curvature;
  curvature.reserve(ss.size());
  double curvature_max = 0.0;
  for (std::size_t i = 0; i < ss.size(); ++i) {
    centerline_is_clear = centerline_is_clear && lower[i] <= 0.0 && upper[i] >= 0.0;
    curvature.push_back(path.curvature(ss[i]));
    curvature_max = std::max(curvature_max, std::abs(curvature.back()));
  }
  const double steerable_curvature = p.curvature_margin *
                                     std::tan(context.vehicle_info.max_steer_angle_rad) /
                                     context.vehicle_info.wheel_base_m;
  if (centerline_is_clear && curvature_max <= steerable_curvature) {
    return std::nullopt;  // the common case: nothing to shift, and the output is left untouched
  }

  // A centerline beyond the steering (a kink between lanelets in a rotary) is shifted to within
  // it where the bounds leave the room. Where they do not, the whole QP is infeasible, and it is
  // solved again no tighter than the centerline already is, so the shift off the bounds elsewhere
  // is not lost with it. The limit is one scalar, so that retry admits the corner's curvature
  // everywhere; a per sample limit is not worth the change to the QP
  const std::vector<double> centerline(ss.size(), 0.0);
  auto offsets = optimize_goal_shift(
    centerline, curvature, SAMPLE_INTERVAL_M, steerable_curvature, lower, upper,
    p.deviation_weight);
  if (!offsets && curvature_max > steerable_curvature) {
    offsets = optimize_goal_shift(
      centerline, curvature, SAMPLE_INTERVAL_M, curvature_max * 1.001, lower, upper,
      p.deviation_weight);
  }
  std::optional<PathPointTrajectory> shifted;
  if (offsets) {
    shifted =
      autoware::experimental::trajectory::pretty_build(shift_path_points(path, ss, *offsets));
  }
  if (!shifted) {
    // The reference_path is followed as it is, since outputting only the ego point here would stop
    // the ego wherever the road is narrower than the footprint with its clearance
    constexpr auto FAILURE = "lateral QP did not solve, following the reference path";
    RCLCPP_WARN_THROTTLE(logger(), steady_clock(), 5000, "%s", FAILURE);
    debug.markers["reference_path_following_failure"].markers.push_back(
      make_failure_marker(context, "reference_path_following_lateral_failure", FAILURE));
    return std::nullopt;
  }

  auto marker = autoware_utils_visualization::create_default_marker(
    "map", stamp, "reference_path_following_shifted_path", 0, Marker::LINE_STRIP,
    autoware_utils_visualization::create_marker_scale(0.15, 0.0, 0.0),
    autoware_utils_visualization::create_marker_color(1.0, 0.0, 1.0, 0.8));
  for (const auto & point : shifted->restore()) {
    marker.points.push_back(point.point.pose.position);
  }
  debug.markers["reference_path_following_shifted_path"].markers.push_back(std::move(marker));
  return shifted;
}

}  // namespace autoware::safety_planner::experimental

PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::experimental::ReferencePathFollowingPlanner,
  autoware::safety_planner::TrajectoryPlannerInterface)
