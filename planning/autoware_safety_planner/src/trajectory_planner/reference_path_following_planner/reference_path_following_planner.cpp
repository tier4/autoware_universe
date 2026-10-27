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
#include <autoware_utils_math/normalization.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
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

//! The offsets at the samples ss of path where its normal there crosses previous, the nearest
//! crossing to the centerline; NaN where it does not cross. Measured along the normal rather than
//! by projecting the points of previous onto path, which puts their feet off on a curve
std::vector<double> project_onto_normals(
  const PathPointTrajectory & path, const std::vector<double> & ss,
  const PathPointTrajectory & previous)
{
  const auto points = previous.restore();
  std::vector<double> offsets(ss.size(), std::numeric_limits<double>::quiet_NaN());
  for (std::size_t i = 0; i < ss.size(); ++i) {
    const auto & origin = path.compute(ss[i]).point.pose.position;
    const double yaw = path.azimuth(ss[i]);
    // (along the tangent, along the normal) of a point of previous
    const auto to_local = [&](const geometry_msgs::msg::Point & q) {
      const double dx = q.x - origin.x;
      const double dy = q.y - origin.y;
      return std::pair{
        std::cos(yaw) * dx + std::sin(yaw) * dy, -std::sin(yaw) * dx + std::cos(yaw) * dy};
    };
    for (std::size_t j = 0; j + 1 < points.size(); ++j) {
      const auto [x0, y0] = to_local(points[j].point.pose.position);
      const auto [x1, y1] = to_local(points[j + 1].point.pose.position);
      if ((x0 > 0.0) == (x1 > 0.0)) {
        continue;
      }
      const double l = y0 + x0 / (x0 - x1) * (y1 - y0);
      if (std::isnan(offsets[i]) || std::abs(l) < std::abs(offsets[i])) {
        offsets[i] = l;
      }
    }
  }
  return offsets;
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
      normal_previous_trajectory_, normal_previous_path_, result.normal_debug);
  }
  const bool cautious_differs = std::any_of(
    input.cautious_constraints.begin(), input.cautious_constraints.end(),
    [](const Constraint & constraint) { return constraint.certainty == Certainty::POSSIBLE; });
  if (cautious_differs) {
    autoware_utils_debug::ScopedTimeTrack side_st("plan_cautious", *time_keeper_);
    result.cautious_trajectory = plan_one_side(
      cautious_turn_indicator_decider_, input.context, input.cautious_constraints,
      cautious_previous_trajectory_, cautious_previous_path_, result.cautious_debug);
  } else {
    // Not left empty, since the node publishes the cautious candidate every cycle
    result.cautious_trajectory = result.normal_trajectory;
    result.cautious_debug = result.normal_debug;
    cautious_previous_trajectory_ = normal_previous_trajectory_;
    cautious_previous_path_ = normal_previous_path_;
  }
  return result;
}

PlannedTrajectory ReferencePathFollowingPlanner::plan_one_side(
  TurnIndicatorDecider & turn_indicator_decider, const PlannerContext & context,
  const std::vector<Constraint> & constraints, std::optional<Trajectory> & previous_trajectory,
  std::optional<PathPointTrajectory> & previous_path, TrajectoryPlannerDebug & debug) const
{
  const auto & p = params_.reference_path_following_planner;
  const auto & vo = p.velocity_optimizer;

  auto phase = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(
    "compile_constraint_list", *time_keeper_);
  auto compiled_constraints = compile_constraint_list(context, constraints);
  phase.reset();

  // The path the velocity is planned on: the reference_path, or the path shifted from the ego back
  // to it and clear of the road_borders. The constraints are then compiled again on the shifted
  // path, since the arc length of every projected view refers to the path it was projected on
  std::optional<PlannerContext> shifted_context;
  bool lateral_failed = false;
  if (p.lateral_optimization.enable) {
    phase = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(
      "optimize_lateral_offset", *time_keeper_);
    if (
      auto shifted =
        shift_path_within_hard_bounds(context, compiled_constraints, previous_path, debug)) {
      shifted_context.emplace(context);
      shifted_context->reference_path = std::move(*shifted);
      compiled_constraints = compile_constraint_list(*shifted_context, constraints);
    } else {
      lateral_failed = true;
    }
    previous_path =
      shifted_context ? std::make_optional(shifted_context->reference_path) : std::nullopt;
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
  const double a_ego = context.acceleration.accel.accel.linear.x;
  auto initial = calc_initial_motion(
    v_ego, a_ego, previous_trajectory, context.odometry.pose.pose.position,
    s_stop - s0 < vo.resolution_m ? 0.0 : params_.engage_velocity.velocity_hard_mps);
  // Beyond the nominal range the planned acceleration is a violation of the soft bound (a brake
  // for a kink in the path), not a decision; pinned as the start it compounds from cycle to cycle
  if (initial.a && (*initial.a < limits.a_nom_min || *initial.a > limits.a_nom_max)) {
    initial.a = a_ego;
  }
  const auto num_points =
    static_cast<std::size_t>(std::round(params_.trajectory_horizon_s / p.time_step_s)) + 1;
  std::optional<TrajectoryPoints> points;
  if (!lateral_failed) {
    points = plan_velocity(path, s0, initial, vp_params, num_points, p.time_step_s);
  }
  phase.reset();

  phase = std::make_unique<autoware_utils_debug::ScopedTimeTrack>("to_trajectory", *time_keeper_);
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = context.odometry.header.stamp;
  if (points) {
    trajectory.points = std::move(*points);
  } else {
    const char * FAILURE =
      lateral_failed ? "lateral QP did not solve" : "velocity QP did not solve";
    RCLCPP_WARN_THROTTLE(logger(), steady_clock(), 5000, "%s", FAILURE);
    // No fallback on purpose, as the MPPI planner: the failure has to be visible downstream, so
    // only the ego point is output and the reason is recorded as a marker. That holds for the
    // lateral QP too, although it stops the ego where the road is narrower than the footprint with
    // its clearance
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
  const std::optional<PathPointTrajectory> & previous_path, TrajectoryPlannerDebug & debug) const
{
  //! [m] sample spacing, 1 m as in the elastic band; an uneven spacing makes the QP unstable
  constexpr double SAMPLE_INTERVAL_M = 1.0;
  const auto & p = params_.reference_path_following_planner.lateral_optimization;
  const auto & path = context.reference_path;
  const auto & stamp = context.odometry.header.stamp;

  // The QP starts at the ego, at its offset and heading, so the path also connects the ego back to
  // the centerline wherever it is off it
  const auto ego = compute_ego_frenet_state(context);
  std::vector<double> ss;
  for (double s = ego.s; s < path.length() - 0.5 * SAMPLE_INTERVAL_M; s += SAMPLE_INTERVAL_M) {
    ss.push_back(s);
  }
  ss.push_back(path.length());
  // l' = (1 - k l) tan(dyaw), as the tangent of the shifted path is (1 - k l) t + l' n
  const double ego_slope =
    (1.0 - path.curvature(ego.s) * ego.l) *
    std::tan(
      autoware_utils_math::normalize_radian(
        autoware_utils_geometry::get_rpy(context.odometry.pose.pose).z - path.azimuth(ego.s)));
  // The path starts at the curvature the wheels are at, and turns them no faster than the steer
  // rate allows at the ego speed: a return to the centerline that needs full opposite lock right
  // at the ego is otherwise planned, and plan_velocity then brakes to a crawl for it
  const double wheel_base = context.vehicle_info.wheel_base_m;
  const double ego_curvature = std::tan(context.steering.steering_tire_angle) / wheel_base;
  //! [m/s] below this the steer rate per meter is not tightened further
  constexpr double MIN_SPEED_FOR_STEER_RATE_MPS = 1.0;
  //! The spline rebuilt through the samples overshoots a step of the discrete curvature by about
  //! half, and plan_velocity reads the steer rate off that spline on the same 1 m grid
  constexpr double CURVATURE_RATE_MARGIN = 0.5;
  const double max_curvature_rate =
    CURVATURE_RATE_MARGIN *
    global_hard_bound_max(compiled_constraints, BoundedQuantity::STEER_RATE) /
    (std::max(context.odometry.twist.twist.linear.x, MIN_SPEED_FOR_STEER_RATE_MPS) * wheel_base);
  // l'' at the ego for the path to have ego_curvature there, from the curvature of the offset path
  // (a^2 k_ref + a l'' + 2 k_ref l'^2) / (a^2 + l'^2)^(3/2) with a = 1 - k_ref l
  const double ego_second_derivative = [&]() {
    const double k_ref = path.curvature(ego.s);
    const double a = 1.0 - k_ref * ego.l;
    return (ego_curvature * std::pow(a * a + ego_slope * ego_slope, 1.5) - a * a * k_ref -
            2.0 * k_ref * ego_slope * ego_slope) /
           a;
  }();

  std::optional<std::vector<double>> offsets;
  if (ss.size() < 4) {
    // Too short for the end conditions of the QP (the ego within ~3 m of the path end): the path
    // is continued straight along the ego heading, and is not connected to the goal
    offsets.emplace();
    for (const double s : ss) {
      offsets->push_back(ego.l + (s - ego.s) * ego_slope);
    }
  } else {
    std::vector<double> nominal(ss.size(), 0.0);
    nominal[0] = ego.l;
    nominal[1] = ego.l + ego_slope * (ss[1] - ss[0]);

    auto [lower, upper] =
      lateral_room_within_hard_bounds(context, compiled_constraints, ss, ego.s, p.clearance_m);
    // The first two samples are pinned to the ego, which may already be inside the clearance
    for (std::size_t i = 0; i < 2; ++i) {
      lower[i] = std::min(lower[i], nominal[i]);
      upper[i] = std::max(upper[i], nominal[i]);
    }
    // The end of the path is held on the centerline (it is the goal when the goal is in the
    // window), so the run of samples up to it whose bounds exclude the centerline admits it
    // instead: a goal within the clearance of a curb (on a shoulder) would otherwise make the QP
    // infeasible and take the shift everywhere else with it. The path is followed as it is there
    for (std::size_t i = ss.size(); i-- > 0 && (lower[i] > 0.0 || upper[i] < 0.0);) {
      lower[i] = std::min(lower[i], 0.0);
      upper[i] = std::max(upper[i], 0.0);
    }
    debug.markers["reference_path_following_lateral_room"] =
      make_lateral_room_markers(path, ss, lower, upper, stamp);
    std::vector<double> curvature;
    curvature.reserve(ss.size());
    double curvature_max = 0.0;
    for (std::size_t i = 0; i < ss.size(); ++i) {
      curvature.push_back(path.curvature(ss[i]));
      curvature_max = std::max(curvature_max, std::abs(curvature.back()));
    }
    const double steerable_curvature = p.curvature_margin *
                                       std::tan(context.vehicle_info.max_steer_angle_rad) /
                                       context.vehicle_info.wheel_base_m;

    // A centerline beyond the steering (a kink between lanelets in a rotary) is shifted to within
    // it where the bounds leave the room. Where they do not, the whole QP is infeasible, and it is
    // solved again no tighter than the centerline already is, so the shift off the bounds
    // elsewhere is not lost with it. The limit is one scalar, so that retry admits the corner's
    // curvature everywhere; a per sample limit is not worth the change to the QP. Last, the end
    // is left free: a path from the ego that does not reach the goal is taken over none
    const double relaxed_curvature = std::max(steerable_curvature, curvature_max * 1.001);
    // The previous output measured on this cycle's reference_path, so that the shift does not
    // jump between cycles where the bounds leave it a choice
    const auto previous =
      previous_path ? project_onto_normals(path, ss, *previous_path) : std::vector<double>{};
    const auto solve =
      [&](const double max_curvature, const bool hold_end, const bool continuous_steer) {
        return optimize_goal_shift(
          nominal, curvature, SAMPLE_INTERVAL_M, max_curvature, lower, upper, p.deviation_weight,
          hold_end, previous, p.stability_weight,
          continuous_steer ? std::make_optional(ego_curvature) : std::nullopt,
          continuous_steer ? max_curvature_rate : INF);
      };
    offsets = solve(steerable_curvature, true, true);
    if (!offsets && curvature_max > steerable_curvature) {
      offsets = solve(relaxed_curvature, true, true);
    }
    if (!offsets) {
      offsets = solve(relaxed_curvature, false, true);
    }
    // The steer continuity is given up last: the steer rate at speed can leave no room between
    // the bounds (the wheels turned towards a curb close ahead)
    if (!offsets) {
      offsets = solve(relaxed_curvature, false, false);
    }
  }
  std::optional<PathPointTrajectory> shifted;
  if (offsets) {
    // Behind the ego the path is continued along the ego heading at the ego curvature, so that it
    // has no step at the ego to catch the projection of the ego onto it and the curvature is
    // continuous there. reference_path.backward_length_m keeps the ego meters from the rear end,
    // so the original points left before the first sample here are not near it
    std::vector<double> arc_lengths;
    std::vector<double> all_offsets;
    for (double s = ego.s - SAMPLE_INTERVAL_M; s > 0.5 * SAMPLE_INTERVAL_M;
         s -= SAMPLE_INTERVAL_M) {
      arc_lengths.push_back(s);
    }
    if (ego.s > 0.5 * SAMPLE_INTERVAL_M) {
      arc_lengths.push_back(0.0);
    }
    std::reverse(arc_lengths.begin(), arc_lengths.end());
    // On the grid of the QP, whose slope is the forward difference and whose second difference
    // is the curvature term: the samples behind continue both, so the spline sees no bend at the
    // ego
    for (const double s : arc_lengths) {
      const double k = (s - ego.s) / SAMPLE_INTERVAL_M;
      all_offsets.push_back(
        ego.l + k * ego_slope * SAMPLE_INTERVAL_M +
        0.5 * k * (k - 1.0) * ego_second_derivative * SAMPLE_INTERVAL_M * SAMPLE_INTERVAL_M);
    }
    arc_lengths.insert(arc_lengths.end(), ss.begin(), ss.end());
    all_offsets.insert(all_offsets.end(), offsets->begin(), offsets->end());
    shifted = autoware::experimental::trajectory::pretty_build(
      shift_path_points(path, arc_lengths, all_offsets));
  }
  if (!shifted) {
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
