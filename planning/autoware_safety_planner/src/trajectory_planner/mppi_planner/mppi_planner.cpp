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

#include "mppi_planner.hpp"

#include "../../utils/frenet_utils.hpp"
#include "../frenet_sampling_based_planner/compiled_constraints_utils.hpp"

#include <autoware/mppi_optimizer/detail/trajectory_utils.hpp>
#include <autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params_conversion.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
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

using autoware::mppi_optimizer::FirstOrderDubinsMppiKinematicLimits;
using autoware::mppi_optimizer::Segment;
using autoware::mppi_optimizer::detail::kMppiDt;
using autoware::mppi_optimizer::detail::kMppiHorizon;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;

rclcpp::Logger logger()
{
  return rclcpp::get_logger("safety_planner.mppi_planner");
}

rclcpp::Clock & steady_clock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

double to_seconds(const builtin_interfaces::msg::Duration & d)
{
  return rclcpp::Duration(d).seconds();
}

//! The VELOCITY limit in effect at the arc length s, global bounds and speed limit zones together
double velocity_limit_at(
  const CompiledConstraints & compiled_constraints, const KinematicLimits & limits, const double s)
{
  double v_max = limits.v_hard;
  for (const auto & bound : compiled_constraints.scalar_bounds) {
    if (bound.quantity == BoundedQuantity::VELOCITY && bound.s0 <= s && s <= bound.s1) {
      v_max = std::min(v_max, bound.max);
    }
  }
  return v_max;
}

//! Upper bound of the global ScalarBound constraints on quantity; INF when there is none
double global_bound_max(
  const CompiledConstraints & compiled_constraints, const BoundedQuantity quantity)
{
  double max = INF;
  for (const auto & bound : compiled_constraints.scalar_bounds) {
    if (bound.quantity == quantity && bound.s0 == -INF && bound.s1 == INF) {
      max = std::min(max, bound.max);
    }
  }
  return max;
}

TrajectoryPoint make_ego_point(const PlannerContext & context)
{
  const double v = std::max(0.0, context.odometry.twist.twist.linear.x);
  const double steer = context.steering.steering_tire_angle;
  TrajectoryPoint point;
  point.pose = context.odometry.pose.pose;
  point.longitudinal_velocity_mps = static_cast<float>(v);
  point.acceleration_mps2 = static_cast<float>(context.acceleration.accel.accel.linear.x);
  point.front_wheel_angle_rad = static_cast<float>(steer);
  point.heading_rate_rps =
    static_cast<float>(v * std::tan(steer) / context.vehicle_info.wheel_base_m);
  return point;
}

void append_segments(const LineString2d & polyline, std::vector<Segment> & segments)
{
  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    segments.push_back(
      Segment{
        static_cast<float>(polyline[i].x()), static_cast<float>(polyline[i].y()),
        static_cast<float>(polyline[i + 1].x()), static_cast<float>(polyline[i + 1].y())});
  }
}

//! The segments exactly as MPPI receives them (after simplification): hard in red, soft in
//! yellow, with the vertices as points so the remaining vertex density can be read off
MarkerArray make_boundary_markers(
  const std::vector<Segment> & road_borders, const std::vector<Segment> & drivable_area,
  const builtin_interfaces::msg::Time & stamp, const double z)
{
  using autoware_utils_visualization::create_default_marker;
  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;

  auto hard_marker = create_default_marker(
    "map", stamp, "mppi_road_borders", 0, Marker::LINE_LIST, create_marker_scale(0.1, 0.0, 0.0),
    create_marker_color(1.0, 0.2, 0.2, 0.8));
  auto soft_marker = create_default_marker(
    "map", stamp, "mppi_drivable_area", 0, Marker::LINE_LIST, create_marker_scale(0.1, 0.0, 0.0),
    create_marker_color(1.0, 1.0, 0.2, 0.8));
  auto vertex_marker = create_default_marker(
    "map", stamp, "mppi_boundary_vertices", 0, Marker::SPHERE_LIST,
    create_marker_scale(0.25, 0.25, 0.25), create_marker_color(1.0, 1.0, 1.0, 0.8));

  const auto append = [&](const std::vector<Segment> & segments, Marker & marker) {
    for (const auto & segment : segments) {
      geometry_msgs::msg::Point p0;
      p0.x = segment.x0;
      p0.y = segment.y0;
      p0.z = z;
      geometry_msgs::msg::Point p1;
      p1.x = segment.x1;
      p1.y = segment.y1;
      p1.z = z;
      marker.points.push_back(p0);
      marker.points.push_back(p1);
      vertex_marker.points.push_back(p0);
      vertex_marker.points.push_back(p1);
    }
  };
  append(road_borders, hard_marker);
  append(drivable_area, soft_marker);

  MarkerArray markers;
  for (auto & marker : {hard_marker, soft_marker, vertex_marker}) {
    if (!marker.points.empty()) {
      markers.markers.push_back(marker);
    }
  }
  return markers;
}

//! MPPI takes an oriented box per object, so the body-frame shape is reduced to its bounding
//! box and the motion to the speed between the first two waypoints (it extrapolates at constant
//! velocity; the later waypoints are not representable)
TrackedObject to_tracked_object(const KeepOut & body)
{
  TrackedObject object;
  double x_min = std::numeric_limits<double>::infinity();
  double x_max = -x_min;
  double y_min = x_min;
  double y_max = -x_min;
  for (const auto & p : body.shape.outer()) {
    x_min = std::min(x_min, p.x());
    x_max = std::max(x_max, p.x());
    y_min = std::min(y_min, p.y());
    y_max = std::max(y_max, p.y());
  }
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = x_max - x_min;
  object.shape.dimensions.y = y_max - y_min;

  const auto & first = body.waypoints.front();
  auto & pose = object.kinematics.pose_with_covariance.pose;
  // The box center is the shape's bounding-box center in the body frame, offset from the pose
  const double c = std::cos(first.pose.yaw);
  const double s = std::sin(first.pose.yaw);
  const double cx = 0.5 * (x_min + x_max);
  const double cy = 0.5 * (y_min + y_max);
  pose.position.x = first.pose.position.x() + c * cx - s * cy;
  pose.position.y = first.pose.position.y() + s * cx + c * cy;
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(first.pose.yaw);

  if (body.waypoints.size() >= 2) {
    const auto & second = body.waypoints[1];
    const double dt = second.t - first.t;
    if (dt > 1e-3) {
      const double dx = second.pose.position.x() - first.pose.position.x();
      const double dy = second.pose.position.y() - first.pose.position.y();
      // Signed along the heading: a body moving against its heading gets a negative speed
      object.kinematics.twist_with_covariance.twist.linear.x = (c * dx + s * dy) / dt;
    }
  }
  object.kinematics.orientation_availability =
    autoware_perception_msgs::msg::TrackedObjectKinematics::AVAILABLE;
  return object;
}

}  // namespace

void MppiPlanner::on_initialize(
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper, const Params & params)
{
  TrajectoryPlannerInterface::on_initialize(time_keeper, params);
  const TurnSignalParams turn_signal_params{
    params.turn_signal.search_distance, params.turn_signal.min_blink_duration,
    params.turn_signal.stopped_velocity_threshold, params.turn_signal.heading_align_threshold};
  normal_turn_indicator_decider_.update_params(turn_signal_params);
  cautious_turn_indicator_decider_.update_params(turn_signal_params);
  normal_optimizer_ = std::make_unique<MppiInterface>();
  cautious_optimizer_ = std::make_unique<MppiInterface>();
  constexpr std::size_t kBoundaryCacheSize = 256;
  boundary_simplifier_ = std::make_unique<BoundarySimplifier>(
    params.mppi_planner.boundary.simplify_tolerance_m, kBoundaryCacheSize);
}

TrajectoryPlannerResult MppiPlanner::plan_trajectories(const TrajectoryPlannerInput & input)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  TrajectoryPlannerResult result;
  {
    autoware_utils_debug::ScopedTimeTrack side_st("plan_normal", *time_keeper_);
    result.normal_trajectory = plan_one_side(
      *normal_optimizer_, normal_turn_indicator_decider_, input.context, input.normal_constraints,
      result.normal_debug);
  }
  const bool cautious_differs = std::any_of(
    input.cautious_constraints.begin(), input.cautious_constraints.end(),
    [](const Constraint & constraint) { return constraint.certainty == Certainty::POSSIBLE; });
  if (cautious_differs) {
    autoware_utils_debug::ScopedTimeTrack side_st("plan_cautious", *time_keeper_);
    result.cautious_trajectory = plan_one_side(
      *cautious_optimizer_, cautious_turn_indicator_decider_, input.context,
      input.cautious_constraints, result.cautious_debug);
  } else {
    // Not left empty: the node would publish an ego-only cautious candidate every cycle
    result.cautious_trajectory = result.normal_trajectory;
    result.cautious_debug = result.normal_debug;
  }
  return result;
}

PlannedTrajectory MppiPlanner::plan_one_side(
  MppiInterface & optimizer, TurnIndicatorDecider & turn_indicator_decider,
  const PlannerContext & context, const std::vector<Constraint> & constraints,
  TrajectoryPlannerDebug & debug)
{
  auto compile_st = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(
    "compile_constraint_list", *time_keeper_);
  const auto compiled_constraints = compile_constraint_list(context, constraints);
  compile_st.reset();
  auto reference_st = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(
    "make_reference_trajectory", *time_keeper_);
  auto trajectory = make_reference_trajectory(context, compiled_constraints);
  reference_st.reset();
  const PathProjector projector(context.reference_path);
  if (
    const auto failure =
      refine(optimizer, context, compiled_constraints, projector, trajectory, debug)) {
    RCLCPP_WARN_THROTTLE(logger(), steady_clock(), 5000, "MPPI failed: %s", failure->c_str());
    // No fallback on purpose: the failure has to be visible downstream to measure the failure
    // rate, so only the ego point (points[0]) is kept and the reason is recorded as a marker
    trajectory.points.resize(1);
    auto marker = autoware_utils_visualization::create_default_marker(
      "map", context.odometry.header.stamp, "mppi_failure", 0, Marker::TEXT_VIEW_FACING,
      autoware_utils_visualization::create_marker_scale(0.0, 0.0, 1.0),
      autoware_utils_visualization::create_marker_color(1.0, 0.0, 0.0, 0.999));
    marker.pose = context.odometry.pose.pose;
    marker.text = *failure;
    debug.markers["mppi_failure"].markers.push_back(std::move(marker));
  }
  autoware_utils_debug::ScopedTimeTrack turn_st("decide_turn_indicators", *time_keeper_);
  const auto turn_indicators = turn_indicator_decider.decide(context, trajectory);
  return PlannedTrajectory{std::move(trajectory), turn_indicators};
}

Trajectory MppiPlanner::make_reference_trajectory(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints) const
{
  const auto & path = context.reference_path;
  const auto limits = collect_kinematic_limits(compiled_constraints);
  const double wheel_base_m = context.vehicle_info.wheel_base_m;
  const double s_max = path.length();
  const double s0 = compute_ego_frenet_state(context).s;
  const double horizon_s = kMppiHorizon * kMppiDt;
  // Stop at the goal, or at the first stop bar closed within the horizon: the rear axle arc
  // length at which the footprint front just touches it
  double s_stop = s_max;
  for (const auto & stop_bar : compiled_constraints.stop_bars) {
    if (stop_bar.time.t1 < 0.0 || stop_bar.time.t0 > horizon_s) {
      continue;
    }
    s_stop = std::min(s_stop, stop_bar.s_stop - context.vehicle_info.max_longitudinal_offset_m);
  }
  s_stop = std::max(s_stop, s0);
  // The output is checked against the hard LAT_ACCEL bound; a reference riding on it would have
  // MPPI's overshoot rejected every cycle
  const double lat_accel = std::min(
    params_.mppi_planner.reference.lat_accel_mps2,
    global_bound_max(compiled_constraints, BoundedQuantity::LAT_ACCEL));
  const double steer_rate = global_bound_max(compiled_constraints, BoundedQuantity::STEER_RATE);

  // v(s) on a grid from the ego to the end of the path: the pointwise caps, then a backward pass
  // at the comfortable deceleration so that a lower cap ahead is reached by braking, then a
  // forward pass from the ego speed at the comfortable acceleration
  constexpr double RES_M = 0.5;
  const auto n = static_cast<std::size_t>(std::ceil(std::max(s_max - s0, 0.0) / RES_M)) + 1;
  std::vector<double> s_grid(n);
  std::vector<double> v_grid(n);
  double prev_steer = std::atan(path.curvature(s0) * wheel_base_m);
  for (std::size_t i = 0; i < n; ++i) {
    const double s = std::min(s0 + static_cast<double>(i) * RES_M, s_max);
    s_grid[i] = s;
    const double kappa = path.curvature(s);
    const double steer = std::atan(kappa * wheel_base_m);
    double v_cap = s >= s_stop ? 0.0 : velocity_limit_at(compiled_constraints, limits, s);
    if (std::abs(kappa) > 1e-6) {
      v_cap = std::min(v_cap, std::sqrt(lat_accel / std::abs(kappa)));
    }
    const double steer_grad = std::abs(steer - prev_steer) / RES_M;
    if (i > 0 && steer_grad > 1e-6) {
      v_cap = std::min(v_cap, steer_rate / steer_grad);
    }
    prev_steer = steer;
    v_grid[i] = v_cap;
  }
  // Kept below the hard limit MPPI brakes with: its acceleration lags the reference by the input
  // delay, and with no headroom it never catches up, so the crawl clamp in refine cuts the speed
  constexpr double DECEL_HEADROOM_RATIO = 0.8;
  const double a_dec =
    std::min(std::abs(limits.a_nom_min), DECEL_HEADROOM_RATIO * std::abs(limits.a_hard_min));
  const double a_acc = limits.a_nom_max;
  for (std::size_t i = n - 1; i-- > 0;) {
    const double ds = s_grid[i + 1] - s_grid[i];
    v_grid[i] = std::min(v_grid[i], std::sqrt(v_grid[i + 1] * v_grid[i + 1] + 2.0 * a_dec * ds));
  }
  // A stopped ego that could not even reach the engage speed before the stop stays stopped;
  // the sqrt(2 a ds) cap would otherwise creep it up to the stop bar, where it violates the bar
  const double engage_mps = params_.engage_velocity.velocity_hard_mps;
  const double v_ego = std::max(0.0, context.odometry.twist.twist.linear.x);
  if (v_ego < engage_mps && v_grid.front() < engage_mps) {
    std::fill(v_grid.begin(), v_grid.end(), 0.0);
  }
  double v_prev = v_ego;
  for (std::size_t i = 0; i < n; ++i) {
    const double ds = i == 0 ? 0.0 : s_grid[i] - s_grid[i - 1];
    v_grid[i] = std::min(v_grid[i], std::sqrt(v_prev * v_prev + 2.0 * a_acc * ds));
    v_prev = v_grid[i];
  }
  // v^2 is linear in s under constant acceleration; interpolating v would flatten the launch
  const auto v_at = [&](const double s) {
    if (n < 2 || s <= s_grid.front()) {
      return v_grid.front();
    }
    if (s >= s_grid.back()) {
      return v_grid.back();
    }
    const auto i = std::min(static_cast<std::size_t>((s - s0) / RES_M), n - 2);
    const double r = (s - s_grid[i]) / (s_grid[i + 1] - s_grid[i]);
    return std::sqrt(v_grid[i] * v_grid[i] * (1.0 - r) + v_grid[i + 1] * v_grid[i + 1] * r);
  };

  // The path ends at the foot of the goal, which may lie off the centerline (a shoulder, ...).
  // The lateral offset blends linearly into the one of the goal over [2B, B] before the end and
  // holds it for the last B, as the sampling planner did; starting only B ahead needs an ever
  // larger curvature as the distance shrinks
  // A goal the path was not connected to (e.g. in the neighboring lane) is not approached
  // laterally either
  constexpr double GOAL_CONNECTED_RADIUS_M = 1.0;
  const double blend_m = params_.mppi_planner.reference.goal_lateral_blend_m;
  const double l_goal =
    autoware_utils_geometry::calc_distance2d(
      path.compute(s_max).point.pose.position, context.goal_pose.position) < GOAL_CONNECTED_RADIUS_M
      ? lateral_offset_at(
          path, s_max, Point2d{context.goal_pose.position.x, context.goal_pose.position.y})
      : 0.0;
  const auto lateral_at = [&](const double s) {
    return l_goal * std::clamp((2.0 * blend_m - (s_max - s)) / blend_m, 0.0, 1.0);
  };

  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = context.odometry.header.stamp;
  trajectory.points.reserve(kMppiHorizon + 1);
  trajectory.points.push_back(make_ego_point(context));
  const double z = context.odometry.pose.pose.position.z;
  double s = s0;
  double v = v_grid.front();
  for (int k = 0; k <= kMppiHorizon; ++k) {
    // v(s) alone cannot leave a standstill (v(s0) = 0 holds s), so the step is taken at the
    // comfortable acceleration and capped by the profile at the predicted position
    const double s_pred = s + v * kMppiDt + 0.5 * a_acc * kMppiDt * kMppiDt;
    const double v_next = std::min(v_at(s_pred), v + a_acc * kMppiDt);
    if (k > 0) {
      const double l = lateral_at(s);
      const auto pose = to_world_pose(path, s, l);
      const double kappa = path.curvature(s);
      const double dl_ds = (lateral_at(std::min(s + RES_M, s_max)) - l) / RES_M;
      TrajectoryPoint point;
      point.time_from_start = rclcpp::Duration::from_seconds(k * kMppiDt);
      point.pose.position.x = pose.position.x();
      point.pose.position.y = pose.position.y();
      point.pose.position.z = z;
      point.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(
        pose.yaw + std::atan2(dl_ds, 1.0 - kappa * l));
      point.longitudinal_velocity_mps = static_cast<float>(v);
      point.acceleration_mps2 = static_cast<float>((v_next - v) / kMppiDt);
      point.heading_rate_rps = static_cast<float>(v * kappa);
      point.front_wheel_angle_rad = static_cast<float>(std::atan(kappa * wheel_base_m));
      trajectory.points.push_back(point);
    }
    s = std::min(s + 0.5 * (v + v_next) * kMppiDt, s_max);
    v = v_next;
  }
  return trajectory;
}

void MppiPlanner::ensure_initialized(
  MppiInterface & optimizer, const PlannerContext & context,
  const CompiledConstraints & compiled_constraints)
{
  if (optimizer.isInitialized()) {
    return;
  }
  const auto & p = params_.mppi_planner;

  // setVehicleParams tears the GPU state down, so the steer bounds are read once here rather
  // than every cycle; vehicle_kinematics emits constant values
  auto vehicle = autoware::mppi_optimizer::makeVehicleParams(context.vehicle_info);
  vehicle.acc_time_constant = static_cast<float>(p.vehicle.acc_time_constant_s);
  vehicle.steer_time_constant = static_cast<float>(p.vehicle.steer_time_constant_s);
  vehicle.acc_time_delay = static_cast<float>(p.vehicle.acc_time_delay_s);
  vehicle.steer_time_delay = static_cast<float>(p.vehicle.steer_time_delay_s);
  vehicle.vel_rate_lim = static_cast<float>(p.vehicle.accel_lim_mps2);
  vehicle.steer_rate_lim = static_cast<float>(p.vehicle.steer_rate_lim_radps);
  for (const auto & bound : compiled_constraints.scalar_bounds) {
    if (!(bound.s0 == -INF && bound.s1 == INF)) {
      continue;
    }
    if (bound.quantity == BoundedQuantity::STEER_ANGLE) {
      vehicle.max_steer_angle = std::min(vehicle.max_steer_angle, static_cast<float>(bound.max));
    } else if (bound.quantity == BoundedQuantity::STEER_RATE) {
      vehicle.steer_rate_lim = std::min(vehicle.steer_rate_lim, static_cast<float>(bound.max));
    }
  }

  autoware::mppi_optimizer::FirstOrderDubinsMppiCostParams cost;
  cost.lambda = static_cast<float>(p.cost.lambda);
  cost.speed_coeff = static_cast<float>(p.cost.speed);
  cost.track_coeff = 0.0F;
  cost.heading_coeff = 0.0F;
  cost.track_center_coeff = static_cast<float>(p.cost.track_center);
  cost.corner_buffer_coeff = static_cast<float>(p.cost.corner_buffer);
  cost.corner_safe_margin = static_cast<float>(p.margin.corner_safe_m);
  cost.lateral_distance_coeff = static_cast<float>(p.cost.lateral_distance);
  cost.lateral_yaw_error_coeff = static_cast<float>(p.cost.lateral_yaw_error);
  cost.remaining_distance_coeff = static_cast<float>(p.cost.remaining_distance);
  cost.path_overshoot_coeff = static_cast<float>(p.cost.path_overshoot);
  cost.lateral_acceleration_coeff = static_cast<float>(p.cost.lateral_acceleration);
  cost.lateral_jerk_coeff = static_cast<float>(p.cost.lateral_jerk);
  cost.longitudinal_jerk_coeff = static_cast<float>(p.cost.longitudinal_jerk);
  cost.accel_cmd_coeff = static_cast<float>(p.cost.accel_cmd);
  cost.steer_cmd_coeff = static_cast<float>(p.cost.steer_cmd);
  cost.steer_rate_coeff = static_cast<float>(p.cost.steer_rate);
  cost.overlimit_coeff = static_cast<float>(p.cost.overlimit);
  cost.accel_cmd_std_dev = static_cast<float>(p.sampling.accel_cmd_std_dev_mps2);
  cost.steer_cmd_std_dev = static_cast<float>(p.sampling.steer_cmd_std_dev_rad);
  cost.accel_cmd_noise_exponent = static_cast<float>(p.sampling.accel_cmd_noise_exponent);
  cost.steer_cmd_noise_exponent = static_cast<float>(p.sampling.steer_cmd_noise_exponent);
  cost.boundary_threshold = static_cast<float>(p.boundary_threshold_m);
  cost.obstacle_collision_margin = static_cast<float>(p.margin.obstacle_collision_m);
  cost.obstacle_safe_margin = static_cast<float>(p.margin.obstacle_safe_m);
  cost.road_border_collision_margin = static_cast<float>(p.margin.road_border_collision_m);
  cost.road_border_safe_margin = static_cast<float>(p.margin.road_border_safe_m);
  cost.drivable_area_safe_margin = static_cast<float>(p.margin.drivable_area_safe_m);
  cost.drivable_area_barrier_weight = static_cast<float>(p.cost.drivable_area_barrier);
  cost.crash_contact_penalty = static_cast<float>(p.cost.crash_contact_penalty);

  autoware::mppi_optimizer::FirstOrderDubinsMppiRuntimeOptions options;
  options.skip_if_invalid = true;
  // u_nom is forced from the reference every cycle (refine). Shifting the previous MPPI controls
  // instead keeps the sampling around a sequence whose head has no deceleration, and with the
  // horizon receding the braking is deferred every cycle until the goal is overshot
  options.use_last_control_as_nominal = false;
  options.use_temporal_mpt_as_nominal = p.nominal_seed == "temporal_mpt";
  options.enable_input_delay_compensation = p.vehicle.enable_input_delay_compensation;

  optimizer.setVehicleParams(vehicle);
  optimizer.setCostParams(cost);
  optimizer.setRuntimeOptions(options);
  optimizer.initialize();
}

std::optional<std::string> MppiPlanner::refine(
  MppiInterface & optimizer, const PlannerContext & context,
  const CompiledConstraints & compiled_constraints, const PathProjector & projector,
  Trajectory & reference, TrajectoryPlannerDebug & debug)
{
  const auto limits = collect_kinematic_limits(compiled_constraints);

  try {
    ensure_initialized(optimizer, context, compiled_constraints);
  } catch (const std::exception & e) {
    return std::string("initialize: ") + e.what();
  }

  auto inputs_st =
    std::make_unique<autoware_utils_debug::ScopedTimeTrack>("build_mppi_inputs", *time_keeper_);
  // points[0] is the ego at t = 0; MPPI takes points[k] as the state at (k + 1) dt
  Trajectory mppi_input;
  mppi_input.header = reference.header;
  mppi_input.points.assign(
    reference.points.begin() + 1, reference.points.begin() + 1 + kMppiHorizon);

  // ---- constraints -> MPPI inputs ----
  std::vector<Segment> road_borders;
  std::vector<Segment> drivable_area;
  TrackedObjects tracked_objects;
  tracked_objects.header = reference.header;
  for (const auto & raw : compiled_constraints.raw_constraints) {
    if (const auto * boundary = std::get_if<Boundary>(&raw.payload)) {
      append_segments(
        boundary_simplifier_->simplify(boundary->polyline),
        raw.hardness == Hardness::HARD ? road_borders : drivable_area);
    } else if (const auto * keep_out = std::get_if<KeepOut>(&raw.payload)) {
      tracked_objects.objects.push_back(to_tracked_object(*keep_out));
    }
  }
  debug.markers["mppi_boundaries"] = make_boundary_markers(
    road_borders, drivable_area, reference.header.stamp, context.odometry.pose.pose.position.z);

  FirstOrderDubinsMppiKinematicLimits kinematic_limits;
  // MPPI enables its velocity interval [0, max] (the sample cost and the recovery of the nominal
  // from a negative predicted velocity) only when a maximum is given; without it the ego stops a
  // few meters short of the goal. The maximum itself is kept out of reach: a maximum the ego has
  // exceeded once latches MPPI's deterministic velocity profile (buildActiveVelocityLimitProfile
  // keeps it while the limit is unchanged and then commands zero acceleration), and the ego never
  // accelerates again. The speed limits are enforced by the reference profile and the output check
  constexpr float UNREACHABLE_VELOCITY_MPS = 100.0F;
  kinematic_limits.max_velocity = UNREACHABLE_VELOCITY_MPS;
  kinematic_limits.min_longitudinal_acceleration = static_cast<float>(limits.a_hard_min);
  kinematic_limits.max_longitudinal_acceleration = static_cast<float>(limits.a_hard_max);
  const double lon_jerk = global_bound_max(compiled_constraints, BoundedQuantity::LON_JERK);
  if (std::isfinite(lon_jerk)) {
    kinematic_limits.min_longitudinal_jerk = -static_cast<float>(lon_jerk);
    kinematic_limits.max_longitudinal_jerk = static_cast<float>(lon_jerk);
  }
  // Stop bars become a per-point maximum of zero beyond the bar. A Gate has no time dimension
  // here: one that is closed anywhere in the horizon closes for the whole horizon
  const double horizon_s = to_seconds(mppi_input.points.back().time_from_start);
  kinematic_limits.max_velocity_by_reference_point.reserve(mppi_input.points.size());
  for (const auto & point : mppi_input.points) {
    const double s = projector.closest(point.pose.position);
    std::optional<float> v_max;
    for (const auto & stop_bar : compiled_constraints.stop_bars) {
      if (stop_bar.time.t1 < 0.0 || stop_bar.time.t0 > horizon_s) {
        continue;
      }
      if (s + context.vehicle_info.max_longitudinal_offset_m >= stop_bar.s_stop) {
        v_max = 0.0F;
      }
    }
    kinematic_limits.max_velocity_by_reference_point.push_back(v_max);
  }

  // ---- optimize ----
  // u_nom comes from the reference: its steer, and the acceleration as the velocity difference
  // starting from the ego. MPPI's own seeding differences the reference alone, so a reference
  // that already stands still seeds zero acceleration and the samples (std 0.1 m/s^2) never find
  // the braking; the ego then creeps past the stop point
  std::vector<float> nominal_accel(mppi_input.points.size());
  std::vector<float> nominal_steer(mppi_input.points.size());
  double v_prev = context.odometry.twist.twist.linear.x;
  for (std::size_t k = 0; k < mppi_input.points.size(); ++k) {
    const double v = mppi_input.points[k].longitudinal_velocity_mps;
    nominal_accel[k] = static_cast<float>((v - v_prev) / kMppiDt);
    nominal_steer[k] = mppi_input.points[k].front_wheel_angle_rad;
    v_prev = v;
  }
  // A forced nominal takes precedence over every other seed inside MPPI
  if (params_.mppi_planner.nominal_seed == "reference") {
    optimizer.setForcedNominalControl(nominal_accel, nominal_steer);
  }
  inputs_st.reset();

  autoware::mppi_optimizer::FirstOrderDubinsMppiOptimizationResult mppi_result;
  try {
    mppi_result = optimizer.optimizeTrajectory(
      mppi_input, context.odometry, context.acceleration, context.steering, tracked_objects,
      road_borders, drivable_area, kinematic_limits);
  } catch (const std::exception & e) {
    return std::string("optimize: ") + e.what();
  }
  debug.trajectories["mppi_reference"] = mppi_input;
  debug.trajectories["mppi_optimized"] = mppi_result.debug.optimized_trajectory;
  {
    // Frozen output while the reference moves off: log the state of the deterministic profile
    float v_out_max = 0.0F;
    for (const auto & p : mppi_result.trajectory.points) {
      v_out_max = std::max(v_out_max, p.longitudinal_velocity_mps);
    }
    if (v_out_max < 0.01F && mppi_input.points.back().longitudinal_velocity_mps > 0.5F) {
      const auto & d = mppi_result.debug;
      RCLCPP_WARN_THROTTLE(
        logger(), steady_clock(), 2000,
        "MPPI output frozen: ego_v=%.3f profile_active=%d external_limit=%d map_limit=%d "
        "rejected=%d nominal_a0=%.3f max_velocity=%.2f pointwise[0]=%s",
        context.odometry.twist.twist.linear.x, d.velocity_limit_profile_active,
        d.external_velocity_limit_active, d.map_velocity_limit_active, d.was_rejected,
        nominal_accel.front(), kinematic_limits.max_velocity.value_or(-1.0F),
        d.effective_max_velocity_by_reference_point.empty()
          ? "none"
          : (d.effective_max_velocity_by_reference_point.front()
               ? std::to_string(*d.effective_max_velocity_by_reference_point.front()).c_str()
               : "nullopt"));
    }
  }
  if (mppi_result.debug.was_rejected) {
    return "rejected: " + to_string(mppi_result.debug.validation.reasons);
  }

  // ---- splice back ----
  // MPPI writes the undelayed commands into acceleration / front_wheel_angle, but the next cycle
  // reads the ego steer from the trajectory, so both are derived from the states where the
  // states allow it. Below walking pace the heading difference is too noisy for the curvature and
  // the steer command is used as is: it is what MPPI wants the wheels to do while (almost)
  // standing, e.g. unwinding a saturated steer before moving off, which the reference steer would
  // not show
  auto splice_st =
    std::make_unique<autoware_utils_debug::ScopedTimeTrack>("splice_back", *time_keeper_);
  Trajectory refined = reference;
  const double wheel_base_m = context.vehicle_info.wheel_base_m;
  const double max_steer = context.vehicle_info.max_steer_angle_rad;
  const auto & optimized = mppi_result.trajectory.points;
  refined.points.resize(optimized.size() + 1);
  // The reference is the fastest profile under the constraints, so MPPI is never allowed to be
  // faster than it at the same index: MPPI tracks the reference speed loosely (the t-MPT seed
  // brakes softer than the reference, and MPPI floors its output at 0.25 m/s as soon as any
  // later point exceeds that), and the excess would either trip the checks below or, at the
  // stop, be cut away in one step. The acceleration is rebuilt from the capped speeds afterwards
  for (std::size_t i = 0; i < optimized.size(); ++i) {
    const auto & in = optimized[i];
    auto & out = refined.points[i + 1];
    const auto & next = i + 1 < optimized.size() ? optimized[i + 1] : in;
    const double v = std::min<double>(
      in.longitudinal_velocity_mps, mppi_input.points[i].longitudinal_velocity_mps);
    const double ds = autoware_utils_geometry::calc_distance2d(in.pose, next.pose);
    constexpr double MIN_CURVATURE_STEP_M = 0.1;
    const double steer = ds > MIN_CURVATURE_STEP_M
                           ? std::atan(
                               autoware_utils_math::normalize_radian(
                                 autoware_utils_geometry::get_rpy(next.pose).z -
                                 autoware_utils_geometry::get_rpy(in.pose).z) /
                               ds * wheel_base_m)
                           : static_cast<double>(in.front_wheel_angle_rad);
    const double kappa = std::tan(steer) / wheel_base_m;
    out.pose = in.pose;
    out.longitudinal_velocity_mps = static_cast<float>(v);
    out.front_wheel_angle_rad = static_cast<float>(std::clamp(steer, -max_steer, max_steer));
    out.heading_rate_rps = static_cast<float>(v * kappa);
  }
  // Once the reference has come to a stop, MPPI creeps on at a few cm/s: at v = 0 the reverse
  // prevention clips the negative half of the sampling noise, so its mean is positive. The creep
  // would later trip the engage-velocity floor and carry the ego past the stop point, so the
  // output is held at the stop from there on
  constexpr float STOPPED_MPS = 1e-3F;
  std::size_t stop_index = optimized.size();
  while (stop_index > 0 &&
         mppi_input.points[stop_index - 1].longitudinal_velocity_mps < STOPPED_MPS) {
    --stop_index;
  }
  for (std::size_t i = stop_index; i < optimized.size(); ++i) {
    auto & out = refined.points[i + 1];
    out.pose = refined.points[stop_index].pose;
    out.longitudinal_velocity_mps = 0.0F;
    out.heading_rate_rps = 0.0F;
  }
  for (std::size_t i = 1; i < refined.points.size(); ++i) {
    const float v_next = i + 1 < refined.points.size()
                           ? refined.points[i + 1].longitudinal_velocity_mps
                           : refined.points[i].longitudinal_velocity_mps;
    refined.points[i].acceleration_mps2 =
      (v_next - refined.points[i].longitudinal_velocity_mps) / kMppiDt;
  }

  splice_st.reset();

  std::string reason;
  {
    autoware_utils_debug::ScopedTimeTrack check_st("satisfies_constraints", *time_keeper_);
    if (!satisfies_constraints(context, compiled_constraints, projector, refined, reason)) {
      return "output violates " + reason;
    }
  }
  reference = std::move(refined);
  return std::nullopt;
}

bool MppiPlanner::satisfies_constraints(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const PathProjector & projector, const Trajectory & trajectory, std::string & reason) const
{
  const auto & path = context.reference_path;
  const auto limits = collect_kinematic_limits(compiled_constraints);
  const double lat_accel_max = global_bound_max(compiled_constraints, BoundedQuantity::LAT_ACCEL);
  const double wheel_base_m = context.vehicle_info.wheel_base_m;

  for (std::size_t k = 0; k < trajectory.points.size(); ++k) {
    const auto & point = trajectory.points[k];
    const double s = projector.closest(point.pose.position);
    const double l =
      lateral_offset_at(path, s, Point2d{point.pose.position.x, point.pose.position.y});
    const double v = point.longitudinal_velocity_mps;
    const double kappa = std::tan(point.front_wheel_angle_rad) / wheel_base_m;
    if (v < -1e-3) {
      reason = "reverse";
      return false;
    }
    if (std::abs(v * v * kappa) > lat_accel_max + 1e-6) {
      reason = "lat_accel (k=" + std::to_string(k) + " v=" + std::to_string(v) +
               " kappa=" + std::to_string(kappa) + ")";
      return false;
    }
    // MPPI is given no reachable velocity limit (see refine), so its output overshoots the
    // reference profile by the sampling noise, and points[0] carries the measured ego speed
    constexpr double VELOCITY_MARGIN_MPS = 0.5;
    if (v > velocity_limit_at(compiled_constraints, limits, s) + VELOCITY_MARGIN_MPS) {
      reason = "velocity (k=" + std::to_string(k) + " v=" + std::to_string(v) + ")";
      return false;
    }

    const auto box = footprint_sl_box(context.vehicle_info, s, l);
    const double t0 = to_seconds(point.time_from_start);
    const double t1 =
      k + 1 < trajectory.points.size() ? to_seconds(trajectory.points[k + 1].time_from_start) : t0;
    for (const auto & bound : compiled_constraints.lateral_bounds) {
      if (
        compiled_constraints.raw_constraints[bound.raw_index].hardness == Hardness::HARD &&
        violates_lateral_bound(bound, box)) {
        reason = "lateral_bound";
        return false;
      }
    }
    for (const auto & occupancy : compiled_constraints.occupancies) {
      if (violates_occupancy(occupancy, box, t0, t1)) {
        reason = "occupancy";
        return false;
      }
    }
    for (const auto & stop_bar : compiled_constraints.stop_bars) {
      if (violates_stop_bar(stop_bar, box, t0, t1)) {
        reason = "stop_bar";
        return false;
      }
    }
  }
  return true;
}

}  // namespace autoware::safety_planner::experimental

PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::experimental::MppiPlanner,
  autoware::safety_planner::TrajectoryPlannerInterface)
