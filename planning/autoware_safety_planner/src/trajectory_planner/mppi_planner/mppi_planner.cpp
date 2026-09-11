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
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
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

namespace autoware::safety_planner::experiment
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

void append_segments(const LineString2d & polyline, std::vector<Segment> & segments)
{
  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    segments.push_back(
      Segment{
        static_cast<float>(polyline[i].x()), static_cast<float>(polyline[i].y()),
        static_cast<float>(polyline[i + 1].x()), static_cast<float>(polyline[i + 1].y())});
  }
}

//! MPPI takes an oriented box per object, so the body-frame shape is reduced to its bounding
//! box and the motion to the speed between the first two waypoints (it extrapolates at constant
//! velocity; the later waypoints are not representable)
TrackedObject to_tracked_object(const RigidBody & body)
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
  frenet_planner_.on_initialize(time_keeper, params);
  normal_optimizer_ = std::make_unique<MppiInterface>();
  cautious_optimizer_ = std::make_unique<MppiInterface>();
}

TrajectoryPlannerResult MppiPlanner::plan_trajectories(const TrajectoryPlannerInput & input)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  auto result = frenet_planner_.plan_trajectories(input);
  if (result.normal_trajectory) {
    autoware_utils_debug::ScopedTimeTrack side_st("refine_normal", *time_keeper_);
    refine_one_side(
      *normal_optimizer_, input.context, input.normal_constraints,
      result.normal_trajectory->trajectory, result.normal_debug);
  }
  if (result.cautious_trajectory) {
    autoware_utils_debug::ScopedTimeTrack side_st("refine_cautious", *time_keeper_);
    refine_one_side(
      *cautious_optimizer_, input.context, input.cautious_constraints,
      result.cautious_trajectory->trajectory, result.cautious_debug);
  }
  return result;
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
  // u_nom is forced from the Frenet trajectory every cycle (refine_one_side). Shifting the
  // previous MPPI controls instead keeps the sampling around a sequence whose head has no
  // deceleration, and with the horizon receding the braking is deferred every cycle until the
  // goal is overshot. The acados t-MPT seed is tuned for the diffusion planner stack
  options.use_last_control_as_nominal = false;
  options.use_temporal_mpt_as_nominal = false;
  options.enable_input_delay_compensation = p.vehicle.enable_input_delay_compensation;

  optimizer.setVehicleParams(vehicle);
  optimizer.setCostParams(cost);
  optimizer.setRuntimeOptions(options);
  optimizer.initialize();
}

bool MppiPlanner::refine_one_side(
  MppiInterface & optimizer, const PlannerContext & context,
  const std::vector<Constraint> & constraints, Trajectory & reference,
  TrajectoryPlannerDebug & debug)
{
  const double dt = params_.frenet_sampling_based_planner.time_step_s;
  if (std::abs(dt - kMppiDt) > 1e-6) {
    RCLCPP_WARN_ONCE(
      logger(), "MPPI reads the reference as one point per %.2f s; time_step_s = %.3f. Skipped.",
      static_cast<double>(kMppiDt), dt);
    return false;
  }
  // points[0] is the ego at t = 0; MPPI takes points[k] as the state at (k + 1) dt
  if (reference.points.size() < static_cast<std::size_t>(kMppiHorizon) + 1) {
    return false;
  }

  const auto compiled_constraints = compile_constraint_list(context, constraints);
  const auto limits = collect_kinematic_limits(compiled_constraints);

  try {
    ensure_initialized(optimizer, context, compiled_constraints);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(logger(), steady_clock(), 5000, "MPPI initialize failed: %s", e.what());
    return false;
  }

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
        boundary->polyline, raw.hardness == Hardness::HARD ? road_borders : drivable_area);
    } else if (const auto * keep_out = std::get_if<KeepOut>(&raw.payload)) {
      // TimedPolygonSequence has no box-per-object form; it stays on the output check
      if (const auto * body = std::get_if<RigidBody>(&keep_out->occupancy)) {
        tracked_objects.objects.push_back(to_tracked_object(*body));
      }
    }
  }

  FirstOrderDubinsMppiKinematicLimits kinematic_limits;
  kinematic_limits.max_velocity = static_cast<float>(limits.v_hard);
  kinematic_limits.min_longitudinal_acceleration = static_cast<float>(limits.a_hard_min);
  kinematic_limits.max_longitudinal_acceleration = static_cast<float>(limits.a_hard_max);
  for (const auto & bound : compiled_constraints.scalar_bounds) {
    if (
      bound.quantity == BoundedQuantity::LON_JERK && bound.s0 == -INF && bound.s1 == INF &&
      std::isfinite(bound.max)) {
      kinematic_limits.min_longitudinal_jerk = -static_cast<float>(bound.max);
      kinematic_limits.max_longitudinal_jerk = static_cast<float>(bound.max);
    }
  }
  // Regional velocity bounds and stop bars become a per-point maximum. A Gate has no time
  // dimension here: one that is closed anywhere in the horizon closes for the whole horizon
  const double horizon_s = to_seconds(mppi_input.points.back().time_from_start);
  const auto & path = context.reference_path;
  kinematic_limits.max_velocity_by_reference_point.reserve(mppi_input.points.size());
  for (const auto & point : mppi_input.points) {
    const double s = experimental::trajectory::closest(path, point.pose.position);
    std::optional<float> v_max;
    for (const auto & bound : compiled_constraints.scalar_bounds) {
      if (bound.quantity == BoundedQuantity::VELOCITY && bound.s0 <= s && s <= bound.s1) {
        v_max = std::min(
          v_max.value_or(std::numeric_limits<float>::infinity()), static_cast<float>(bound.max));
      }
    }
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
  // u_nom comes from the Frenet trajectory: its steer, and the acceleration as the velocity
  // difference starting from the ego. MPPI's own seeding differences the reference alone, so a
  // reference that already stands still seeds zero acceleration and the samples (std 0.1 m/s^2)
  // never find the braking; the ego then creeps past the stop point
  std::vector<float> nominal_accel(mppi_input.points.size());
  std::vector<float> nominal_steer(mppi_input.points.size());
  double v_prev = context.odometry.twist.twist.linear.x;
  for (std::size_t k = 0; k < mppi_input.points.size(); ++k) {
    const double v = mppi_input.points[k].longitudinal_velocity_mps;
    nominal_accel[k] = static_cast<float>((v - v_prev) / kMppiDt);
    nominal_steer[k] = mppi_input.points[k].front_wheel_angle_rad;
    v_prev = v;
  }
  optimizer.setForcedNominalControl(nominal_accel, nominal_steer);

  autoware::mppi_optimizer::FirstOrderDubinsMppiOptimizationResult mppi_result;
  try {
    mppi_result = optimizer.optimizeTrajectory(
      mppi_input, context.odometry, context.acceleration, context.steering, tracked_objects,
      road_borders, drivable_area, kinematic_limits);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(logger(), steady_clock(), 5000, "MPPI failed: %s", e.what());
    return false;
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
    RCLCPP_WARN_THROTTLE(
      logger(), steady_clock(), 5000,
      "MPPI rejected its output (%s); keeping the Frenet trajectory",
      to_string(mppi_result.debug.validation.reasons).c_str());
    return false;
  }

  // ---- splice back ----
  // MPPI writes the undelayed commands into acceleration / front_wheel_angle, but the next cycle
  // reads the ego steer from the trajectory, so both are derived from the states where the
  // states allow it. Below walking pace the heading difference is too noisy for the curvature and
  // the steer command is used as is: it is what MPPI wants the wheels to do while (almost)
  // standing, e.g. unwinding a saturated steer before moving off, which the reference steer would
  // not show. The Frenet points beyond the MPPI horizon are dropped: keeping them puts a
  // position jump at the seam
  Trajectory refined = reference;
  const double wheel_base_m = context.vehicle_info.wheel_base_m;
  const double max_steer = context.vehicle_info.max_steer_angle_rad;
  const auto & optimized = mppi_result.trajectory.points;
  refined.points.resize(optimized.size() + 1);
  // Over the final crawl of the reference (below the engage speed up to its end) MPPI may not be
  // faster than the reference: MPPI floors its own output at 0.25 m/s as soon as any later point
  // exceeds that, and the creep at the stop (see below) makes one, so the ego would otherwise
  // roll on at 0.25 m/s. A launch also starts below the engage speed, which is why the tail is
  // taken and not every slow point
  const float engage_mps = static_cast<float>(params_.engage_velocity.velocity_hard_mps);
  std::size_t crawl_index = optimized.size();
  while (crawl_index > 0 &&
         mppi_input.points[crawl_index - 1].longitudinal_velocity_mps < engage_mps) {
    --crawl_index;
  }
  for (std::size_t i = 0; i < optimized.size(); ++i) {
    const auto & in = optimized[i];
    auto & out = refined.points[i + 1];
    const auto & next = i + 1 < optimized.size() ? optimized[i + 1] : in;
    const double v = i >= crawl_index ? std::min<double>(
                                          in.longitudinal_velocity_mps,
                                          mppi_input.points[i].longitudinal_velocity_mps)
                                      : in.longitudinal_velocity_mps;
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
    out.acceleration_mps2 = static_cast<float>((next.longitudinal_velocity_mps - v) / kMppiDt);
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
    out.acceleration_mps2 = 0.0F;
    out.heading_rate_rps = 0.0F;
  }

  std::string reason;
  if (!satisfies_constraints(context, compiled_constraints, refined, reason)) {
    RCLCPP_WARN_THROTTLE(
      logger(), steady_clock(), 5000, "MPPI output violates %s; keeping the Frenet trajectory",
      reason.c_str());
    return false;
  }
  reference = std::move(refined);
  return true;
}

bool MppiPlanner::satisfies_constraints(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const Trajectory & trajectory, std::string & reason) const
{
  const auto & path = context.reference_path;
  const auto limits = collect_kinematic_limits(compiled_constraints);
  double lat_accel_max = INF;
  for (const auto & bound : compiled_constraints.scalar_bounds) {
    if (bound.quantity == BoundedQuantity::LAT_ACCEL && bound.s0 == -INF && bound.s1 == INF) {
      lat_accel_max = std::min(lat_accel_max, bound.max);
    }
  }
  const double wheel_base_m = context.vehicle_info.wheel_base_m;

  for (std::size_t k = 0; k < trajectory.points.size(); ++k) {
    const auto & point = trajectory.points[k];
    const double s = experimental::trajectory::closest(path, point.pose.position);
    const double l =
      lateral_offset_at(path, s, Point2d{point.pose.position.x, point.pose.position.y});
    const double v = point.longitudinal_velocity_mps;
    const double kappa = std::tan(point.front_wheel_angle_rad) / wheel_base_m;
    if (v < -1e-3) {
      reason = "reverse";
      return false;
    }
    if (std::abs(v * v * kappa) > lat_accel_max + 1e-6) {
      reason = "lat_accel";
      return false;
    }
    double v_max = limits.v_hard;
    for (const auto & bound : compiled_constraints.scalar_bounds) {
      if (bound.quantity == BoundedQuantity::VELOCITY && bound.s0 <= s && s <= bound.s1) {
        v_max = std::min(v_max, bound.max);
      }
    }
    if (v > v_max + 1e-3) {
      reason = "velocity";
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

}  // namespace autoware::safety_planner::experiment

PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::experiment::MppiPlanner,
  autoware::safety_planner::TrajectoryPlannerInterface)
