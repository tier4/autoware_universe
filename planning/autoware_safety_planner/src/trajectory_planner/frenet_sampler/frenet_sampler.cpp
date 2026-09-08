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

#include "frenet_sampler.hpp"

#include "../../utils/trajectory_conversion.hpp"

#include <autoware_frenet_planner/polynomials.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

//! The VELOCITY limit in effect at the arc length s, global and region-limited bounds together
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

//! Upper bounds of the global ScalarBound constraints, per quantity: the ones
//! collect_kinematic_limits does not read (LAT_ACCEL, LON_JERK, STEER_ANGLE, STEER_RATE,
//! CURVATURE)
struct GlobalBounds
{
  double lat_accel{INF};
  double lon_jerk{INF};
  double curvature{INF};
  double steer_angle{INF};
  double steer_rate{INF};
};

GlobalBounds collect_global_bounds(const CompiledConstraints & compiled_constraints)
{
  GlobalBounds bounds;
  for (const auto & bound : compiled_constraints.scalar_bounds) {
    if (!(bound.s0 == -INF && bound.s1 == INF)) {
      continue;
    }
    switch (bound.quantity) {
      case BoundedQuantity::LAT_ACCEL:
        bounds.lat_accel = std::min(bounds.lat_accel, bound.max);
        break;
      case BoundedQuantity::LON_JERK:
        bounds.lon_jerk = std::min(bounds.lon_jerk, bound.max);
        break;
      case BoundedQuantity::CURVATURE:
        bounds.curvature = std::min(bounds.curvature, bound.max);
        break;
      case BoundedQuantity::STEER_ANGLE:
        bounds.steer_angle = std::min(bounds.steer_angle, bound.max);
        break;
      case BoundedQuantity::STEER_RATE:
        bounds.steer_rate = std::min(bounds.steer_rate, bound.max);
        break;
      default:
        break;  // VELOCITY and LON_ACCEL belong to KinematicLimits
    }
  }
  return bounds;
}

//! Linear interpolation of samples spaced res apart from s0, clamped at both ends
double interpolate_uniform(
  const std::vector<double> & values, const double s0, const double res, const double s)
{
  if (values.size() < 2) {
    return values.front();
  }
  const double u = std::clamp((s - s0) / res, 0.0, static_cast<double>(values.size() - 1));
  const auto i = std::min(static_cast<std::size_t>(u), values.size() - 2);
  const double r = u - static_cast<double>(i);
  return values[i] * (1.0 - r) + values[i + 1] * r;
}

//! Linear interpolation of headings, wrapping at 2 pi
double interpolate_uniform_angle(
  const std::vector<double> & yaws, const double s0, const double res, const double s)
{
  if (yaws.size() < 2) {
    return yaws.front();
  }
  const double u = std::clamp((s - s0) / res, 0.0, static_cast<double>(yaws.size() - 1));
  const auto i = std::min(static_cast<std::size_t>(u), yaws.size() - 2);
  const double r = u - static_cast<double>(i);
  const double d = autoware_utils_math::normalize_radian(yaws[i + 1] - yaws[i]);
  return autoware_utils_math::normalize_radian(yaws[i] + d * r);
}

}  // namespace

TrajectoryPlannerResult FrenetSamplingBasedPlanner::plan(const TrajectoryPlannerInput & input)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  TrajectoryPlannerResult result;
  {
    autoware_utils_debug::ScopedTimeTrack side_st("plan_normal", *time_keeper_);
    auto compiled = compile_constraint_list(input.context, input.normal_constraints);
    // The candidates ride on the marker topic of the rough plan, so that the node does not need
    // another publisher
    result.normal_trajectory =
      plan_one_side(input.context, compiled, result.debug.rough_plan_result.debug.debug_markers);
    result.debug.compiled_constraints = std::move(compiled);
  }
  {
    autoware_utils_debug::ScopedTimeTrack side_st("plan_cautious", *time_keeper_);
    const auto compiled = compile_constraint_list(input.context, input.cautious_constraints);
    MarkerArray unused_markers;
    result.cautious_trajectory = plan_one_side(input.context, compiled, unused_markers);
  }
  return result;
}

std::optional<Trajectory> FrenetSamplingBasedPlanner::plan_one_side(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  MarkerArray & debug_markers) const
{
  const auto initial_state = compute_initial_state(context);
  const auto paths = generate_paths(context, initial_state);
  const auto profiles = generate_velocity_profiles(context, initial_state, compiled_constraints);

  std::vector<Candidate> candidates;
  candidates.reserve(paths.size() * profiles.size());
  for (const auto & path : paths) {
    for (const auto & profile : profiles) {
      candidates.push_back(combine(context, path, profile));
      evaluate(context, compiled_constraints, initial_state.l_goal, candidates.back());
    }
  }
  append_debug_markers(context, candidates, debug_markers);

  const Candidate * best = nullptr;
  for (const auto & candidate : candidates) {
    if (candidate.valid && (!best || candidate.cost < best->cost)) {
      best = &candidate;
    }
  }

  if (!best) {
    std::map<std::string, int> reasons;
    for (const auto & candidate : candidates) {
      const auto pos = candidate.tag.rfind('[');
      reasons[pos == std::string::npos ? "?" : candidate.tag.substr(pos)]++;
    }
    std::stringstream ss;
    for (const auto & [reason, count] : reasons) {
      ss << " " << reason << "x" << count;
    }
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("safety_planner"), steady_clock, 5000,
      "[frenet_sampler] no valid candidate (%zu sampled:%s). Falling back to the stop trajectory.",
      candidates.size(), ss.str().c_str());
    // The stop trajectory keeps the current lateral position: the path back from the ego heading
    // to l0, driven at the hardest deceleration
    const auto straight =
      sample_path(context, initial_state, context.reference_path.length(), initial_state.l);
    const auto stop =
      make_stop_profile(initial_state, collect_kinematic_limits(compiled_constraints));
    return to_trajectory_msg(context, combine(context, straight, stop));
  }
  return to_trajectory_msg(context, *best);
}

FrenetSamplingBasedPlanner::InitialState FrenetSamplingBasedPlanner::compute_initial_state(
  const PlannerContext & context) const
{
  const auto ego = compute_ego_frenet_state(context);
  const auto & path = context.reference_path;
  const double ego_yaw = autoware_utils_geometry::get_rpy(context.odometry.pose.pose).z;
  const double frenet_yaw = autoware_utils_math::normalize_radian(ego_yaw - path.azimuth(ego.s));
  const double v = context.odometry.twist.twist.linear.x;

  InitialState state;
  state.s = ego.s;
  state.l = ego.l;
  // tan diverges as the heading approaches +-90 deg against the centerline, so the slope is cut
  // off at the equivalent of +-60 deg. The factor (1 - k_ref*l) inverts the heading expression of
  // sample_path; without it yaw[0] does not match the ego heading
  state.dl_ds = (1.0 - path.curvature(ego.s) * ego.l) *
                std::tan(std::clamp(frenet_yaw, -M_PI / 3.0, M_PI / 3.0));
  // Pinning l''(0) to 0 would flatten the start of the lateral motion at every replan and barely
  // approach a lateral target 40 m ahead, the restart problem of a receding horizon. The initial
  // curvature comes from the measured steer angle instead, under the small angle approximation
  // (l' assumed small) rather than the exact Frenet expression
  const double kappa_ego =
    std::tan(context.steering.steering_tire_angle) / context.vehicle_info.wheel_base_m;
  state.d2l_ds2 = kappa_ego - path.curvature(ego.s);
  state.v = std::max(0.0, v * std::cos(frenet_yaw));
  state.a = context.acceleration.accel.accel.linear.x;
  state.l_goal = lateral_offset_at(
    path, path.length(), Point2d{context.goal_pose.position.x, context.goal_pose.position.y});
  return state;
}

FrenetSamplingBasedPlanner::PathCandidate FrenetSamplingBasedPlanner::sample_path(
  const PlannerContext & context, const InitialState & initial_state, const double length,
  const double l_target) const
{
  using autoware::frenet_planner::Polynomial;

  const auto & ref = context.reference_path;
  const double res = params_.frenet_sampler.path_resolution_m;
  const double s0 = initial_state.s;
  const double s_max = ref.length();

  // l(s) joins the initial (l0, l'0, l''0) to the terminal (l_T, 0, 0) over the arc length L and
  // holds l_T beyond it. A terminal state past the end of the path is cut at the end
  const double L = std::max(res, std::min(length, s_max - s0));
  const Polynomial lat(
    initial_state.l, initial_state.dl_ds, initial_state.d2l_ds2, l_target, 0.0, 0.0, L);

  PathCandidate path;
  for (double s = s0; s <= s_max + 1e-9; s += res) {
    const double u = s - s0;
    const double l = u <= L ? lat.position(u) : l_target;
    const double dl_ds = u <= L ? lat.velocity(u) : 0.0;
    const double s_ref = std::clamp(s, 0.0, s_max);
    path.s.push_back(s);
    path.l.push_back(l);
    // The heading comes from the analytic Frenet expression psi = psi_ref + atan(l' /
    // (1 - k_ref*l)). Taking it from the chord between world positions would put the first heading
    // off the ego heading by k*res/2, which in closed loop drifts the ego heading a little every
    // cycle until it hits the steer rate limit after a few dozen of them
    path.yaw.push_back(
      autoware_utils_math::normalize_radian(
        ref.azimuth(s_ref) + std::atan2(dl_ds, 1.0 - ref.curvature(s_ref) * l)));
  }
  // The curvature is the difference of the headings over the arc length, centered except at the
  // ends. Since l'(0) comes from the ego heading, yaw[0] already matches it and a candidate leaving
  // in another direction needs no separate rejection here
  const auto n = path.s.size();
  for (std::size_t i = 0; i < n; ++i) {
    const auto i0 = i == 0 ? i : i - 1;
    const auto i1 = i + 1 < n ? i + 1 : i;
    const double dyaw = autoware_utils_math::normalize_radian(path.yaw[i1] - path.yaw[i0]);
    path.kappa.push_back(i1 > i0 ? dyaw / (static_cast<double>(i1 - i0) * res) : 0.0);
  }
  std::stringstream ss;
  ss << "L=" << L << " l=" << l_target;
  path.tag = ss.str();
  return path;
}

std::vector<FrenetSamplingBasedPlanner::PathCandidate> FrenetSamplingBasedPlanner::generate_paths(
  const PlannerContext & context, const InitialState & initial_state) const
{
  const auto & p = params_.frenet_sampler;
  // The lateral position of the goal joins the terminal candidates, so that a goal off the grid
  // (on the shoulder, ...) can still be reached
  auto lateral_targets = p.target_lateral_positions_m;
  const bool on_grid = std::any_of(
    lateral_targets.begin(), lateral_targets.end(),
    [&](const double l) { return std::abs(l - initial_state.l_goal) < 0.05; });
  if (!on_grid) {
    lateral_targets.push_back(initial_state.l_goal);
  }
  std::vector<PathCandidate> paths;
  for (const double length : p.target_lengths_m) {
    for (const double l_target : lateral_targets) {
      paths.push_back(sample_path(context, initial_state, length, l_target));
    }
  }
  return paths;
}

std::vector<FrenetSamplingBasedPlanner::VelocityProfile>
FrenetSamplingBasedPlanner::generate_velocity_profiles(
  const PlannerContext & context, const InitialState & initial_state,
  const CompiledConstraints & compiled_constraints) const
{
  using autoware::frenet_planner::Polynomial;

  const auto & p = params_.frenet_sampler;
  const double dt = p.time_step_s;
  const double horizon = p.horizon_s;
  const double s_max = context.reference_path.length();
  const auto limits = collect_kinematic_limits(compiled_constraints);
  double v_limit = velocity_limit_at(compiled_constraints, limits, initial_state.s);
  {
    // The terminal speeds are sampled against the speed at which the path curvature within the
    // horizon can still be taken under the lateral acceleration and steer rate limits. Sampling
    // ratios of the global limit (tens of km/h) instead leaves no candidate at an intermediate
    // speed in front of a curve, so only the stop profile survives and the ego crawls to a halt.
    // The limit at a point ahead is converted into the speed from which that point is reachable by
    // braking; taking the curvature ahead as the limit here would slow the whole horizon down
    // because of the curvature spike at the end of the path, where it joins the goal. The
    // deceleration only sets the sampling reference, so a fixed value is enough; whether a
    // candidate really works is decided in evaluate()
    constexpr double SAMPLING_DECEL_MPS2 = 1.0;
    const auto bounds = collect_global_bounds(compiled_constraints);
    const double wheel_base_m = context.vehicle_info.wheel_base_m;
    const double res = p.path_resolution_m;
    const double s_end = std::min(s_max, initial_state.s + v_limit * horizon);
    double prev_steer = std::atan(context.reference_path.curvature(initial_state.s) * wheel_base_m);
    for (double s = initial_state.s + res; s <= s_end; s += res) {
      const double kappa = context.reference_path.curvature(s);
      const double steer = std::atan(kappa * wheel_base_m);
      double v_cap = INF;
      if (std::abs(kappa) > 1e-6) {
        v_cap = std::min(v_cap, std::sqrt(bounds.lat_accel / std::abs(kappa)));
      }
      const double steer_grad = std::abs(steer - prev_steer) / res;  // [rad/m]
      if (steer_grad > 1e-6) {
        v_cap = std::min(v_cap, bounds.steer_rate / steer_grad);
      }
      prev_steer = steer;
      if (std::isfinite(v_cap)) {
        v_limit = std::min(
          v_limit, std::sqrt(v_cap * v_cap + 2.0 * SAMPLING_DECEL_MPS2 * (s - initial_state.s)));
      }
    }
  }

  std::vector<VelocityProfile> profiles;
  const auto sample = [&](const double duration, const double v_target, const double s_target) {
    const Polynomial lon(
      initial_state.s, initial_state.v, initial_state.a, s_target, v_target, 0.0, duration);
    VelocityProfile profile;
    for (double t = 0.0; t <= horizon + 1e-9; t += dt) {
      profile.t.push_back(t);
      if (t <= duration) {
        profile.s.push_back(lon.position(t));
        profile.v.push_back(lon.velocity(t));
        profile.a.push_back(lon.acceleration(t));
      } else {
        // Hold the terminal state at a constant speed until the end of the horizon
        profile.s.push_back(s_target + v_target * (t - duration));
        profile.v.push_back(v_target);
        profile.a.push_back(0.0);
      }
    }
    std::stringstream ss;
    ss << "T=" << duration << " v=" << v_target;
    profile.tag = ss.str();
    profiles.push_back(std::move(profile));
  };

  for (const double duration : p.target_durations_s) {
    for (const double v_ratio : p.target_velocity_ratios) {
      const double v_target = v_ratio * v_limit;
      // The terminal s is the distance covered at the average speed, as in velocity keeping, and
      // never reaches beyond the goal at the end of the path
      const double s_target =
        std::min(s_max, initial_state.s + 0.5 * (initial_state.v + v_target) * duration);
      sample(duration, v_target, s_target);
    }
  }

  // Profiles that stop at the goal: T is derived from the remaining distance, as the time to
  // cover it at the average of the initial speed. With a fixed set of T the only candidates left a
  // few meters before the goal either cannot cover the distance within T, i.e. drive backwards, or
  // overshoot it, and all of them are rejected
  {
    // Stop at the goal, or at the nearest stop bar (Gate) ahead if that comes first. Without this
    // the only candidate that respects a stop bar is standstill (every profile that moves reaches
    // the bar within the horizon), so the ego would never approach it. The target is the
    // base_link position whose footprint front just touches the bar (violates_stop_bar)
    double s_stop_target = s_max;
    for (const auto & stop_bar : compiled_constraints.stop_bars) {
      if (stop_bar.time.t1 < 0.0 || stop_bar.time.t0 > horizon) {
        continue;
      }
      s_stop_target = std::min(
        s_stop_target,
        stop_bar.s_stop - stop_bar.margin - context.vehicle_info.max_longitudinal_offset_m);
    }
    s_stop_target = std::max(s_stop_target, initial_state.s);
    const double remaining = s_stop_target - initial_state.s;
    const double duration = std::clamp(
      2.0 * remaining / std::max(initial_state.v, 0.1), p.target_durations_s.front(), horizon);
    sample(duration, 0.0, s_stop_target);
  }
  return profiles;
}

FrenetSamplingBasedPlanner::VelocityProfile FrenetSamplingBasedPlanner::make_stop_profile(
  const InitialState & initial_state, const KinematicLimits & limits) const
{
  const auto & p = params_.frenet_sampler;
  const double dt = p.time_step_s;
  const double decel = std::abs(limits.a_hard_min);

  VelocityProfile profile;
  profile.tag = "stop";
  double s = initial_state.s;
  double v = initial_state.v;
  for (double t = 0.0; t <= p.horizon_s + 1e-9; t += dt) {
    profile.t.push_back(t);
    profile.s.push_back(s);
    profile.v.push_back(v);
    profile.a.push_back(v > 0.0 ? -decel : 0.0);
    const double v_next = std::max(0.0, v - decel * dt);
    s += 0.5 * (v + v_next) * dt;
    v = v_next;
  }
  return profile;
}

FrenetSamplingBasedPlanner::Candidate FrenetSamplingBasedPlanner::combine(
  const PlannerContext & context, const PathCandidate & path, const VelocityProfile & profile) const
{
  const auto & ref = context.reference_path;
  const double res = params_.frenet_sampler.path_resolution_m;
  const double s0 = path.s.front();
  const double s_max = ref.length();

  Candidate candidate;
  candidate.tag = path.tag + " " + profile.tag;
  candidate.s = profile.s;
  candidate.l.reserve(profile.s.size());
  candidate.points.reserve(profile.s.size());
  for (std::size_t k = 0; k < profile.t.size(); ++k) {
    const double s = profile.s[k];
    const double l = interpolate_uniform(path.l, s0, res, s);
    candidate.l.push_back(l);

    OptimizedTrajectoryPoint point;
    point.t = profile.t[k];
    point.pose = to_world_pose(ref, std::clamp(s, 0.0, s_max), l);
    point.pose.yaw = interpolate_uniform_angle(path.yaw, s0, res, s);
    point.kappa = interpolate_uniform(path.kappa, s0, res, s);
    point.v = profile.v[k];
    point.a = profile.a[k];
    candidate.points.push_back(point);
  }
  return candidate;
}

void FrenetSamplingBasedPlanner::evaluate(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const double l_goal, Candidate & candidate) const
{
  const auto & p = params_.frenet_sampler;
  const double s_max = context.reference_path.length();
  const double blend_length =
    *std::max_element(p.target_lengths_m.begin(), p.target_lengths_m.end());
  const double wheel_base_m = context.vehicle_info.wheel_base_m;
  const auto limits = collect_kinematic_limits(compiled_constraints);
  const auto bounds = collect_global_bounds(compiled_constraints);
  const double dt = p.time_step_s;
  const auto reject = [&](const char * reason) {
    candidate.valid = false;
    candidate.tag += std::string(" [") + reason + "]";
  };

  double cost = 0.0;
  for (std::size_t k = 0; k < candidate.points.size(); ++k) {
    const double s = candidate.s[k];
    const double l = candidate.l[k];
    const auto & point = candidate.points[k];

    // A candidate that passes the goal at the end of the path, or drives backwards, is invalid
    if (s > s_max + 1e-3) {
      return reject("beyond_goal");
    }
    if (point.v < -1e-3) {
      return reject("reverse");
    }
    // --- vehicle kinematics (the ScalarBound constraints of VehicleKinematics) ---
    const double v_max = velocity_limit_at(compiled_constraints, limits, s);
    if (point.v > v_max + 1e-6) {
      return reject("velocity");
    }
    if (point.a < limits.a_hard_min - 1e-6 || point.a > limits.a_hard_max + 1e-6) {
      return reject("lon_accel");
    }
    if (std::abs(point.kappa) > bounds.curvature) {
      return reject("curvature");
    }
    const double steer = std::atan(point.kappa * wheel_base_m);
    if (std::abs(steer) > bounds.steer_angle) {
      return reject("steer_angle");
    }
    if (std::abs(point.v * point.v * point.kappa) > bounds.lat_accel) {
      return reject("lat_accel");
    }
    if (k + 1 < candidate.points.size()) {
      const auto & next = candidate.points[k + 1];
      if (std::abs((next.a - point.a) / dt) > bounds.lon_jerk) {
        return reject("lon_jerk");
      }
      const double next_steer = std::atan(next.kappa * wheel_base_m);
      if (std::abs((next_steer - steer) / dt) > bounds.steer_rate) {
        return reject("steer_rate");
      }
    }

    // --- geometric constraints, on the projected views ---
    const auto box = footprint_sl_box(context.vehicle_info, s, l);
    const double t0 = point.t;
    const double t1 = (k + 1 < candidate.points.size()) ? candidate.points[k + 1].t : t0;
    double soft_bound_cost = 0.0;
    for (const auto & bound : compiled_constraints.lateral_bounds) {
      const auto & raw = compiled_constraints.raw_constraints[bound.raw_index];
      if (raw.hardness == Hardness::HARD) {
        if (violates_lateral_bound(bound, box)) {
          return reject("lateral_bound");
        }
        continue;
      }
      // A soft boundary, the own lane bound towards a parallel lane, costs slack_weight times the
      // squared amount by which it is exceeded
      double extreme_l = 0.0;
      if (!lateral_bound_extreme_l(bound, box.s_min, box.s_max, extreme_l)) {
        continue;
      }
      const double violation = bound.forbidden_side == Side::LEFT
                                 ? box.l_max - (extreme_l - bound.margin)
                                 : (extreme_l + bound.margin) - box.l_min;
      if (violation > 0.0) {
        soft_bound_cost += raw.slack_weight * violation * violation;
      }
    }
    for (const auto & occupancy : compiled_constraints.occupancies) {
      if (violates_occupancy(occupancy, compiled_constraints, box, t0, t1)) {
        return reject("occupancy");
      }
    }
    for (const auto & stop_bar : compiled_constraints.stop_bars) {
      if (violates_stop_bar(stop_bar, box, t0, t1)) {
        return reject("stop_bar");
      }
    }

    // Soft cost, integrated over time. The lateral reference is the centerline, except that over
    // the last 2B to B before the goal (B being the longest lateral travel) it blends linearly into
    // the lateral position of the goal, and stays there for the remaining B. Starting to approach
    // it only B ahead would need an ever larger curvature as the distance shrinks, and the
    // candidates would be rejected by the steer rate and end up near the centerline
    const double l_ref =
      l_goal * std::clamp((2.0 * blend_length - (s_max - s)) / blend_length, 0.0, 1.0);
    const double dv = v_max - point.v;
    cost += p.weights.lateral * (l - l_ref) * (l - l_ref) * dt;
    cost += soft_bound_cost * dt;
    cost += p.weights.velocity * dv * dv * dt;
    cost += p.weights.curvature * point.kappa * point.kappa * dt;
    if (k + 1 < candidate.points.size()) {
      const double lon_jerk = (candidate.points[k + 1].a - point.a) / dt;
      cost += p.weights.lon_jerk * lon_jerk * lon_jerk * dt;
    }
  }
  candidate.cost = cost;
}

Trajectory FrenetSamplingBasedPlanner::to_trajectory_msg(
  const PlannerContext & context, const Candidate & candidate) const
{
  const double z = context.odometry.pose.pose.position.z;
  const double wheel_base_m = context.vehicle_info.wheel_base_m;

  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = context.odometry.header.stamp;
  trajectory.points.reserve(candidate.points.size());
  for (const auto & point : candidate.points) {
    trajectory.points.push_back(to_trajectory_point(point, z, wheel_base_m));
  }
  const double engage_velocity_mps =
    params_.engage_velocity.enable ? params_.engage_velocity.velocity_hard_mps : 0.0;
  return set_engage_speed(trajectory, engage_velocity_mps);
}

void FrenetSamplingBasedPlanner::append_debug_markers(
  const PlannerContext & context, const std::vector<Candidate> & candidates,
  MarkerArray & debug_markers) const
{
  using autoware_utils_visualization::create_default_marker;
  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;

  const double z = context.odometry.pose.pose.position.z;
  const auto stamp = context.odometry.header.stamp;

  auto valid_marker = create_default_marker(
    "map", stamp, "candidates_valid", 0, Marker::LINE_LIST, create_marker_scale(0.05, 0.0, 0.0),
    create_marker_color(0.0, 1.0, 0.0, 0.5));
  auto invalid_marker = create_default_marker(
    "map", stamp, "candidates_invalid", 0, Marker::LINE_LIST, create_marker_scale(0.03, 0.0, 0.0),
    create_marker_color(1.0, 0.0, 0.0, 0.2));

  for (const auto & candidate : candidates) {
    auto & marker = candidate.valid ? valid_marker : invalid_marker;
    for (std::size_t k = 0; k + 1 < candidate.points.size(); ++k) {
      for (const std::size_t i : {k, k + 1}) {
        geometry_msgs::msg::Point point;
        point.x = candidate.points[i].pose.position.x();
        point.y = candidate.points[i].pose.position.y();
        point.z = z;
        marker.points.push_back(point);
      }
    }
  }
  if (!valid_marker.points.empty()) {
    debug_markers.markers.push_back(valid_marker);
  }
  if (!invalid_marker.points.empty()) {
    debug_markers.markers.push_back(invalid_marker);
  }
}

}  // namespace autoware::safety_planner

PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::FrenetSamplingBasedPlanner,
  autoware::safety_planner::TrajectoryPlannerInterface)
