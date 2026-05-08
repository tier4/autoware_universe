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

#include "trajectory_min_rule_based_path_optimizer.hpp"

#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/trajectory/utils/crop.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils_math/normalization.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

namespace
{

// dt is a CasADi runtime parameter scaling the dynamics, so the time horizon adapts.
constexpr size_t kMinInputPoints = 2;
// constexpr double kDtMin = 0.005;  // floor for runtime dt (5 ms); below this we skip optimisation
constexpr double kDtChangeRelativeReset =
  0.05;  // 5% relative dt change between cycles forces a warm-start reset
// Maximum allowed gap between ego and the previous solution path before we
// abandon the previous-solution anchor and fall back to the input projection.
constexpr double kAnchorMaxDistFromPrevSol = 1.0;
// Maximum forward optimisation horizon [m]. Beyond this distance the input
// trajectory is left untouched: the per-stage spatial step would otherwise
// blow up (s/N) and SQP would have to model regions the controller will
// never reach within one planning cycle.
constexpr double kMaxForwardHorizonM = 100.0;

double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  return tf2::getYaw(q);
}

geometry_msgs::msg::Quaternion quat_from_yaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

// Unwrap `psi` so its difference from `psi_prev` lies in (-pi, pi]. acados'
// LINEAR_LS cost subtracts the reference without angle normalisation, so
// references handed to setStageReference must be continuous across stages.
double unwrap_to(double psi, double psi_prev)
{
  while (psi - psi_prev > M_PI) psi -= 2.0 * M_PI;
  while (psi - psi_prev < -M_PI) psi += 2.0 * M_PI;
  return psi;
}

}  // namespace

void TrajectoryMinRuleBasedPathOptimizer::optimize_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & /*params*/,
  TrajectoryOptimizerData & data)
{
  // Gating happens at the planner level: the plugin is only loaded when the
  // minimum-rule-based planner wants to use it, so here we just run whenever
  // we are invoked.
  if (traj_points.size() < kMinInputPoints) {
    return;
  }
  if (!acados_) {
    return;
  }

  const double L = vehicle_info_.wheel_base_m;
  const double delta_max =
    std::max(0.05, vehicle_info_.max_steer_angle_rad - plugin_params_.steer_margin_rad);
  const double u_delta_max = std::max(1e-3, plugin_params_.u_delta_max_rad_per_s);
  const double dt_max = plugin_params_.dt_max_s;

  const double v_floor = std::max(1e-3, plugin_params_.v_ref_floor_mps);
  const double v_min_box = std::min(plugin_params_.v_min_mps, plugin_params_.v_max_mps - 1e-3);
  const double v_max_box = std::max(plugin_params_.v_max_mps, v_min_box + 1e-3);

  if (L < 1e-3 || vehicle_info_.max_steer_angle_rad < 1e-3) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "[min_rule_based_path_optimizer] Invalid vehicle_info (L=%.3f, delta_max=%.3f).", L,
      vehicle_info_.max_steer_angle_rad);
    return;
  }

  // ---- 1. Build a Trajectory<TrajectoryPoint> from the input ----
  // pretty_build does the arc-length parametrisation + spline interpolation
  // for us; downstream we sample (X, Y, ψ, v, κ) via compute()/curvature(),
  // so the manual cumulative-distance / yaw-unwrap loop is no longer needed.
  const auto in_traj_opt = autoware::experimental::trajectory::pretty_build(traj_points);
  if (!in_traj_opt) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "[min_rule_based_path_optimizer] pretty_build() failed; skipping optimisation.");
    return;
  }
  const auto & in_traj = *in_traj_opt;
  const double s_total = in_traj.length();
  if (s_total <= 1e-3) {
    return;
  }

  // Project ego onto the input trajectory. Upstream stages prepend a few
  // metres of path behind base_link, so traj_points.front() is *not* the
  // current ego pose; using s = 0 as the start of the OCP would optimise
  // already-passed history. We instead crop the trajectory at the closest
  // projection and run the OCP on the forward slice only.
  const auto & ego_pose = data.current_odometry.pose.pose;
  const double s_ego = autoware::experimental::trajectory::closest(in_traj, ego_pose.position);
  // Cap the forward horizon at kMaxForwardHorizonM so the OCP discretisation
  // step (s/N) stays bounded on long inputs.
  const double fwd_length = std::min(s_total - s_ego, kMaxForwardHorizonM);
  const auto fwd_traj = autoware::experimental::trajectory::crop(in_traj, s_ego, fwd_length);
  // After cropping, `fwd_traj`'s arc length runs from 0 (= ego projection) to
  // s_horizon_in (≤ kMaxForwardHorizonM), so all downstream sampling can use
  // the local frame s ∈ [0, s_horizon_in].
  const double s_horizon_in = fwd_traj.length();
  if (s_horizon_in <= 1e-3) {
    return;
  }

  // Reference (nominal) speed used to size dt and to feed the v reference.
  // Sampled along the forward slice only — including the pre-ego history
  // would skew dt sizing when the upstream pads with low-speed points.
  constexpr size_t kVSampleCount = 32;
  double v_ref_avg = 0.0;
  for (size_t i = 0; i < kVSampleCount; ++i) {
    const double s_local =
      (static_cast<double>(i) + 0.5) * s_horizon_in / static_cast<double>(kVSampleCount);
    v_ref_avg += std::abs(static_cast<double>(fwd_traj.compute(s_local).longitudinal_velocity_mps));
  }
  v_ref_avg /= static_cast<double>(kVSampleCount);
  v_ref_avg = std::max(v_floor, v_ref_avg);
  // Keep v_ref strictly inside the input box so the unconstrained tracking term
  // can reach it without bumping into a constraint.
  v_ref_avg = clamp(v_ref_avg, v_min_box + 1e-3, v_max_box - 1e-3);

  // Fixed dt: each stage advances `dt_max_s` of physical time regardless of
  // input length or speed. The OCP's spatial horizon then becomes a
  // consequence of v(t) the solver picks (bounded by [v_min, v_max] · N · dt).
  // v_ref_avg is no longer used for sizing dt and only feeds debug logs.
  const double dt_runtime = dt_max;

  // ---- 2. Initial state ----
  // Anchor pose (X0, Y0, psi0) selection — three-tier policy:
  //   (a) First call / no previous solution → project ego onto the input
  //       (= cropped front). This snaps the OCP to the input geometry, which
  //       is the only reasonable choice when we have no prior context.
  //   (b) Previous solution exists AND ego is within kAnchorMaxDistFromPrevSol
  //       metres of it → project ego onto the previous solution and use that
  //       projection as the anchor. This avoids the per-cycle "snap back to
  //       input" jitter that otherwise pulls the optimizer's output toward
  //       the (un-smoothed) input geometry whenever the solver wants to
  //       deviate from it.
  //   (c) Previous solution exists but ego has drifted too far from it
  //       (lane change, replan, large disturbance) → fall back to (a).
  // delta0: previous solution if available, else 0.
  double X0;
  double Y0;
  double psi0;
  bool anchor_from_prev_sol = false;
  if (last_world_sol_traj_) {
    const double s_prev =
      autoware::experimental::trajectory::closest(*last_world_sol_traj_, ego_pose.position);
    const auto prev_anchor = last_world_sol_traj_->compute(s_prev);
    const double dist_to_prev = std::hypot(
      prev_anchor.pose.position.x - ego_pose.position.x,
      prev_anchor.pose.position.y - ego_pose.position.y);
    if (dist_to_prev < kAnchorMaxDistFromPrevSol) {
      X0 = prev_anchor.pose.position.x;
      Y0 = prev_anchor.pose.position.y;
      psi0 = yaw_from_quat(prev_anchor.pose.orientation);
      anchor_from_prev_sol = true;
    }
  }
  if (!anchor_from_prev_sol) {
    const auto fwd_front = fwd_traj.compute(0.0);
    X0 = fwd_front.pose.position.x;
    Y0 = fwd_front.pose.position.y;
    psi0 = yaw_from_quat(fwd_front.pose.orientation);
  }
  double delta0 = 0.0;
  if (has_last_solution_) {
    // Take the second stage so that warm-start consistency holds.
    delta0 = last_solution_.xtraj[1][3];
  }
  delta0 = clamp(delta0, -delta_max, delta_max);

  // Local frame: ego-centred (X0, Y0). Heading kept in raw radians.
  const std::array<double, time_mpt::NX> x0_local = {0.0, 0.0, psi0, delta0};

  // ---- 3. Acados parameters, box constraints, and cost weights ----
  acados_->setParametersAllStages({L, dt_runtime});
  acados_->setDeltaBoxAllStages(delta_max);
  acados_->setInputBoxAllStages(u_delta_max, v_min_box, v_max_box);
  acados_->setStageCostDiagonalAllStages(
    plugin_params_.w_stage_position_xy, plugin_params_.w_stage_position_xy,
    plugin_params_.w_stage_yaw, plugin_params_.w_stage_delta, plugin_params_.w_stage_steer_rate,
    plugin_params_.w_stage_velocity);
  acados_->setTerminalCostDiagonal(
    plugin_params_.w_terminal_position_xy, plugin_params_.w_terminal_position_xy,
    plugin_params_.w_terminal_yaw, plugin_params_.w_terminal_delta);

  // ---- 4. Reference sampling ----
  // Each stage k targets s_k = (k/N) * s_horizon_in along the cropped
  // forward trajectory. v_ref_k is sampled from the input's
  // longitudinal_velocity at that arc length (clamped to the input box).
  // With v as a free input, the solver can deviate from v_ref to make the
  // integrated path land on the terminal pose — this is the extra freedom
  // the time-axis formulation buys us.
  const double s_per_stage = s_horizon_in / static_cast<double>(time_mpt::N);
  // The orientation interpolator returns a normalised quaternion, so converting
  // back to yaw gives a value in (-pi, pi]. Unwrap stage-by-stage so the cost
  // term sees a continuous psi_ref across the horizon.
  double psi_ref_prev = psi0;

  const bool publish_markers = plugin_params_.enable_debug_topics;
  visualization_msgs::msg::MarkerArray debug_markers;
  const auto stamp = get_node_ptr()->now();
  if (publish_markers) {
    // Clear previous frame's markers first.
    visualization_msgs::msg::Marker clear_m;
    clear_m.header.frame_id = "map";
    clear_m.header.stamp = stamp;
    clear_m.action = visualization_msgs::msg::Marker::DELETEALL;
    debug_markers.markers.push_back(clear_m);
    // Anchor (X0, Y0) — green sphere.
    visualization_msgs::msg::Marker anchor_m;
    anchor_m.header.frame_id = "map";
    anchor_m.header.stamp = stamp;
    anchor_m.ns = "anchor";
    anchor_m.id = 0;
    anchor_m.type = visualization_msgs::msg::Marker::SPHERE;
    anchor_m.action = visualization_msgs::msg::Marker::ADD;
    anchor_m.pose.position.x = X0;
    anchor_m.pose.position.y = Y0;
    anchor_m.pose.position.z = ego_pose.position.z;
    anchor_m.pose.orientation.w = 1.0;
    anchor_m.scale.x = 0.5;
    anchor_m.scale.y = 0.5;
    anchor_m.scale.z = 0.5;
    anchor_m.color.r = 0.0f;
    anchor_m.color.g = 1.0f;
    anchor_m.color.b = 0.0f;
    anchor_m.color.a = 1.0f;
    debug_markers.markers.push_back(anchor_m);
  }

  for (size_t k = 0; k <= time_mpt::N; ++k) {
    const double s_k = static_cast<double>(k) * s_per_stage;
    const auto ref_point = fwd_traj.compute(s_k);
    const double Xref = ref_point.pose.position.x;
    const double Yref = ref_point.pose.position.y;
    const double psi_ref = unwrap_to(yaw_from_quat(ref_point.pose.orientation), psi_ref_prev);
    psi_ref_prev = psi_ref;

    if (k < time_mpt::N) {
      double v_ref_k = std::abs(static_cast<double>(ref_point.longitudinal_velocity_mps));
      v_ref_k = clamp(v_ref_k, v_min_box, v_max_box);
      if (v_ref_k < v_floor) v_ref_k = v_floor;
      std::array<double, time_mpt::NY> yref{Xref - X0, Yref - Y0, psi_ref, 0.0, 0.0, v_ref_k};
      acados_->setStageReference(static_cast<int>(k), yref);
    } else {
      std::array<double, time_mpt::NYN> yref_e{Xref - X0, Yref - Y0, psi_ref, 0.0};
      acados_->setTerminalReference(yref_e);
    }

    if (publish_markers) {
      visualization_msgs::msg::Marker ref_m;
      ref_m.header.frame_id = "map";
      ref_m.header.stamp = stamp;
      ref_m.ns = "reference";
      ref_m.id = static_cast<int>(k);
      ref_m.type = visualization_msgs::msg::Marker::SPHERE;
      ref_m.action = visualization_msgs::msg::Marker::ADD;
      ref_m.pose.position.x = Xref;
      ref_m.pose.position.y = Yref;
      ref_m.pose.position.z = ego_pose.position.z;
      ref_m.pose.orientation.w = 1.0;
      ref_m.scale.x = 0.2;
      ref_m.scale.y = 0.2;
      ref_m.scale.z = 0.2;
      ref_m.color.r = 1.0f;
      ref_m.color.g = 0.0f;
      ref_m.color.b = 0.0f;
      ref_m.color.a = 1.0f;
      debug_markers.markers.push_back(ref_m);
    }
  }
  if (publish_markers) {
    pub_debug_markers_->publish(debug_markers);
  }

  // ---- 5. Warm start ----
  // Always use the dynamics-consistent analytic warm start built from the
  // input geometry: positions/yaws follow the reference path; delta comes
  // directly from the path curvature κ via δ = atan(L · κ), which is exactly
  // the kinematic-bicycle relation; v matches the input speed profile;
  // u_delta is the finite-difference of delta scaled by 1/dt.
  //
  // Reusing the previous cycle's solution as warm start would be tempting
  // but its xtraj is in the previous local frame (anchor X0/Y0/psi0 differs
  // from the new cycle's anchor) and on the previous time grid (different
  // dt). Full SQP, unlike SQP_RTI's single iteration, does not tolerate this
  // frame inconsistency — it stalls trying to repair the dynamics defects.
  // The analytic warm start is cheap and gives every solve a clean,
  // frame-correct starting point.
  {
    std::array<std::array<double, time_mpt::NX>, time_mpt::N + 1> xtraj_init{};
    std::array<std::array<double, time_mpt::NU>, time_mpt::N> utraj_init{};
    double psi_init_prev = psi0;
    for (size_t k = 0; k <= time_mpt::N; ++k) {
      const double s_k = static_cast<double>(k) * s_per_stage;
      const auto p = fwd_traj.compute(s_k);
      xtraj_init[k][0] = p.pose.position.x - X0;
      xtraj_init[k][1] = p.pose.position.y - Y0;
      const double psi_init = unwrap_to(yaw_from_quat(p.pose.orientation), psi_init_prev);
      psi_init_prev = psi_init;
      xtraj_init[k][2] = psi_init;
    }
    xtraj_init[0][3] = delta0;
    const double delta_clamp = delta_max - 1e-3;
    for (size_t k = 1; k <= time_mpt::N; ++k) {
      const double s_k = static_cast<double>(k) * s_per_stage;
      const double kappa = fwd_traj.curvature(s_k);
      const double d = std::atan(L * kappa);
      xtraj_init[k][3] = clamp(d, -delta_clamp, delta_clamp);
    }
    // u_delta = ddelta/dt; v from input speed profile clamped to the box.
    for (size_t k = 0; k < time_mpt::N; ++k) {
      const double u_phys = (xtraj_init[k + 1][3] - xtraj_init[k][3]) / dt_runtime;
      utraj_init[k][0] = clamp(u_phys, -u_delta_max, u_delta_max);
      const double s_k = static_cast<double>(k) * s_per_stage;
      double v_init =
        std::abs(static_cast<double>(fwd_traj.compute(s_k).longitudinal_velocity_mps));
      v_init = clamp(v_init, v_min_box, v_max_box);
      if (v_init < v_floor) v_init = v_floor;
      utraj_init[k][1] = v_init;
    }
    acados_->setWarmStart(xtraj_init, utraj_init);
  }

  // ---- 6. Solve ----
  auto solution = acados_->getControl(x0_local);

  // Accept "max iter reached" (status==2) when KKT is small enough that the
  // partially-converged solution is still usable. The downstream out-of-band
  // check (lateral / yaw deviation vs the input) is the actual quality gate;
  // rejecting on status alone discards usable paths on tight curves where
  // full SQP rarely drives KKT all the way to `tol`.
  constexpr double kKktAcceptThreshold = 0.2;
  const bool solver_accepted =
    solution.status == 0 || (solution.status == 2 && solution.kkt_norm_inf < kKktAcceptThreshold);
  if (!solver_accepted) {
    RCLCPP_ERROR(
      get_node_ptr()->get_logger(), "[min_rule_based_path_optimizer] solve failed (status=%d, %s).",
      solution.status, solution.info.c_str());
    return;
  }

  const double elapsed_ms = solution.elapsed_time * 1000.0;
  if (elapsed_ms > plugin_params_.elapsed_time_warn_threshold_ms) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "[min_rule_based_path_optimizer] solve elapsed %.2f ms exceeds threshold (%.1f ms).",
      elapsed_ms, plugin_params_.elapsed_time_warn_threshold_ms);
  }

  // ---- 7. Validation + write-back via a Trajectory built on the solution ----
  // The solution lives on the time grid (uniform dt). Building a
  // Trajectory<TrajectoryPoint> from the per-stage poses gives us:
  //   - an arc-length parametrisation of the solution (sol_traj.length()),
  //   - compute()/curvature() for resampling onto the input's arc-length grid.
  // We reuse the input's velocity/accel/time-from-start fields so only the
  // pose is overwritten.
  std::vector<TrajectoryPoint> sol_points;
  sol_points.reserve(time_mpt::N + 1);
  for (size_t k = 0; k <= time_mpt::N; ++k) {
    TrajectoryPoint pt{};
    pt.pose.position.x = solution.xtraj[k][0] + X0;
    pt.pose.position.y = solution.xtraj[k][1] + Y0;
    pt.pose.orientation = quat_from_yaw(solution.xtraj[k][2]);
    sol_points.push_back(pt);
  }
  const auto sol_traj_opt = autoware::experimental::trajectory::pretty_build(sol_points);
  if (!sol_traj_opt) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 2000,
      "[min_rule_based_path_optimizer] could not build solution trajectory; keeping warm-start.");
    return;
  }
  const auto & sol_traj = *sol_traj_opt;
  const double s_sol_horizon = sol_traj.length();

  // Validation: at each stage's solution arc length, compare against the
  // forward input at the same local arc length (both share s = 0 at ego).
  double max_lat_err = 0.0;
  double max_yaw_err = 0.0;
  size_t k_lat = 0;
  size_t k_yaw = 0;
  const auto sol_bases = sol_traj.get_underlying_bases();
  for (size_t k = 0; k < sol_bases.size(); ++k) {
    const double s_query = std::min(sol_bases[k], s_horizon_in);
    const auto sol_pt = sol_traj.compute(sol_bases[k]);
    const auto ref_pt = fwd_traj.compute(s_query);
    const double lat_err = std::hypot(
      sol_pt.pose.position.x - ref_pt.pose.position.x,
      sol_pt.pose.position.y - ref_pt.pose.position.y);
    const double yaw_err = std::abs(autoware_utils_math::normalize_radian(
      yaw_from_quat(sol_pt.pose.orientation) - yaw_from_quat(ref_pt.pose.orientation)));
    if (lat_err > max_lat_err) {
      max_lat_err = lat_err;
      k_lat = k;
    }
    if (yaw_err > max_yaw_err) {
      max_yaw_err = yaw_err;
      k_yaw = k;
    }
  }
  const bool solution_out_of_band =
    (max_lat_err > plugin_params_.max_lat_error_from_input_m ||
     max_yaw_err > plugin_params_.max_yaw_error_from_input_rad);
  if (solution_out_of_band) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 2000,
      "[min_rule_based_path_optimizer] solution out of band "
      "(max_lat_err=%.2f m @stage=%zu, max_yaw_err=%.3f rad @stage=%zu, "
      "sqp_iter=%d, %.2f ms, s_ego=%.1f / s_total=%.1f m, dt=%.3f s, v_ref=%.2f m/s). "
      "Keeping warm-start.",
      max_lat_err, k_lat, max_yaw_err, k_yaw, solution.sqp_iter, elapsed_ms, s_ego, s_total,
      dt_runtime, v_ref_avg);
  }

  // ---- 8. Write back: replace traj_points with the optimized region only ----
  // Output covers exactly the OCP region [s_ego, s_ego + s_sol_horizon]:
  //   - pre-ego history (upstream padding behind base_link) is dropped,
  //   - any portion past the solution horizon is also dropped.
  // For each kept index, velocity / acceleration / time_from_start are taken
  // from the original input point so only pose is overwritten by the OCP.
  if (!solution_out_of_band || 1) {
    const auto in_bases = in_traj.get_underlying_bases();
    const size_t n_in = std::min(in_bases.size(), traj_points.size());
    TrajectoryPoints optimized;
    optimized.reserve(n_in);
    for (size_t i = 0; i < n_in; ++i) {
      if (in_bases[i] < s_ego) {
        continue;  // pre-ego history; drop
      }
      const double s_query_sol = in_bases[i] - s_ego;
      if (s_query_sol > s_sol_horizon) {
        break;  // past the optimisation horizon; drop
      }
      auto pt = traj_points[i];
      const auto out_pt = sol_traj.compute(s_query_sol);
      pt.pose.position.x = out_pt.pose.position.x;
      pt.pose.position.y = out_pt.pose.position.y;
      // z preserved
      pt.pose.orientation = out_pt.pose.orientation;
      optimized.push_back(pt);
    }
    traj_points = std::move(optimized);
    // Persist the world-frame solution so the next cycle can anchor on it
    // instead of snapping back to the input projection. Out-of-band solves
    // intentionally leave the previous good trajectory in place.
    last_world_sol_traj_ = sol_traj;
  }

  if (plugin_params_.enable_debug_topics) {
    RCLCPP_DEBUG_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 1000,
      "[min_rule_based_path_optimizer] solve %.2f ms, sqp_iter=%d, max_lat_err=%.2f m.", elapsed_ms,
      solution.sqp_iter, max_lat_err);
  }

  // ---- 9. Save warm-start state ----
  last_solution_ = solution;
  has_last_solution_ = true;
  last_x0_world_x_ = X0;
  last_x0_world_y_ = Y0;
  last_x0_world_yaw_ = psi0;
  last_dt_runtime_ = dt_runtime;
}

void TrajectoryMinRuleBasedPathOptimizer::set_up_params()
{
  auto * node_ptr = get_node_ptr();
  using autoware_utils_rclcpp::get_or_declare_parameter;

  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*node_ptr).getVehicleInfo();

  plugin_params_.dt_max_s =
    get_or_declare_parameter<double>(*node_ptr, "min_rule_based_path_optimizer.horizon.dt_max_s");
  plugin_params_.v_ref_floor_mps = get_or_declare_parameter<double>(
    *node_ptr, "min_rule_based_path_optimizer.horizon.v_ref_floor_mps");
  plugin_params_.u_delta_max_rad_per_s = get_or_declare_parameter<double>(
    *node_ptr, "min_rule_based_path_optimizer.input.u_delta_max_rad_per_s");
  plugin_params_.v_min_mps =
    get_or_declare_parameter<double>(*node_ptr, "min_rule_based_path_optimizer.input.v_min_mps");
  plugin_params_.v_max_mps =
    get_or_declare_parameter<double>(*node_ptr, "min_rule_based_path_optimizer.input.v_max_mps");
  plugin_params_.steer_margin_rad = get_or_declare_parameter<double>(
    *node_ptr, "min_rule_based_path_optimizer.vehicle.steer_margin_rad");
  plugin_params_.w_stage_position_xy = get_or_declare_parameter<double>(
    *node_ptr, "min_rule_based_path_optimizer.weights.stage.position_xy");
  plugin_params_.w_stage_yaw =
    get_or_declare_parameter<double>(*node_ptr, "min_rule_based_path_optimizer.weights.stage.yaw");
  plugin_params_.w_stage_delta = get_or_declare_parameter<double>(
    *node_ptr, "min_rule_based_path_optimizer.weights.stage.delta");
  plugin_params_.w_stage_steer_rate = get_or_declare_parameter<double>(
    *node_ptr, "min_rule_based_path_optimizer.weights.stage.steer_rate");
  plugin_params_.w_stage_velocity = get_or_declare_parameter<double>(
    *node_ptr, "min_rule_based_path_optimizer.weights.stage.velocity");
  plugin_params_.w_terminal_position_xy = get_or_declare_parameter<double>(
    *node_ptr, "min_rule_based_path_optimizer.weights.terminal.position_xy");
  plugin_params_.w_terminal_yaw = get_or_declare_parameter<double>(
    *node_ptr, "min_rule_based_path_optimizer.weights.terminal.yaw");
  plugin_params_.w_terminal_delta = get_or_declare_parameter<double>(
    *node_ptr, "min_rule_based_path_optimizer.weights.terminal.delta");
  plugin_params_.warm_start_enable =
    get_or_declare_parameter<bool>(*node_ptr, "min_rule_based_path_optimizer.warm_start.enable");
  plugin_params_.reset_pose_threshold_m = get_or_declare_parameter<double>(
    *node_ptr, "min_rule_based_path_optimizer.warm_start.reset_pose_threshold_m");
  plugin_params_.reset_yaw_threshold_rad = get_or_declare_parameter<double>(
    *node_ptr, "min_rule_based_path_optimizer.warm_start.reset_yaw_threshold_rad");
  plugin_params_.max_lat_error_from_input_m = get_or_declare_parameter<double>(
    *node_ptr, "min_rule_based_path_optimizer.validation.max_lat_error_from_input_m");
  plugin_params_.max_yaw_error_from_input_rad = get_or_declare_parameter<double>(
    *node_ptr, "min_rule_based_path_optimizer.validation.max_yaw_error_from_input_rad");
  plugin_params_.elapsed_time_warn_threshold_ms = get_or_declare_parameter<double>(
    *node_ptr, "min_rule_based_path_optimizer.debug.elapsed_time_warn_threshold_ms");
  plugin_params_.publish_replay_fixture_on_failure = get_or_declare_parameter<bool>(
    *node_ptr, "min_rule_based_path_optimizer.debug.publish_replay_fixture_on_failure");
  plugin_params_.enable_debug_topics = get_or_declare_parameter<bool>(
    *node_ptr, "min_rule_based_path_optimizer.debug.enable_debug_topics");

  acados_ = std::make_unique<time_mpt::AcadosInterfaceTime>();

  pub_debug_markers_ = node_ptr->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/debug/min_rule_based_path_optimizer/markers", rclcpp::QoS{1});

  RCLCPP_INFO(
    node_ptr->get_logger(),
    "[min_rule_based_path_optimizer] initialized. L=%.3f m, delta_max=%.3f rad, "
    "u_delta_max=%.3f rad/s, v=[%.2f, %.2f] m/s, N=%zu, dt=%.3f s (fixed).",
    vehicle_info_.wheel_base_m, vehicle_info_.max_steer_angle_rad,
    plugin_params_.u_delta_max_rad_per_s, plugin_params_.v_min_mps, plugin_params_.v_max_mps,
    time_mpt::N, plugin_params_.dt_max_s);
}

rcl_interfaces::msg::SetParametersResult TrajectoryMinRuleBasedPathOptimizer::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.horizon.dt_max_s", plugin_params_.dt_max_s);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.horizon.v_ref_floor_mps",
    plugin_params_.v_ref_floor_mps);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.input.u_delta_max_rad_per_s",
    plugin_params_.u_delta_max_rad_per_s);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.input.v_min_mps", plugin_params_.v_min_mps);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.input.v_max_mps", plugin_params_.v_max_mps);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.weights.stage.position_xy",
    plugin_params_.w_stage_position_xy);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.weights.stage.yaw", plugin_params_.w_stage_yaw);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.weights.stage.delta", plugin_params_.w_stage_delta);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.weights.stage.steer_rate",
    plugin_params_.w_stage_steer_rate);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.weights.stage.velocity",
    plugin_params_.w_stage_velocity);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.weights.terminal.position_xy",
    plugin_params_.w_terminal_position_xy);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.weights.terminal.yaw",
    plugin_params_.w_terminal_yaw);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.weights.terminal.delta",
    plugin_params_.w_terminal_delta);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.vehicle.steer_margin_rad",
    plugin_params_.steer_margin_rad);
  update_param<bool>(
    parameters, "min_rule_based_path_optimizer.warm_start.enable",
    plugin_params_.warm_start_enable);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.warm_start.reset_pose_threshold_m",
    plugin_params_.reset_pose_threshold_m);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.warm_start.reset_yaw_threshold_rad",
    plugin_params_.reset_yaw_threshold_rad);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.validation.max_lat_error_from_input_m",
    plugin_params_.max_lat_error_from_input_m);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.validation.max_yaw_error_from_input_rad",
    plugin_params_.max_yaw_error_from_input_rad);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.debug.elapsed_time_warn_threshold_ms",
    plugin_params_.elapsed_time_warn_threshold_ms);
  update_param<bool>(
    parameters, "min_rule_based_path_optimizer.debug.publish_replay_fixture_on_failure",
    plugin_params_.publish_replay_fixture_on_failure);
  update_param<bool>(
    parameters, "min_rule_based_path_optimizer.debug.enable_debug_topics",
    plugin_params_.enable_debug_topics);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::trajectory_optimizer::plugin
