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

#include "path_optimizer.hpp"

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
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
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
constexpr double kMaxForwardHorizonM = 200.0;
// Maximum gap between ego and the nearest point on the previous successful
// output before we abandon the previous-output fallback and let the caller
// fall back to the raw input.
constexpr double kFallbackMaxDistFromPrevSolM = 0.2;

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

// Closest point on the line segment (A, B) to query Q (2D). Returns the
// perpendicular distance and the projection point. A degenerate segment
// (|B-A| ≈ 0) collapses to A.
std::pair<double, geometry_msgs::msg::Point> closest_point_on_segment(
  const geometry_msgs::msg::Point & Q, const geometry_msgs::msg::Point & A,
  const geometry_msgs::msg::Point & B)
{
  const double tx = B.x - A.x;
  const double ty = B.y - A.y;
  const double t2 = tx * tx + ty * ty;
  double s = 0.0;
  if (t2 > 1e-12) {
    s = ((Q.x - A.x) * tx + (Q.y - A.y) * ty) / t2;
    s = std::clamp(s, 0.0, 1.0);
  }
  geometry_msgs::msg::Point P;
  P.x = A.x + s * tx;
  P.y = A.y + s * ty;
  P.z = 0.0;
  const double dx = Q.x - P.x;
  const double dy = Q.y - P.y;
  return {std::hypot(dx, dy), P};
}

}  // namespace

PathOptimizer::PathOptimizer(const std::string & name, rclcpp::Node * node_ptr)
: name_(name), node_ptr_(node_ptr)
{
  set_up_params();
}

bool PathOptimizer::try_apply_previous_output_fallback(
  TrajectoryPoints & traj_points, const geometry_msgs::msg::Pose & ego_pose) const
{
  if (!last_output_traj_) {
    return false;
  }
  const auto & prev = *last_output_traj_;

  const double s_nearest = autoware::experimental::trajectory::closest(prev, ego_pose.position);
  const auto nearest_pt = prev.compute(s_nearest);
  const double dist = std::hypot(
    nearest_pt.pose.position.x - ego_pose.position.x,
    nearest_pt.pose.position.y - ego_pose.position.y);
  if (dist > kFallbackMaxDistFromPrevSolM) {
    return false;
  }

  // crop()'s first arg is offset from current start_; the stored trajectory
  // is freshly built so start_ = 0 and s_nearest is in [0, length].
  const double remaining = prev.length() - s_nearest;
  if (remaining <= 1e-3) {
    return false;
  }
  const auto cropped = autoware::experimental::trajectory::crop(prev, s_nearest, remaining);
  auto out = cropped.restore();
  if (out.size() < kMinInputPoints) {
    return false;
  }

  traj_points = std::move(out);
  return true;
}

std::vector<PathOptimizer::SelectedHalfSpace>
PathOptimizer::select_uncrossable_active_set_for_stage(
  const std::array<geometry_msgs::msg::Point, 3> & warm_start_circles,
  const std::vector<UncrossableSegment> & segments) const
{
  std::vector<SelectedHalfSpace> result;
  if (segments.empty() || !uncrossable_params_.enable) {
    return result;
  }
  const int M = std::max(1, uncrossable_params_.K_max_per_stage);

  // Stage centre for azimuth binning: middle vehicle circle. Front/rear
  // circles see the same K segments; per-circle distance still drives the
  // activity score so a segment relevant only to e.g. the front circle still
  // makes it through.
  const auto & stage_center = warm_start_circles[1];

  struct Candidate
  {
    size_t segment_idx{0};
    size_t closest_circle_idx{0};
    double min_clearance_m{0.0};  // min over circles of (perp_dist - r)
    geometry_msgs::msg::Point anchor;
    double azimuth{0.0};
  };

  std::vector<Candidate> candidates;
  candidates.reserve(segments.size());

  // Layer 3/4 (consolidated): per-segment, take the most-active circle. Drop
  // the segment when even the closest circle has more than
  // `clearance_drop_threshold_m` of room — beyond that the constraint cannot
  // become active over the SQP step.
  const double R_drop = uncrossable_params_.clearance_drop_threshold_m;
  for (size_t s = 0; s < segments.size(); ++s) {
    const auto & seg = segments[s];
    double best_clearance = std::numeric_limits<double>::infinity();
    size_t best_circle = 0;
    geometry_msgs::msg::Point best_anchor;
    for (size_t c = 0; c < 3; ++c) {
      const auto [d, P] = closest_point_on_segment(warm_start_circles[c], seg.p_a, seg.p_b);
      const double clearance = d - vehicle_circles_[c].radius_m;
      if (clearance < best_clearance) {
        best_clearance = clearance;
        best_circle = c;
        best_anchor = P;
      }
    }
    if (best_clearance > R_drop) continue;
    Candidate cnd;
    cnd.segment_idx = s;
    cnd.closest_circle_idx = best_circle;
    cnd.min_clearance_m = best_clearance;
    cnd.anchor = best_anchor;
    cnd.azimuth = std::atan2(best_anchor.y - stage_center.y, best_anchor.x - stage_center.x);
    candidates.push_back(cnd);
  }

  // Layer 5: directional binning. M equal-angle bins of 2π/M; per bin, keep
  // the candidate with the smallest min_clearance (most active).
  std::vector<std::optional<Candidate>> per_bin(M);
  for (const auto & c : candidates) {
    int b =
      static_cast<int>(std::floor((c.azimuth + M_PI) / (2.0 * M_PI) * static_cast<double>(M)));
    if (b < 0) b = 0;
    if (b >= M) b = M - 1;
    if (!per_bin[b] || c.min_clearance_m < per_bin[b]->min_clearance_m) {
      per_bin[b] = c;
    }
  }

  // Layer 6: emit half-spaces. Inward normal points from the segment toward
  // the closest warm-start circle (no orientation convention assumed on the
  // polyline). For visualisation we don't pad with dummies; downstream
  // rendering handles the empty bins.
  result.reserve(static_cast<size_t>(M));
  for (int b = 0; b < M; ++b) {
    if (!per_bin[b]) continue;
    const auto & c = *per_bin[b];
    const auto & seg = segments[c.segment_idx];
    const double tx_seg = seg.p_b.x - seg.p_a.x;
    const double ty_seg = seg.p_b.y - seg.p_a.y;
    const double tlen = std::hypot(tx_seg, ty_seg);
    if (tlen < 1e-9) continue;  // degenerate segment, skip
    const double n_left_x = -ty_seg / tlen;
    const double n_left_y = tx_seg / tlen;
    // Pick the side that points from the anchor toward the warm-start circle
    // centre, i.e. into the drivable region.
    const double dx = warm_start_circles[c.closest_circle_idx].x - c.anchor.x;
    const double dy = warm_start_circles[c.closest_circle_idx].y - c.anchor.y;
    const double sgn = (dx * n_left_x + dy * n_left_y) >= 0.0 ? 1.0 : -1.0;

    SelectedHalfSpace hs;
    hs.p_a = seg.p_a;
    hs.p_b = seg.p_b;
    hs.anchor = c.anchor;
    hs.n_x = sgn * n_left_x;
    hs.n_y = sgn * n_left_y;
    hs.min_clearance_m = c.min_clearance_m;
    hs.azimuth_bin = b;
    result.push_back(hs);
  }
  return result;
}

void PathOptimizer::optimize_trajectory(
  TrajectoryPoints & traj_points, const Odometry & odometry,
  const std::vector<UncrossableSegment> & uncrossable_segments)
{
  const auto & ego_pose = odometry.pose.pose;

  if (traj_points.size() < kMinInputPoints) {
    try_apply_previous_output_fallback(traj_points, ego_pose);
    return;
  }
  if (!acados_) {
    try_apply_previous_output_fallback(traj_points, ego_pose);
    return;
  }

  const double L = vehicle_info_.wheel_base_m;
  const double delta_max =
    std::max(0.05, vehicle_info_.max_steer_angle_rad - plugin_params_.steer_margin_rad);
  const double u_delta_max = std::max(1e-3, plugin_params_.u_delta_max_rad_per_s);
  // Two-stage dt schedule. dt_schedule[k] is the physical time step of stage
  // k; t_cumulative[k] is the cumulative physical time at stage k (so
  // t_cumulative[0] = 0 and t_cumulative[N] = total horizon). Used both for
  // setParameters and for non-uniform reference sampling (s_k below).
  const int near_count =
    std::clamp(plugin_params_.near_stage_count, 0, static_cast<int>(time_mpt::N));
  const double dt_near = std::max(0.001, plugin_params_.dt_near_s);
  const double dt_far = std::max(0.001, plugin_params_.dt_far_s);
  std::array<double, time_mpt::N> dt_schedule{};
  std::array<double, time_mpt::N + 1> t_cumulative{};
  t_cumulative[0] = 0.0;
  for (size_t k = 0; k < time_mpt::N; ++k) {
    dt_schedule[k] = (static_cast<int>(k) < near_count) ? dt_near : dt_far;
    t_cumulative[k + 1] = t_cumulative[k] + dt_schedule[k];
  }
  const double t_total = t_cumulative[time_mpt::N];
  if (t_total <= 1e-6) {
    try_apply_previous_output_fallback(traj_points, ego_pose);
    return;
  }

  const double v_floor = std::max(1e-3, plugin_params_.v_ref_floor_mps);
  const double v_min_box = std::min(plugin_params_.v_min_mps, plugin_params_.v_max_mps - 1e-3);
  const double v_max_box = std::max(plugin_params_.v_max_mps, v_min_box + 1e-3);

  if (L < 1e-3 || vehicle_info_.max_steer_angle_rad < 1e-3) {
    RCLCPP_WARN_THROTTLE(
      node_ptr_->get_logger(), *node_ptr_->get_clock(), 5000,
      "[min_rule_based_path_optimizer] Invalid vehicle_info (L=%.3f, delta_max=%.3f).", L,
      vehicle_info_.max_steer_angle_rad);
    try_apply_previous_output_fallback(traj_points, ego_pose);
    return;
  }

  // ---- 1. Build a Trajectory<TrajectoryPoint> from the input ----
  // pretty_build does the arc-length parametrisation + spline interpolation
  // for us; downstream we sample (X, Y, ψ, v, κ) via compute()/curvature(),
  // so the manual cumulative-distance / yaw-unwrap loop is no longer needed.
  const auto in_traj_opt = autoware::experimental::trajectory::pretty_build(traj_points);
  if (!in_traj_opt) {
    RCLCPP_WARN_THROTTLE(
      node_ptr_->get_logger(), *node_ptr_->get_clock(), 5000,
      "[min_rule_based_path_optimizer] pretty_build() failed; skipping optimisation.");
    try_apply_previous_output_fallback(traj_points, ego_pose);
    return;
  }
  const auto & in_traj = *in_traj_opt;
  const double s_total = in_traj.length();
  if (s_total <= 1e-3) {
    try_apply_previous_output_fallback(traj_points, ego_pose);
    return;
  }

  // Project ego onto the input trajectory. Upstream stages prepend a few
  // metres of path behind base_link, so traj_points.front() is *not* the
  // current ego pose; using s = 0 as the start of the OCP would optimise
  // already-passed history. We instead crop the trajectory at the closest
  // projection and run the OCP on the forward slice only.
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
    try_apply_previous_output_fallback(traj_points, ego_pose);
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
  if (last_output_traj_) {
    const double s_prev =
      autoware::experimental::trajectory::closest(*last_output_traj_, ego_pose.position);
    const auto prev_anchor = last_output_traj_->compute(s_prev);
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
  // Set L for every stage including the terminal (terminal does not integrate
  // but the parameter set still has to exist), and set the per-stage dt for
  // stages 0..N-1. The terminal "dt" is unused but we mirror the last value.
  for (size_t k = 0; k < time_mpt::N; ++k) {
    acados_->setParameters(static_cast<int>(k), {L, dt_schedule[k]});
  }
  acados_->setParameters(static_cast<int>(time_mpt::N), {L, dt_schedule[time_mpt::N - 1]});
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
  // Stage k targets s_k = (t_cumulative[k] / t_total) * s_horizon_in along
  // the cropped forward trajectory. Because dt_schedule packs the near-field
  // densely, t_cumulative[k] grows slowly for small k → s_k is concentrated
  // near s = 0, then opens out. Stage 0 targets s = 0 (= ego projection),
  // stage N targets s = s_horizon_in (= input end of the cropped slice).
  // v_ref_k is sampled from the input's longitudinal_velocity at that arc
  // length (clamped to the input box). With v as a free input, the solver
  // can deviate from v_ref to make the integrated path land on the terminal
  // pose — this is the extra freedom the time-axis formulation buys us.
  auto s_at_stage = [&](size_t k) { return (t_cumulative[k] / t_total) * s_horizon_in; };
  // The orientation interpolator returns a normalised quaternion, so converting
  // back to yaw gives a value in (-pi, pi]. Unwrap stage-by-stage so the cost
  // term sees a continuous psi_ref across the horizon.
  double psi_ref_prev = psi0;

  const bool publish_markers = plugin_params_.enable_debug_topics;
  visualization_msgs::msg::MarkerArray debug_markers;
  const auto stamp = node_ptr_->now();
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
    const double s_k = s_at_stage(k);
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

  // ---- 4b. Uncrossable_boundary active-set preview ----
  // Per-stage Layer 3-6 selection over the segments fed in by the planner.
  // The result is currently consumed only by the debug visualisation; the
  // acados con_h side is still pending and will pull from the same pipeline
  // once wired up. See
  // my_docs/min_rule_based_path_optimizer_uncrossable_boundary_constraints.md.
  const bool run_uncrossable = uncrossable_params_.enable && !uncrossable_segments.empty();
  if (run_uncrossable && publish_markers) {
    const int K_max = std::max(1, uncrossable_params_.K_max_per_stage);
    for (size_t k = 0; k <= time_mpt::N; ++k) {
      const double s_k = s_at_stage(k);
      const auto ref_pt = fwd_traj.compute(s_k);
      const double psi_ws = yaw_from_quat(ref_pt.pose.orientation);
      std::array<geometry_msgs::msg::Point, 3> circles;
      for (size_t c = 0; c < 3; ++c) {
        const double lo = vehicle_circles_[c].lon_offset_m;
        circles[c].x = ref_pt.pose.position.x + lo * std::cos(psi_ws);
        circles[c].y = ref_pt.pose.position.y + lo * std::sin(psi_ws);
        circles[c].z = ref_pt.pose.position.z;
      }
      const auto selected = select_uncrossable_active_set_for_stage(circles, uncrossable_segments);
      if (selected.empty()) continue;

      // Stage-coloured LINE_LIST of selected segments. Stage gradient: red at
      // stage 0 → green at terminal so we can see which constraints affect
      // the near vs far horizon.
      const float tcol = static_cast<float>(k) / static_cast<float>(time_mpt::N);
      visualization_msgs::msg::Marker seg_m;
      seg_m.header.frame_id = "map";
      seg_m.header.stamp = stamp;
      seg_m.ns = "uncrossable_selected";
      seg_m.id = static_cast<int>(k);
      seg_m.type = visualization_msgs::msg::Marker::LINE_LIST;
      seg_m.action = visualization_msgs::msg::Marker::ADD;
      seg_m.scale.x = 0.08;
      seg_m.color.r = 1.0f - tcol;
      seg_m.color.g = tcol;
      seg_m.color.b = 0.4f;
      seg_m.color.a = 0.85f;
      seg_m.pose.orientation.w = 1.0;
      for (const auto & hs : selected) {
        seg_m.points.push_back(hs.p_a);
        seg_m.points.push_back(hs.p_b);
      }
      debug_markers.markers.push_back(seg_m);

      // Inward-normal arrow per selected half-space, anchored at the segment
      // closest point. Length = margin + a fixed visual padding so the arrow
      // is always visible.
      const double arrow_len = 0.5 + uncrossable_params_.margin_m;
      for (size_t i = 0; i < selected.size(); ++i) {
        const auto & hs = selected[i];
        visualization_msgs::msg::Marker arr;
        arr.header.frame_id = "map";
        arr.header.stamp = stamp;
        arr.ns = "uncrossable_normal";
        arr.id = static_cast<int>(k) * K_max + static_cast<int>(i);
        arr.type = visualization_msgs::msg::Marker::ARROW;
        arr.action = visualization_msgs::msg::Marker::ADD;
        arr.scale.x = 0.04;  // shaft diameter
        arr.scale.y = 0.08;  // head diameter
        arr.scale.z = 0.10;  // head length
        arr.color.r = 1.0f - tcol;
        arr.color.g = tcol;
        arr.color.b = 0.4f;
        arr.color.a = 0.9f;
        arr.pose.orientation.w = 1.0;
        geometry_msgs::msg::Point tip;
        tip.x = hs.anchor.x + hs.n_x * arrow_len;
        tip.y = hs.anchor.y + hs.n_y * arrow_len;
        tip.z = hs.anchor.z;
        arr.points.push_back(hs.anchor);
        arr.points.push_back(tip);
        debug_markers.markers.push_back(arr);
      }
    }
  }

  // 3-circle vehicle footprint at the warm-start anchor (stage 0). Lets the
  // user check by eye that the radii / lon offsets line up with the actual
  // vehicle outline in rviz.
  if (publish_markers && uncrossable_params_.enable) {
    for (size_t c = 0; c < 3; ++c) {
      const double lo = vehicle_circles_[c].lon_offset_m;
      visualization_msgs::msg::Marker cyl;
      cyl.header.frame_id = "map";
      cyl.header.stamp = stamp;
      cyl.ns = "uncrossable_vehicle_circle_stage0";
      cyl.id = static_cast<int>(c);
      cyl.type = visualization_msgs::msg::Marker::CYLINDER;
      cyl.action = visualization_msgs::msg::Marker::ADD;
      cyl.pose.position.x = X0 + lo * std::cos(psi0);
      cyl.pose.position.y = Y0 + lo * std::sin(psi0);
      cyl.pose.position.z = ego_pose.position.z;
      cyl.pose.orientation.w = 1.0;
      const double diameter = 2.0 * vehicle_circles_[c].radius_m;
      cyl.scale.x = diameter;
      cyl.scale.y = diameter;
      cyl.scale.z = 0.05;
      cyl.color.r = 0.2f;
      cyl.color.g = 0.6f;
      cyl.color.b = 1.0f;
      cyl.color.a = 0.25f;
      debug_markers.markers.push_back(cyl);
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
      const double s_k = s_at_stage(k);
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
      const double s_k = s_at_stage(k);
      const double kappa = fwd_traj.curvature(s_k);
      const double d = std::atan(L * kappa);
      xtraj_init[k][3] = clamp(d, -delta_clamp, delta_clamp);
    }
    // u_delta = ddelta/dt scaled by the per-stage dt; v from input speed
    // profile clamped to the box.
    for (size_t k = 0; k < time_mpt::N; ++k) {
      const double u_phys = (xtraj_init[k + 1][3] - xtraj_init[k][3]) / dt_schedule[k];
      utraj_init[k][0] = clamp(u_phys, -u_delta_max, u_delta_max);
      const double s_k = s_at_stage(k);
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
      node_ptr_->get_logger(), "[min_rule_based_path_optimizer] solve failed (status=%d, %s).",
      solution.status, solution.info.c_str());
    try_apply_previous_output_fallback(traj_points, ego_pose);
    return;
  }

  const double elapsed_ms = solution.elapsed_time * 1000.0;
  if (elapsed_ms > plugin_params_.elapsed_time_warn_threshold_ms) {
    RCLCPP_WARN_THROTTLE(
      node_ptr_->get_logger(), *node_ptr_->get_clock(), 5000,
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
      node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000,
      "[min_rule_based_path_optimizer] could not build solution trajectory; keeping warm-start.");
    try_apply_previous_output_fallback(traj_points, ego_pose);
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
      node_ptr_->get_logger(), *node_ptr_->get_clock(), 2000,
      "[min_rule_based_path_optimizer] solution out of band "
      "(max_lat_err=%.2f m @stage=%zu, max_yaw_err=%.3f rad @stage=%zu, "
      "sqp_iter=%d, %.2f ms, s_ego=%.1f / s_total=%.1f m, T_horizon=%.2f s, v_ref=%.2f m/s). "
      "Keeping warm-start.",
      max_lat_err, k_lat, max_yaw_err, k_yaw, solution.sqp_iter, elapsed_ms, s_ego, s_total,
      t_total, v_ref_avg);
    try_apply_previous_output_fallback(traj_points, ego_pose);
  } else {
    // ---- 8. Write back: replace traj_points with the optimized region only ----
    // Output covers exactly the OCP region [s_ego, s_ego + s_sol_horizon]:
    //   - pre-ego history (upstream padding behind base_link) is dropped,
    //   - any portion past the solution horizon is also dropped.
    // For each kept index, velocity / acceleration / time_from_start are taken
    // from the original input point so only pose is overwritten by the OCP.
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
    // Persist the full output (poses from the optimizer + velocities from
    // the input it was built against) as a Trajectory so the next cycle can
    // (a) anchor warm-start on it, and (b) fall back to it on failure.
    // Out-of-band solves intentionally leave the previous good trajectory
    // in place. If pretty_build fails on the output, we drop it; the next
    // cycle will then run without anchor or fallback rather than with a
    // stale one — which matches what would happen without persistence.
    if (auto built = autoware::experimental::trajectory::pretty_build(traj_points)) {
      last_output_traj_ = std::move(built);
    } else {
      last_output_traj_.reset();
    }
  }

  if (plugin_params_.enable_debug_topics) {
    RCLCPP_DEBUG_THROTTLE(
      node_ptr_->get_logger(), *node_ptr_->get_clock(), 1000,
      "[min_rule_based_path_optimizer] solve %.2f ms, sqp_iter=%d, max_lat_err=%.2f m.", elapsed_ms,
      solution.sqp_iter, max_lat_err);
  }

  // ---- 9. Save warm-start state ----
  last_solution_ = solution;
  has_last_solution_ = true;
  last_x0_world_x_ = X0;
  last_x0_world_y_ = Y0;
  last_x0_world_yaw_ = psi0;
  last_dt_runtime_ = t_total / static_cast<double>(time_mpt::N);  // average dt, log only
}

void PathOptimizer::set_up_params()
{
  auto * node_ptr = node_ptr_;
  using autoware_utils_rclcpp::get_or_declare_parameter;

  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*node_ptr).getVehicleInfo();

  plugin_params_.dt_near_s =
    get_or_declare_parameter<double>(*node_ptr, "min_rule_based_path_optimizer.horizon.dt_near_s");
  plugin_params_.dt_far_s =
    get_or_declare_parameter<double>(*node_ptr, "min_rule_based_path_optimizer.horizon.dt_far_s");
  plugin_params_.near_stage_count = static_cast<int>(get_or_declare_parameter<int64_t>(
    *node_ptr, "min_rule_based_path_optimizer.horizon.near_stage_count"));
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

  uncrossable_params_.enable = get_or_declare_parameter<bool>(
    *node_ptr, "min_rule_based_path_optimizer.uncrossable_boundary.enable");
  uncrossable_params_.clearance_drop_threshold_m = get_or_declare_parameter<double>(
    *node_ptr, "min_rule_based_path_optimizer.uncrossable_boundary.clearance_drop_threshold_m");
  uncrossable_params_.K_max_per_stage = static_cast<int>(get_or_declare_parameter<int64_t>(
    *node_ptr, "min_rule_based_path_optimizer.uncrossable_boundary.K_max_per_stage"));
  uncrossable_params_.margin_m = get_or_declare_parameter<double>(
    *node_ptr, "min_rule_based_path_optimizer.uncrossable_boundary.margin_m");

  // 3-circle vehicle footprint: tile [−rear_overhang, length−rear_overhang] in
  // body x with three equal segments of length L/3, place each circle at the
  // segment centre, and pick the smallest radius that covers the corner of its
  // segment-by-half-width rectangle. base_link is at the rear axle in our
  // bicycle model, so lon_offset_m here is along the body x from base_link.
  {
    const double L = vehicle_info_.vehicle_length_m;
    const double W = vehicle_info_.vehicle_width_m;
    const double rear_overhang = vehicle_info_.rear_overhang_m;
    const double seg = std::max(0.1, L / 3.0);
    const double radius = std::hypot(0.5 * seg, 0.5 * W);
    vehicle_circles_[0] = {-rear_overhang + 0.5 * seg, radius};  // rear
    vehicle_circles_[1] = {-rear_overhang + 1.5 * seg, radius};  // mid
    vehicle_circles_[2] = {-rear_overhang + 2.5 * seg, radius};  // front
  }

  acados_ = std::make_unique<time_mpt::AcadosInterfaceTime>();

  pub_debug_markers_ = node_ptr->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/debug/min_rule_based_path_optimizer/markers", rclcpp::QoS{1});

  RCLCPP_INFO(
    node_ptr->get_logger(),
    "[min_rule_based_path_optimizer] initialized. L=%.3f m, delta_max=%.3f rad, "
    "u_delta_max=%.3f rad/s, v=[%.2f, %.2f] m/s, N=%zu, dt schedule = "
    "%d * %.3f s (near) + %d * %.3f s (far) = %.2f s total.",
    vehicle_info_.wheel_base_m, vehicle_info_.max_steer_angle_rad,
    plugin_params_.u_delta_max_rad_per_s, plugin_params_.v_min_mps, plugin_params_.v_max_mps,
    time_mpt::N, plugin_params_.near_stage_count, plugin_params_.dt_near_s,
    static_cast<int>(time_mpt::N) - plugin_params_.near_stage_count, plugin_params_.dt_far_s,
    plugin_params_.near_stage_count * plugin_params_.dt_near_s +
      (static_cast<int>(time_mpt::N) - plugin_params_.near_stage_count) * plugin_params_.dt_far_s);
}

rcl_interfaces::msg::SetParametersResult PathOptimizer::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.horizon.dt_near_s", plugin_params_.dt_near_s);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.horizon.dt_far_s", plugin_params_.dt_far_s);
  {
    int64_t near_stage_count_64 = plugin_params_.near_stage_count;
    if (update_param<int64_t>(
          parameters, "min_rule_based_path_optimizer.horizon.near_stage_count",
          near_stage_count_64)) {
      plugin_params_.near_stage_count = static_cast<int>(near_stage_count_64);
    }
  }
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

  update_param<bool>(
    parameters, "min_rule_based_path_optimizer.uncrossable_boundary.enable",
    uncrossable_params_.enable);
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.uncrossable_boundary.clearance_drop_threshold_m",
    uncrossable_params_.clearance_drop_threshold_m);
  {
    int64_t k_max_64 = uncrossable_params_.K_max_per_stage;
    if (update_param<int64_t>(
          parameters, "min_rule_based_path_optimizer.uncrossable_boundary.K_max_per_stage",
          k_max_64)) {
      uncrossable_params_.K_max_per_stage = static_cast<int>(k_max_64);
    }
  }
  update_param<double>(
    parameters, "min_rule_based_path_optimizer.uncrossable_boundary.margin_m",
    uncrossable_params_.margin_m);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::minimum_rule_based_planner
