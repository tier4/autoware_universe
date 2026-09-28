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

#include "autoware/ml_planner/optimization/acados_solver_wrapper.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

extern "C" {
#include "c_generated_code/acados_solver_ml_planner_optimizer.h"
}

namespace autoware::ml_planner::optimization
{
namespace
{
constexpr size_t gen_nx = ML_PLANNER_OPTIMIZER_NX;
constexpr size_t gen_nu = ML_PLANNER_OPTIMIZER_NU;
constexpr size_t gen_np = ML_PLANNER_OPTIMIZER_NP;
constexpr size_t gen_n = ML_PLANNER_OPTIMIZER_N;
constexpr size_t gen_ny = ML_PLANNER_OPTIMIZER_NY;
constexpr size_t gen_nyn = ML_PLANNER_OPTIMIZER_NYN;

static_assert(gen_nx == opt_nx, "generated solver NX mismatch, re-run generate_solver.py");
static_assert(gen_nu == opt_nu, "generated solver NU mismatch, re-run generate_solver.py");
static_assert(gen_np == 1, "generated solver NP mismatch, re-run generate_solver.py");
static_assert(gen_n == opt_horizon, "generated solver N mismatch, re-run generate_solver.py");
static_assert(gen_ny == opt_nx + opt_nu, "generated solver NY mismatch");
static_assert(gen_nyn == opt_nx, "generated solver NYN mismatch");

/// Symmetric 2x2 block [xx, yy, xy] of R(yaw) * diag(w_lon, w_lat) * R(yaw)^T.
std::array<double, 3> position_block(
  const double yaw, const double longitudinal_weight, const double lateral_weight)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return std::array<double, 3>{
    longitudinal_weight * c * c + lateral_weight * s * s,  // xx
    longitudinal_weight * s * s + lateral_weight * c * c,  // yy
    (longitudinal_weight - lateral_weight) * c * s};       // xy = yx
}

/**
 * @brief Collapse two weighted quadratic penalties on the same variable into one.
 *
 * A least-squares cost can only carry a single reference per output, but tracking the model
 * output and tracking the previous plan are two penalties on the same position. They are
 * exactly equivalent to one penalty with the summed weight and a blended reference,
 *
 *   |y - r1|^2_W1 + |y - r2|^2_W2 = |y - r*|^2_(W1+W2) + const,
 *   r* = (W1 + W2)^-1 (W1 r1 + W2 r2),
 *
 * and the dropped constant does not move the minimizer. So the temporal consistency term
 * needs no extra cost outputs and no regenerated solver - it only changes W and yref.
 *
 * This is the 2x2 position case: `block` is [xx, yy, xy] of the summed weight.
 */
std::array<double, 2> blend_position_reference(
  const std::array<double, 3> & first_block, const std::array<double, 2> & first_reference,
  const std::array<double, 3> & second_block, const std::array<double, 2> & second_reference,
  const std::array<double, 3> & block)
{
  const auto weighted = [](const std::array<double, 3> & w, const std::array<double, 2> & r) {
    return std::array<double, 2>{w[0] * r[0] + w[2] * r[1], w[2] * r[0] + w[1] * r[1]};
  };
  const auto lhs = weighted(first_block, first_reference);
  const auto rhs = weighted(second_block, second_reference);
  const std::array<double, 2> rhs_sum{lhs[0] + rhs[0], lhs[1] + rhs[1]};
  const double determinant = block[0] * block[1] - block[2] * block[2];
  if (!(std::abs(determinant) > 1.0e-12)) {
    // Both penalties are (near) zero-weight, so any reference minimizes the cost equally.
    return first_reference;
  }
  return std::array<double, 2>{
    (block[1] * rhs_sum[0] - block[2] * rhs_sum[1]) / determinant,
    (block[0] * rhs_sum[1] - block[2] * rhs_sum[0]) / determinant};
}

/// Scalar case of blend_position_reference().
double blend_reference(
  const double first_weight, const double first_reference, const double second_weight,
  const double second_reference)
{
  const double total = first_weight + second_weight;
  if (!(total > 0.0)) {
    return first_reference;
  }
  return (first_weight * first_reference + second_weight * second_reference) / total;
}
}  // namespace

struct AcadosSolverWrapper::Impl
{
  ml_planner_optimizer_solver_capsule * capsule{nullptr};
  ocp_nlp_config * config{nullptr};
  ocp_nlp_dims * dims{nullptr};
  ocp_nlp_in * in{nullptr};
  ocp_nlp_out * out{nullptr};
  ocp_nlp_solver * solver{nullptr};
  void * opts{nullptr};
  TrajectoryOptimizationParams params{};
};

AcadosSolverWrapper::AcadosSolverWrapper(
  const TrajectoryOptimizationParams & params, const double wheelbase_m,
  const double max_steering_angle_rad)
: impl_(std::make_unique<Impl>())
{
  impl_->capsule = ml_planner_optimizer_acados_create_capsule();
  if (ml_planner_optimizer_acados_create(impl_->capsule) != 0) {
    ml_planner_optimizer_acados_free_capsule(impl_->capsule);
    impl_->capsule = nullptr;
    throw std::runtime_error("failed to create acados solver");
  }
  impl_->config = ml_planner_optimizer_acados_get_nlp_config(impl_->capsule);
  impl_->dims = ml_planner_optimizer_acados_get_nlp_dims(impl_->capsule);
  impl_->in = ml_planner_optimizer_acados_get_nlp_in(impl_->capsule);
  impl_->out = ml_planner_optimizer_acados_get_nlp_out(impl_->capsule);
  impl_->solver = ml_planner_optimizer_acados_get_nlp_solver(impl_->capsule);
  impl_->opts = ml_planner_optimizer_acados_get_nlp_opts(impl_->capsule);

  // Cost weights depend on the per-stage reference heading (longitudinal/lateral split),
  // so they are written in solve(). Only the reference-independent settings are set here.
  impl_->params = params;

  // Input bounds (all stages).
  std::array<double, gen_nu> lbu{params.min_acceleration_mps2, -params.max_steering_rate_rps};
  std::array<double, gen_nu> ubu{params.max_acceleration_mps2, params.max_steering_rate_rps};
  for (size_t stage = 0; stage < gen_n; ++stage) {
    ocp_nlp_constraints_model_set(
      impl_->config, impl_->dims, impl_->in, impl_->out, static_cast<int>(stage), "lbu",
      lbu.data());
    ocp_nlp_constraints_model_set(
      impl_->config, impl_->dims, impl_->in, impl_->out, static_cast<int>(stage), "ubu",
      ubu.data());
  }

  // State bounds on v and delta (stages 1..N; stage 0 is the initial state equality).
  std::array<double, 2> lbx{params.min_velocity_mps, -max_steering_angle_rad};
  std::array<double, 2> ubx{params.max_velocity_mps, max_steering_angle_rad};
  for (size_t stage = 1; stage <= gen_n; ++stage) {
    ocp_nlp_constraints_model_set(
      impl_->config, impl_->dims, impl_->in, impl_->out, static_cast<int>(stage), "lbx",
      lbx.data());
    ocp_nlp_constraints_model_set(
      impl_->config, impl_->dims, impl_->in, impl_->out, static_cast<int>(stage), "ubx",
      ubx.data());
  }

  // Soft lateral acceleration bounds (stages 0..N-1).
  std::array<double, 1> lh{-params.max_lateral_acceleration_mps2};
  std::array<double, 1> uh{params.max_lateral_acceleration_mps2};
  for (size_t stage = 0; stage < gen_n; ++stage) {
    ocp_nlp_constraints_model_set(
      impl_->config, impl_->dims, impl_->in, impl_->out, static_cast<int>(stage), "lh", lh.data());
    ocp_nlp_constraints_model_set(
      impl_->config, impl_->dims, impl_->in, impl_->out, static_cast<int>(stage), "uh", uh.data());
  }

  // Vehicle parameter (all stages).
  std::array<double, gen_np> p{wheelbase_m};
  for (size_t stage = 0; stage <= gen_n; ++stage) {
    ml_planner_optimizer_acados_update_params(
      impl_->capsule, static_cast<int>(stage), p.data(), static_cast<int>(gen_np));
  }

  int max_iter = std::max(params.max_sqp_iterations, 1);
  ocp_nlp_solver_opts_set(impl_->config, impl_->opts, "max_iter", &max_iter);
}

AcadosSolverWrapper::~AcadosSolverWrapper()
{
  if (impl_ && impl_->capsule) {
    ml_planner_optimizer_acados_free(impl_->capsule);
    ml_planner_optimizer_acados_free_capsule(impl_->capsule);
  }
}

SolverSolution AcadosSolverWrapper::solve(
  const std::array<double, opt_nx> & initial_state,
  const std::array<StageReference, opt_horizon> & references,
  const std::optional<GoalTerminalReference> & goal_terminal_reference,
  const std::array<StageTemporalReference, opt_horizon> * temporal_references,
  const SolverSolution * warm_start)
{
  auto x0 = initial_state;

  // Initial state equality constraint.
  ocp_nlp_constraints_model_set(
    impl_->config, impl_->dims, impl_->in, impl_->out, 0, "lbx", x0.data());
  ocp_nlp_constraints_model_set(
    impl_->config, impl_->dims, impl_->in, impl_->out, 0, "ubx", x0.data());

  // Cost weights and references. The 2x2 position block is rotated to the reference heading
  // of each stage so longitudinal and lateral tracking errors are weighted separately:
  //   W_pos = R(yaw_ref) * diag(w_lon, w_lat) * R(yaw_ref)^T.
  // acados scales stage costs by dt; multiply by 1/dt (= N/Tf) so the configured weights
  // keep a per-sample magnitude (same convention as generate_solver.py).
  // y = [x, y, yaw, v, delta, a, delta_rate]. The v and delta references remain zero unless
  // another term supplies one, so their weights penalize state magnitude directly.
  //
  // Temporal consistency adds a second penalty on position, yaw and velocity whose
  // reference is the previous cycle's plan. Weights and references are written together
  // because the two penalties are folded into the single least-squares term the solver was
  // generated with (see blend_position_reference).
  const double unscale = 1.0 / opt_dt_s;
  const double w_lon = impl_->params.weight_longitudinal;
  const double w_lat = impl_->params.weight_lateral;
  const auto & temporal = impl_->params.temporal_consistency;
  const bool use_temporal = temporal.enable && temporal_references != nullptr;
  // Weight profile over the horizon: hold the near stages, which the controller executes,
  // and let the far stages - a prediction that is supposed to move - go free.
  const double decay_ratio = std::clamp(temporal.far_weight_ratio, 0.0, 1.0);
  const auto temporal_scale = [&temporal, decay_ratio](const size_t stage) {
    if (!(temporal.decay_time_constant_s > 0.0)) {
      return 1.0;
    }
    const double time_s = opt_dt_s * static_cast<double>(stage);
    return decay_ratio + (1.0 - decay_ratio) * std::exp(-time_s / temporal.decay_time_constant_s);
  };

  std::array<double, gen_ny * gen_ny> stage_weight_matrix{};
  stage_weight_matrix[4 * gen_ny + 4] = unscale * impl_->params.weight_steering_angle;
  stage_weight_matrix[5 * gen_ny + 5] = unscale * impl_->params.weight_acceleration;
  stage_weight_matrix[6 * gen_ny + 6] = unscale * impl_->params.weight_steering_rate;
  std::array<double, gen_ny> yref{};
  for (size_t stage = 0; stage < gen_n; ++stage) {
    // Stage 0 tracks the (fixed) initial state, so its state penalties contribute no cost
    // and only the input regularization is left; it gets no temporal reference either.
    const bool is_initial_stage = stage == 0;
    const auto & ref = references[is_initial_stage ? 0 : stage - 1];
    const double yaw_ref = is_initial_stage ? x0[2] : ref.yaw;
    const std::array<double, 2> position_ref =
      is_initial_stage ? std::array<double, 2>{x0[0], x0[1]} : std::array<double, 2>{ref.x, ref.y};

    const auto track_block = position_block(yaw_ref, w_lon, w_lat);
    auto block = track_block;
    auto blended_position = position_ref;
    double yaw_weight = impl_->params.weight_yaw;
    double velocity_weight = impl_->params.weight_velocity;
    double blended_yaw = yaw_ref;
    double blended_velocity = 0.0;

    if (use_temporal && !is_initial_stage) {
      const auto & previous = (*temporal_references)[stage - 1];
      const double scale = temporal_scale(stage);
      const double t_lon = scale * temporal.weight_longitudinal;
      const double t_lat = scale * temporal.weight_lateral;
      const double t_yaw = scale * temporal.weight_yaw;
      const double t_velocity = scale * temporal.weight_velocity;
      const auto temporal_block = position_block(previous.yaw, t_lon, t_lat);
      block = {
        track_block[0] + temporal_block[0], track_block[1] + temporal_block[1],
        track_block[2] + temporal_block[2]};
      blended_position = blend_position_reference(
        track_block, position_ref, temporal_block, {previous.x, previous.y}, block);
      blended_yaw = blend_reference(yaw_weight, yaw_ref, t_yaw, previous.yaw);
      yaw_weight += t_yaw;
      // The tracking term references velocity to zero, i.e. it penalizes magnitude; the
      // blend therefore pulls toward a fraction of the previous velocity.
      blended_velocity = blend_reference(velocity_weight, 0.0, t_velocity, previous.velocity);
      velocity_weight += t_velocity;
    }

    stage_weight_matrix[0] = unscale * block[0];
    stage_weight_matrix[gen_ny + 1] = unscale * block[1];
    stage_weight_matrix[1] = unscale * block[2];
    stage_weight_matrix[gen_ny] = unscale * block[2];
    stage_weight_matrix[2 * gen_ny + 2] = unscale * yaw_weight;
    stage_weight_matrix[3 * gen_ny + 3] = unscale * velocity_weight;
    ocp_nlp_cost_model_set(
      impl_->config, impl_->dims, impl_->in, static_cast<int>(stage), "W",
      stage_weight_matrix.data());

    yref = {blended_position[0], blended_position[1], blended_yaw, blended_velocity, 0.0, 0.0, 0.0};
    ocp_nlp_cost_model_set(
      impl_->config, impl_->dims, impl_->in, static_cast<int>(stage), "yref", yref.data());
  }

  // Terminal stage. The goal penalty, when latched, is a third quadratic on the same
  // outputs and folds in the same way.
  const double terminal_scale = impl_->params.terminal_weight_scale / unscale;
  const auto & terminal_ref = references[gen_n - 1];
  std::array<double, 2> terminal_position{terminal_ref.x, terminal_ref.y};
  double terminal_yaw = terminal_ref.yaw;
  double terminal_velocity = 0.0;
  if (goal_terminal_reference) {
    terminal_position = {goal_terminal_reference->x, goal_terminal_reference->y};
    terminal_yaw = goal_terminal_reference->yaw;
    terminal_velocity = goal_terminal_reference->velocity;
  }
  auto terminal_block = position_block(terminal_yaw, w_lon, w_lat);
  for (auto & entry : terminal_block) {
    entry *= terminal_scale;
  }
  double terminal_yaw_weight = terminal_scale * impl_->params.weight_yaw;
  double terminal_velocity_weight = terminal_scale * impl_->params.weight_velocity;
  if (goal_terminal_reference) {
    const auto goal_block = position_block(
      goal_terminal_reference->yaw, impl_->params.goal.weight_longitudinal,
      impl_->params.goal.weight_lateral);
    // Same reference for both penalties here, so summing the weights is all that is needed.
    for (size_t i = 0; i < terminal_block.size(); ++i) {
      terminal_block[i] += goal_block[i];
    }
    terminal_yaw_weight += impl_->params.goal.weight_yaw;
    terminal_velocity_weight += impl_->params.goal.weight_velocity;
  }
  if (use_temporal) {
    const auto & previous = (*temporal_references)[gen_n - 1];
    const double scale = temporal_scale(gen_n);
    const double t_lon = scale * temporal.weight_longitudinal;
    const double t_lat = scale * temporal.weight_lateral;
    const double t_yaw = scale * temporal.weight_yaw;
    const double t_velocity = scale * temporal.weight_velocity;
    const auto temporal_block = position_block(previous.yaw, t_lon, t_lat);
    auto summed_block = terminal_block;
    for (size_t i = 0; i < summed_block.size(); ++i) {
      summed_block[i] += temporal_block[i];
    }
    terminal_position = blend_position_reference(
      terminal_block, terminal_position, temporal_block, {previous.x, previous.y}, summed_block);
    terminal_block = summed_block;
    terminal_yaw = blend_reference(terminal_yaw_weight, terminal_yaw, t_yaw, previous.yaw);
    terminal_yaw_weight += t_yaw;
    terminal_velocity =
      blend_reference(terminal_velocity_weight, terminal_velocity, t_velocity, previous.velocity);
    terminal_velocity_weight += t_velocity;
  }
  std::array<double, gen_nyn * gen_nyn> terminal_weight_matrix{};
  terminal_weight_matrix[0] = terminal_block[0];
  terminal_weight_matrix[gen_nyn + 1] = terminal_block[1];
  terminal_weight_matrix[1] = terminal_block[2];
  terminal_weight_matrix[gen_nyn] = terminal_block[2];
  terminal_weight_matrix[2 * gen_nyn + 2] = terminal_yaw_weight;
  terminal_weight_matrix[3 * gen_nyn + 3] = terminal_velocity_weight;
  terminal_weight_matrix[4 * gen_nyn + 4] = terminal_scale * impl_->params.weight_steering_angle;
  ocp_nlp_cost_model_set(
    impl_->config, impl_->dims, impl_->in, static_cast<int>(gen_n), "W",
    terminal_weight_matrix.data());

  std::array<double, gen_nyn> yref_e{
    terminal_position[0], terminal_position[1], terminal_yaw, terminal_velocity, 0.0};
  ocp_nlp_cost_model_set(
    impl_->config, impl_->dims, impl_->in, static_cast<int>(gen_n), "yref", yref_e.data());

  // Initial guess: previous solution shifted by one stage, or the initial state.
  for (size_t stage = 0; stage <= gen_n; ++stage) {
    std::array<double, gen_nx> x_guess = x0;
    if (warm_start != nullptr) {
      x_guess = warm_start->states[std::min(stage + 1, gen_n)];
    }
    ocp_nlp_out_set(
      impl_->config, impl_->dims, impl_->out, impl_->in, static_cast<int>(stage), "x",
      x_guess.data());
    if (stage < gen_n) {
      std::array<double, gen_nu> u_guess{};
      if (warm_start != nullptr) {
        u_guess = warm_start->inputs[std::min(stage + 1, gen_n - 1)];
      }
      ocp_nlp_out_set(
        impl_->config, impl_->dims, impl_->out, impl_->in, static_cast<int>(stage), "u",
        u_guess.data());
    }
  }

  SolverSolution solution;
  solution.status = ml_planner_optimizer_acados_solve(impl_->capsule);
  ocp_nlp_get(impl_->solver, "time_tot", &solution.solve_time_s);
  ocp_nlp_get(impl_->solver, "sqp_iter", &solution.sqp_iterations);

  for (size_t stage = 0; stage <= gen_n; ++stage) {
    ocp_nlp_out_get(
      impl_->config, impl_->dims, impl_->out, static_cast<int>(stage), "x",
      solution.states[stage].data());
    if (stage < gen_n) {
      ocp_nlp_out_get(
        impl_->config, impl_->dims, impl_->out, static_cast<int>(stage), "u",
        solution.inputs[stage].data());
    }
  }

  return solution;
}

}  // namespace autoware::ml_planner::optimization
