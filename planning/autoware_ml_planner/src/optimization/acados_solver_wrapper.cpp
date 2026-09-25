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
  const std::array<StageReference, opt_horizon> & references, const SolverSolution * warm_start)
{
  auto x0 = initial_state;

  // Initial state equality constraint.
  ocp_nlp_constraints_model_set(
    impl_->config, impl_->dims, impl_->in, impl_->out, 0, "lbx", x0.data());
  ocp_nlp_constraints_model_set(
    impl_->config, impl_->dims, impl_->in, impl_->out, 0, "ubx", x0.data());

  // Cost weights. The 2x2 position block is rotated to the reference heading of each
  // stage so longitudinal and lateral tracking errors are weighted separately:
  //   W_pos = R(yaw_ref) * diag(w_lon, w_lat) * R(yaw_ref)^T.
  // acados scales stage costs by dt; multiply by 1/dt (= N/Tf) so the configured weights
  // keep a per-sample magnitude (same convention as generate_solver.py).
  // y = [x, y, yaw, v, delta, a, delta_rate]; v and delta have no reference (see header).
  const double unscale = 1.0 / opt_dt_s;
  const double w_lon = impl_->params.weight_longitudinal;
  const double w_lat = impl_->params.weight_lateral;
  auto position_block = [&](const double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return std::array<double, 3>{
      w_lon * c * c + w_lat * s * s,  // xx
      w_lon * s * s + w_lat * c * c,  // yy
      (w_lon - w_lat) * c * s};       // xy = yx
  };
  std::array<double, gen_ny * gen_ny> stage_weight_matrix{};
  stage_weight_matrix[2 * gen_ny + 2] = unscale * impl_->params.weight_yaw;
  stage_weight_matrix[5 * gen_ny + 5] = unscale * impl_->params.weight_acceleration;
  stage_weight_matrix[6 * gen_ny + 6] = unscale * impl_->params.weight_steering_rate;
  for (size_t stage = 0; stage < gen_n; ++stage) {
    const double yaw_ref = (stage == 0) ? x0[2] : references[stage - 1].yaw;
    const auto [w_xx, w_yy, w_xy] = position_block(yaw_ref);
    stage_weight_matrix[0] = unscale * w_xx;
    stage_weight_matrix[gen_ny + 1] = unscale * w_yy;
    stage_weight_matrix[1] = unscale * w_xy;
    stage_weight_matrix[gen_ny] = unscale * w_xy;
    ocp_nlp_cost_model_set(
      impl_->config, impl_->dims, impl_->in, static_cast<int>(stage), "W",
      stage_weight_matrix.data());
  }
  const double terminal_scale = impl_->params.terminal_weight_scale / unscale;
  std::array<double, gen_nyn * gen_nyn> terminal_weight_matrix{};
  const auto [we_xx, we_yy, we_xy] = position_block(references[gen_n - 1].yaw);
  terminal_weight_matrix[0] = terminal_scale * we_xx;
  terminal_weight_matrix[gen_nyn + 1] = terminal_scale * we_yy;
  terminal_weight_matrix[1] = terminal_scale * we_xy;
  terminal_weight_matrix[gen_nyn] = terminal_scale * we_xy;
  terminal_weight_matrix[2 * gen_nyn + 2] = terminal_scale * impl_->params.weight_yaw;
  ocp_nlp_cost_model_set(
    impl_->config, impl_->dims, impl_->in, static_cast<int>(gen_n), "W",
    terminal_weight_matrix.data());

  // References: stage 0 tracks the (fixed) initial state so it contributes no cost,
  // stages 1..N-1 track the input trajectory, stage N is the terminal reference.
  std::array<double, gen_ny> yref{};
  std::copy(x0.begin(), x0.end(), yref.begin());
  ocp_nlp_cost_model_set(impl_->config, impl_->dims, impl_->in, 0, "yref", yref.data());
  for (size_t stage = 1; stage < gen_n; ++stage) {
    const auto & ref = references[stage - 1];
    yref = {ref.x, ref.y, ref.yaw, 0.0, 0.0, 0.0, 0.0};
    ocp_nlp_cost_model_set(
      impl_->config, impl_->dims, impl_->in, static_cast<int>(stage), "yref", yref.data());
  }
  const auto & terminal_ref = references[gen_n - 1];
  std::array<double, gen_nyn> yref_e{terminal_ref.x, terminal_ref.y, terminal_ref.yaw, 0.0, 0.0};
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
