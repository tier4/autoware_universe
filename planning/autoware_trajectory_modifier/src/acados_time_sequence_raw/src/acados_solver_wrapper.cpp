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

#include "autoware/trajectory_modifier/time_sequence_raw/acados_solver_wrapper.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>

extern "C" {
#include "c_generated_code/acados_solver_kinematic_bicycle_time_seq.h"
}

namespace autoware::trajectory_modifier::time_sequence_raw
{
namespace
{
constexpr size_t gen_nx = KINEMATIC_BICYCLE_TIME_SEQ_NX;
constexpr size_t gen_nu = KINEMATIC_BICYCLE_TIME_SEQ_NU;
constexpr size_t gen_np = KINEMATIC_BICYCLE_TIME_SEQ_NP;
constexpr size_t gen_n = KINEMATIC_BICYCLE_TIME_SEQ_N;
constexpr size_t gen_ny = KINEMATIC_BICYCLE_TIME_SEQ_NY;
constexpr size_t gen_nyn = KINEMATIC_BICYCLE_TIME_SEQ_NYN;

static_assert(gen_nx == opt_nx, "generated solver NX mismatch, re-run generate_solver.py");
static_assert(gen_nu == opt_nu, "generated solver NU mismatch, re-run generate_solver.py");
static_assert(gen_np == 1, "generated solver NP mismatch, re-run generate_solver.py");
static_assert(gen_n == opt_horizon, "generated solver N mismatch, re-run generate_solver.py");
static_assert(gen_ny == opt_nx + opt_nu, "generated solver NY mismatch");
static_assert(gen_nyn == opt_nx, "generated solver NYN mismatch");

size_t w_index(const size_t row, const size_t col, const size_t ny)
{
  return row * ny + col;
}

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
    return first_reference;
  }
  return std::array<double, 2>{
    (block[1] * rhs_sum[0] - block[2] * rhs_sum[1]) / determinant,
    (block[0] * rhs_sum[1] - block[2] * rhs_sum[0]) / determinant};
}

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
  kinematic_bicycle_time_seq_solver_capsule * capsule{nullptr};
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
  impl_->capsule = kinematic_bicycle_time_seq_acados_create_capsule();
  if (kinematic_bicycle_time_seq_acados_create(impl_->capsule) != 0) {
    kinematic_bicycle_time_seq_acados_free_capsule(impl_->capsule);
    impl_->capsule = nullptr;
    throw std::runtime_error("failed to create time-sequence acados solver");
  }
  impl_->config = kinematic_bicycle_time_seq_acados_get_nlp_config(impl_->capsule);
  impl_->dims = kinematic_bicycle_time_seq_acados_get_nlp_dims(impl_->capsule);
  impl_->in = kinematic_bicycle_time_seq_acados_get_nlp_in(impl_->capsule);
  impl_->out = kinematic_bicycle_time_seq_acados_get_nlp_out(impl_->capsule);
  impl_->solver = kinematic_bicycle_time_seq_acados_get_nlp_solver(impl_->capsule);
  impl_->opts = kinematic_bicycle_time_seq_acados_get_nlp_opts(impl_->capsule);

  impl_->params = params;

  std::array<double, gen_nu> lbu{params.min_jerk_mps3, -params.max_steering_rate_rps};
  std::array<double, gen_nu> ubu{params.max_jerk_mps3, params.max_steering_rate_rps};
  for (size_t stage = 0; stage < gen_n; ++stage) {
    ocp_nlp_constraints_model_set(
      impl_->config, impl_->dims, impl_->in, impl_->out, static_cast<int>(stage), "lbu",
      lbu.data());
    ocp_nlp_constraints_model_set(
      impl_->config, impl_->dims, impl_->in, impl_->out, static_cast<int>(stage), "ubu",
      ubu.data());
  }

  std::array<double, 3> lbx{
    params.min_velocity_mps, -max_steering_angle_rad, params.min_acceleration_mps2};
  std::array<double, 3> ubx{
    params.max_velocity_mps, max_steering_angle_rad, params.max_acceleration_mps2};
  for (size_t stage = 1; stage <= gen_n; ++stage) {
    ocp_nlp_constraints_model_set(
      impl_->config, impl_->dims, impl_->in, impl_->out, static_cast<int>(stage), "lbx",
      lbx.data());
    ocp_nlp_constraints_model_set(
      impl_->config, impl_->dims, impl_->in, impl_->out, static_cast<int>(stage), "ubx",
      ubx.data());
  }

  std::array<double, 1> lh{-params.max_lateral_acceleration_mps2};
  std::array<double, 1> uh{params.max_lateral_acceleration_mps2};
  for (size_t stage = 0; stage < gen_n; ++stage) {
    ocp_nlp_constraints_model_set(
      impl_->config, impl_->dims, impl_->in, impl_->out, static_cast<int>(stage), "lh", lh.data());
    ocp_nlp_constraints_model_set(
      impl_->config, impl_->dims, impl_->in, impl_->out, static_cast<int>(stage), "uh", uh.data());
  }

  std::array<double, gen_np> p{wheelbase_m};
  for (size_t stage = 0; stage <= gen_n; ++stage) {
    kinematic_bicycle_time_seq_acados_update_params(
      impl_->capsule, static_cast<int>(stage), p.data(), static_cast<int>(gen_np));
  }

  int max_iter = std::max(params.max_sqp_iterations, 1);
  ocp_nlp_solver_opts_set(impl_->config, impl_->opts, "max_iter", &max_iter);
}

AcadosSolverWrapper::~AcadosSolverWrapper()
{
  if (impl_ && impl_->capsule) {
    kinematic_bicycle_time_seq_acados_free(impl_->capsule);
    kinematic_bicycle_time_seq_acados_free_capsule(impl_->capsule);
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

  ocp_nlp_constraints_model_set(
    impl_->config, impl_->dims, impl_->in, impl_->out, 0, "lbx", x0.data());
  ocp_nlp_constraints_model_set(
    impl_->config, impl_->dims, impl_->in, impl_->out, 0, "ubx", x0.data());

  const double unscale = 1.0 / opt_dt_s;
  const double w_lon = impl_->params.weight_longitudinal;
  const double w_lat = impl_->params.weight_lateral;
  const auto & temporal = impl_->params.temporal_consistency;
  const bool use_temporal = temporal.enable && temporal_references != nullptr;
  const double decay_ratio = std::clamp(temporal.far_weight_ratio, 0.0, 1.0);
  const auto temporal_scale = [&temporal, decay_ratio](const size_t stage) {
    if (!(temporal.decay_time_constant_s > 0.0)) {
      return 1.0;
    }
    const double time_s = opt_dt_s * static_cast<double>(stage);
    return decay_ratio + (1.0 - decay_ratio) * std::exp(-time_s / temporal.decay_time_constant_s);
  };

  std::array<double, gen_ny * gen_ny> stage_weight_matrix{};
  stage_weight_matrix[w_index(kYJerk, kYJerk, gen_ny)] = unscale * impl_->params.weight_jerk;
  stage_weight_matrix[w_index(kYDeltaRate, kYDeltaRate, gen_ny)] =
    unscale * impl_->params.weight_steering_rate;
  std::array<double, gen_ny> yref{};
  for (size_t stage = 0; stage < gen_n; ++stage) {
    const bool is_initial_stage = stage == 0;
    const auto & ref = references[is_initial_stage ? 0 : stage - 1];
    const double yaw_ref = is_initial_stage ? x0[kPsi] : ref.yaw;
    const std::array<double, 2> position_ref = is_initial_stage
                                                 ? std::array<double, 2>{x0[kX], x0[kY]}
                                                 : std::array<double, 2>{ref.x, ref.y};

    const auto track_block = position_block(yaw_ref, w_lon, w_lat);
    auto block = track_block;
    auto blended_position = position_ref;
    double yaw_weight = impl_->params.weight_yaw;
    double velocity_weight = 0.0;
    double blended_yaw = yaw_ref;
    double blended_velocity = 0.0;

    if (use_temporal && !is_initial_stage && (*temporal_references)[stage - 1].valid) {
      const auto & previous = (*temporal_references)[stage - 1];
      const double scale = temporal_scale(stage);
      const double t_lon = scale * temporal.weight_longitudinal;
      const double t_lat = scale * temporal.weight_lateral;
      const double t_yaw = scale * temporal.weight_yaw;
      const double t_velocity = scale * temporal.weight_velocity;
      const auto temporal_block = position_block(yaw_ref, t_lon, t_lat);
      block = {
        track_block[0] + temporal_block[0], track_block[1] + temporal_block[1],
        track_block[2] + temporal_block[2]};
      blended_position = blend_position_reference(
        track_block, position_ref, temporal_block, {previous.x, previous.y}, block);
      blended_yaw = blend_reference(yaw_weight, yaw_ref, t_yaw, previous.yaw);
      yaw_weight += t_yaw;
      blended_velocity = blend_reference(velocity_weight, 0.0, t_velocity, previous.velocity);
      velocity_weight += t_velocity;
    }

    stage_weight_matrix[w_index(kX, kX, gen_ny)] = unscale * block[0];
    stage_weight_matrix[w_index(kY, kY, gen_ny)] = unscale * block[1];
    stage_weight_matrix[w_index(kX, kY, gen_ny)] = unscale * block[2];
    stage_weight_matrix[w_index(kY, kX, gen_ny)] = unscale * block[2];
    stage_weight_matrix[w_index(kPsi, kPsi, gen_ny)] = unscale * yaw_weight;
    stage_weight_matrix[w_index(kV, kV, gen_ny)] = unscale * velocity_weight;
    ocp_nlp_cost_model_set(
      impl_->config, impl_->dims, impl_->in, static_cast<int>(stage), "W",
      stage_weight_matrix.data());

    if (is_initial_stage) {
      yref = {};
      std::copy(x0.begin(), x0.end(), yref.begin());
    } else {
      yref = {blended_position[0],
              blended_position[1],
              blended_yaw,
              blended_velocity,
              0.0,
              0.0,
              0.0,
              0.0};
    }
    ocp_nlp_cost_model_set(
      impl_->config, impl_->dims, impl_->in, static_cast<int>(stage), "yref", yref.data());
  }

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
  double terminal_velocity_weight = 0.0;
  if (goal_terminal_reference) {
    const auto & goal = impl_->params.goal;
    const auto goal_block =
      position_block(goal_terminal_reference->yaw, goal.weight_longitudinal, goal.weight_lateral);
    for (size_t i = 0; i < terminal_block.size(); ++i) {
      terminal_block[i] += goal_block[i];
    }
    terminal_yaw_weight += goal.weight_yaw;
    terminal_velocity_weight += goal.weight_velocity;
  }
  // Do not fold the previous terminal into yref_e. After age-based resampling the last
  // temporal sample is clamped to the previous horizon end, whose heading is often stale
  // relative to the current DP/goal yaw. Blending those anisotropic lon/lat frames with
  // the goal pose produced a kinematically unreachable last point. Mid-horizon stages
  // still carry the temporal term.
  std::array<double, gen_nyn * gen_nyn> terminal_weight_matrix{};
  terminal_weight_matrix[w_index(kX, kX, gen_nyn)] = terminal_block[0];
  terminal_weight_matrix[w_index(kY, kY, gen_nyn)] = terminal_block[1];
  terminal_weight_matrix[w_index(kX, kY, gen_nyn)] = terminal_block[2];
  terminal_weight_matrix[w_index(kY, kX, gen_nyn)] = terminal_block[2];
  terminal_weight_matrix[w_index(kPsi, kPsi, gen_nyn)] = terminal_yaw_weight;
  terminal_weight_matrix[w_index(kV, kV, gen_nyn)] = terminal_velocity_weight;
  ocp_nlp_cost_model_set(
    impl_->config, impl_->dims, impl_->in, static_cast<int>(gen_n), "W",
    terminal_weight_matrix.data());

  std::array<double, gen_nyn> yref_e{
    terminal_position[0], terminal_position[1], terminal_yaw, terminal_velocity, 0.0, 0.0};
  ocp_nlp_cost_model_set(
    impl_->config, impl_->dims, impl_->in, static_cast<int>(gen_n), "yref", yref_e.data());

  for (size_t stage = 0; stage <= gen_n; ++stage) {
    std::array<double, gen_nx> x_guess = x0;
    if (warm_start != nullptr) {
      x_guess = warm_start->states[stage];
    }
    ocp_nlp_out_set(
      impl_->config, impl_->dims, impl_->out, impl_->in, static_cast<int>(stage), "x",
      x_guess.data());
    if (stage < gen_n) {
      std::array<double, gen_nu> u_guess{};
      if (warm_start != nullptr) {
        u_guess = warm_start->inputs[std::min(stage, gen_n - 1)];
      }
      ocp_nlp_out_set(
        impl_->config, impl_->dims, impl_->out, impl_->in, static_cast<int>(stage), "u",
        u_guess.data());
    }
  }

  SolverSolution solution;
  solution.status = kinematic_bicycle_time_seq_acados_solve(impl_->capsule);
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

}  // namespace autoware::trajectory_modifier::time_sequence_raw
