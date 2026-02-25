// Copyright 2025 TIER IV, Inc.
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

#include "acados_interface.hpp"

#include "acados_c/sim_interface.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>

namespace
{
constexpr double kSimStepMax = 0.01;  // max step for sim integration [s]
}

AcadosInterface::AcadosInterface(int max_iter, double tol)
{
  capsule_ = combined_longitudinal_lateral_acados_create_capsule();
  combined_longitudinal_lateral_acados_create(capsule_);

  sim_capsule_ = combined_longitudinal_lateral_acados_sim_solver_create_capsule();
  combined_longitudinal_lateral_acados_sim_create(sim_capsule_);

  double lbx0[NX] = {0};
  double ubx0[NX] = {0};

  nlp_config_ = combined_longitudinal_lateral_acados_get_nlp_config(capsule_);
  nlp_dims_ = combined_longitudinal_lateral_acados_get_nlp_dims(capsule_);
  nlp_in_ = combined_longitudinal_lateral_acados_get_nlp_in(capsule_);
  nlp_out_ = combined_longitudinal_lateral_acados_get_nlp_out(capsule_);
  nlp_solver_ = combined_longitudinal_lateral_acados_get_nlp_solver(capsule_);

  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", lbx0);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", ubx0);

  setSolverOptions(max_iter, tol);
}

AcadosInterface::~AcadosInterface()
{
  combined_longitudinal_lateral_acados_sim_free(sim_capsule_);
  combined_longitudinal_lateral_acados_sim_solver_free_capsule(sim_capsule_);
  combined_longitudinal_lateral_acados_free(capsule_);
  combined_longitudinal_lateral_acados_free_capsule(capsule_);
}

std::array<double, NX> AcadosInterface::predictStateAfterDelay(
  const std::array<double, NX> & x,
  double delay_time,
  const std::array<double, NP> & p)
{
  if (delay_time <= 0.0) {
    return x;
  }
  ::sim_config * acados_sim_config = combined_longitudinal_lateral_acados_get_sim_config(sim_capsule_);
  void * acados_sim_dims = combined_longitudinal_lateral_acados_get_sim_dims(sim_capsule_);
  sim_in * acados_sim_in = combined_longitudinal_lateral_acados_get_sim_in(sim_capsule_);
  sim_out * acados_sim_out = combined_longitudinal_lateral_acados_get_sim_out(sim_capsule_);

  const int n_steps = std::max(1, static_cast<int>(std::ceil(delay_time / kSimStepMax)));
  double T = delay_time / n_steps;

  std::array<double, NX> x_current = x;
  std::array<double, NU> u_zero{};
  u_zero.fill(0.0);

  combined_longitudinal_lateral_acados_sim_update_params(sim_capsule_, const_cast<double *>(p.data()), NP);

  for (int i = 0; i < n_steps; ++i) {
    sim_in_set(acados_sim_config, acados_sim_dims, acados_sim_in, "x", x_current.data());
    sim_in_set(acados_sim_config, acados_sim_dims, acados_sim_in, "u", u_zero.data());
    sim_in_set(acados_sim_config, acados_sim_dims, acados_sim_in, "T", &T);
    combined_longitudinal_lateral_acados_sim_solve(sim_capsule_);
    sim_out_get(acados_sim_config, acados_sim_dims, acados_sim_out, "x", x_current.data());
  }
  return x_current;
}

void AcadosInterface::setSolverOptions(int max_iter, double tol)
{
  void * nlp_opts = combined_longitudinal_lateral_acados_get_nlp_opts(capsule_);
  if (nlp_opts == nullptr) {
    std::cerr << "Warning: nlp_opts is null, cannot set solver options" << std::endl;
    return;
  }
  ocp_nlp_solver_opts_set(nlp_config_, nlp_opts, "max_iter", &max_iter);
  double tol_val = tol;
  ocp_nlp_solver_opts_set(nlp_config_, nlp_opts, "tol_stat", &tol_val);
  ocp_nlp_solver_opts_set(nlp_config_, nlp_opts, "tol_eq", &tol_val);
  ocp_nlp_solver_opts_set(nlp_config_, nlp_opts, "tol_ineq", &tol_val);
  ocp_nlp_solver_opts_set(nlp_config_, nlp_opts, "tol_comp", &tol_val);
}

void AcadosInterface::setParameters(int stage, std::array<double, NP> params)
{
  combined_longitudinal_lateral_acados_update_params(
    capsule_, stage, const_cast<double *>(params.data()), NP);
}

void AcadosInterface::setParametersAllStages(std::array<double, NP> params)
{
  for (int i = 0; i <= static_cast<int>(N); ++i) {
    combined_longitudinal_lateral_acados_update_params(
      capsule_, i, const_cast<double *>(params.data()), NP);
  }
}

void AcadosInterface::setWarmStart(std::array<double, NX> x0, std::array<double, NU> u0)
{
  for (size_t i = 0; i < N; ++i) {
    ocp_nlp_out_set(
      nlp_config_, nlp_dims_, nlp_out_, nlp_in_, static_cast<int>(i), "x", x0.data());
    ocp_nlp_out_set(
      nlp_config_, nlp_dims_, nlp_out_, nlp_in_, static_cast<int>(i), "u", u0.data());
  }
  ocp_nlp_out_set(
    nlp_config_, nlp_dims_, nlp_out_, nlp_in_, static_cast<int>(N), "x", x0.data());
}

void AcadosInterface::setInitialState(std::array<double, NX> x0)
{
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", x0.data());
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", x0.data());
}

std::array<std::array<double, NX>, N + 1> AcadosInterface::getStateTrajectory() const
{
  std::array<std::array<double, NX>, N + 1> xtraj;
  for (size_t ii = 0; ii <= N; ii++) {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, static_cast<int>(ii), "x", &xtraj[ii]);
  }
  return xtraj;
}

std::array<std::array<double, NU>, N> AcadosInterface::getControlTrajectory() const
{
  std::array<std::array<double, NU>, N> utraj;
  for (size_t ii = 0; ii < N; ii++) {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, static_cast<int>(ii), "u", &utraj[ii]);
  }
  return utraj;
}

int AcadosInterface::solve()
{
  return combined_longitudinal_lateral_acados_solve(capsule_);
}

AcadosSolution AcadosInterface::getControlWithDelayCompensation(
  const std::array<double, NX> & current_state,
  double delay_time,
  const std::array<double, NP> & p)
{
  if (delay_time <= 0.0) {
    return getControl(current_state);
  }
  std::array<double, NX> x0 = predictStateAfterDelay(current_state, delay_time, p);
  return getControl(x0);
}

AcadosSolution AcadosInterface::getControlWithDelayCompensation(
  const std::array<double, NX> & current_state,
  double delay_time,
  double kappa_ref,
  double tau_equiv,
  double lf,
  double lr)
{
  std::array<double, NP> p = {tau_equiv, kappa_ref, lf, lr};
  return getControlWithDelayCompensation(current_state, delay_time, p);
}

AcadosSolution AcadosInterface::getControl(std::array<double, NX> x0)
{
  setInitialState(x0);

  int status = combined_longitudinal_lateral_acados_solve(capsule_);
  double elapsed_time = 0.0;
  ocp_nlp_get(nlp_solver_, "time_tot", &elapsed_time);

  double kkt_norm_inf = 0.0;
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);
  int sqp_iter = 0;
  ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iter);

  combined_longitudinal_lateral_acados_print_stats(capsule_);

  std::stringstream ss;
  ss << "\nSolver info: SQP iter " << sqp_iter << " time " << (elapsed_time * 1000) << " ms KKT "
     << kkt_norm_inf << std::endl;

  AcadosSolution solution;
  solution.xtraj = getStateTrajectory();
  solution.utraj = getControlTrajectory();
  solution.sqp_iter = sqp_iter;
  solution.kkt_norm_inf = kkt_norm_inf;
  solution.elapsed_time = elapsed_time;
  solution.status = status;
  solution.info = ss.str();
  return solution;
}
