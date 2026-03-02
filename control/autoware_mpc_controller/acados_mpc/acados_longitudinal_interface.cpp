// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use it except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "acados_longitudinal_interface.hpp"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/sim_interface.h"

#include <cmath>
#include <sstream>

namespace
{
constexpr double kSimStepMax = 0.01;
constexpr double kTf = 5.0;  // must match mpc_longitudinal_only.py
constexpr int kNyPath = LONGITUDINAL_ONLY_NY;   // 4
constexpr int kNyTerminal = LONGITUDINAL_ONLY_NYN;  // 3
}  // namespace

AcadosLongitudinalInterface::AcadosLongitudinalInterface(int max_iter, double tol)
{
  capsule_ = longitudinal_only_acados_create_capsule();
  longitudinal_only_acados_create(capsule_);
  sim_capsule_ = longitudinal_only_acados_sim_solver_create_capsule();
  longitudinal_only_acados_sim_create(sim_capsule_);

  double lbx0[LON_NX] = {0};
  double ubx0[LON_NX] = {0};
  nlp_config_ = longitudinal_only_acados_get_nlp_config(capsule_);
  nlp_dims_ = longitudinal_only_acados_get_nlp_dims(capsule_);
  nlp_in_ = longitudinal_only_acados_get_nlp_in(capsule_);
  nlp_out_ = longitudinal_only_acados_get_nlp_out(capsule_);
  nlp_solver_ = longitudinal_only_acados_get_nlp_solver(capsule_);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", lbx0);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", ubx0);
  setSolverOptions(max_iter, tol);
}

AcadosLongitudinalInterface::~AcadosLongitudinalInterface()
{
  longitudinal_only_acados_sim_free(sim_capsule_);
  longitudinal_only_acados_sim_solver_free_capsule(sim_capsule_);
  longitudinal_only_acados_free(capsule_);
  longitudinal_only_acados_free_capsule(capsule_);
}

void AcadosLongitudinalInterface::setSolverOptions(int max_iter, double tol)
{
  void * nlp_opts = longitudinal_only_acados_get_nlp_opts(capsule_);
  if (nlp_opts) {
    ocp_nlp_solver_opts_set(nlp_config_, nlp_opts, "max_iter", &max_iter);
    double tol_val = tol;
    ocp_nlp_solver_opts_set(nlp_config_, nlp_opts, "tol_stat", &tol_val);
    ocp_nlp_solver_opts_set(nlp_config_, nlp_opts, "tol_eq", &tol_val);
    ocp_nlp_solver_opts_set(nlp_config_, nlp_opts, "tol_ineq", &tol_val);
    ocp_nlp_solver_opts_set(nlp_config_, nlp_opts, "tol_comp", &tol_val);
  }
}

void AcadosLongitudinalInterface::setCostReference(double s0, double v_ref)
{
  const double dt = kTf / static_cast<double>(LON_N);
  std::array<double, kNyPath> yref_path{};
  for (int stage = 0; stage < static_cast<int>(LON_N); ++stage) {
    yref_path[0] = s0 + v_ref * static_cast<double>(stage) * dt;
    yref_path[1] = v_ref;
    yref_path[2] = 0.0;
    yref_path[3] = 0.0;
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, stage, "yref", yref_path.data());
  }
  std::array<double, kNyTerminal> yref_terminal{};
  yref_terminal[0] = s0 + v_ref * static_cast<double>(LON_N) * dt;
  yref_terminal[1] = v_ref;
  yref_terminal[2] = 0.0;
  ocp_nlp_cost_model_set(
    nlp_config_, nlp_dims_, nlp_in_, static_cast<int>(LON_N), "yref", yref_terminal.data());
}

void AcadosLongitudinalInterface::setInitialState(std::array<double, LON_NX> x0)
{
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", x0.data());
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", x0.data());
}

void AcadosLongitudinalInterface::setParameters(double tau_equiv)
{
  double p[LON_NP] = {tau_equiv};
  for (int i = 0; i <= static_cast<int>(LON_N); ++i) {
    longitudinal_only_acados_update_params(capsule_, i, p, LON_NP);
  }
}

std::array<double, LON_NX> AcadosLongitudinalInterface::predictStateAfterDelay(
  const std::array<double, LON_NX> & x,
  double delay_time,
  double tau_equiv,
  double u_hold)
{
  if (delay_time <= 0.0) {
    return x;
  }
  sim_config * acados_sim_config = longitudinal_only_acados_get_sim_config(sim_capsule_);
  void * acados_sim_dims = longitudinal_only_acados_get_sim_dims(sim_capsule_);
  sim_in * acados_sim_in = longitudinal_only_acados_get_sim_in(sim_capsule_);
  sim_out * acados_sim_out = longitudinal_only_acados_get_sim_out(sim_capsule_);

  double p[LON_NP] = {tau_equiv};
  longitudinal_only_acados_sim_update_params(sim_capsule_, p, LON_NP);

  const int n_steps = std::max(1, static_cast<int>(std::ceil(delay_time / kSimStepMax)));
  double T = delay_time / n_steps;
  std::array<double, LON_NX> x_current = x;
  std::array<double, LON_NU> u_hold_arr = {u_hold};

  for (int i = 0; i < n_steps; ++i) {
    sim_in_set(acados_sim_config, acados_sim_dims, acados_sim_in, "x", x_current.data());
    sim_in_set(acados_sim_config, acados_sim_dims, acados_sim_in, "u", u_hold_arr.data());
    sim_in_set(acados_sim_config, acados_sim_dims, acados_sim_in, "T", &T);
    longitudinal_only_acados_sim_solve(sim_capsule_);
    sim_out_get(acados_sim_config, acados_sim_dims, acados_sim_out, "x", x_current.data());
  }
  return x_current;
}

int AcadosLongitudinalInterface::solve()
{
  return longitudinal_only_acados_solve(capsule_);
}

AcadosLongitudinalSolution AcadosLongitudinalInterface::getControl(std::array<double, LON_NX> x0)
{
  setInitialState(x0);
  int status = longitudinal_only_acados_solve(capsule_);
  double elapsed_time = 0.0;
  ocp_nlp_get(nlp_solver_, "time_tot", &elapsed_time);
  int sqp_iter = 0;
  ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iter);

  std::array<double, LON_NU> u0{};
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", u0.data());

  std::ostringstream ss;
  ss << "Longitudinal MPC: status=" << status << " sqp_iter=" << sqp_iter
     << " time_ms=" << (elapsed_time * 1000.0);

  AcadosLongitudinalSolution sol;
  sol.u_cmd = u0[0];
  sol.status = status;
  sol.sqp_iter = sqp_iter;
  sol.elapsed_time = elapsed_time;
  sol.info = ss.str();
  return sol;
}
