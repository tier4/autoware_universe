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

#include "acados_interface_time.hpp"

#include <sstream>

namespace time_mpt
{

AcadosInterfaceTime::AcadosInterfaceTime()
{
  capsule_ = kinematic_bicycle_time_acados_create_capsule();
  kinematic_bicycle_time_acados_create(capsule_);

  nlp_config_ = kinematic_bicycle_time_acados_get_nlp_config(capsule_);
  nlp_dims_ = kinematic_bicycle_time_acados_get_nlp_dims(capsule_);
  nlp_in_ = kinematic_bicycle_time_acados_get_nlp_in(capsule_);
  nlp_out_ = kinematic_bicycle_time_acados_get_nlp_out(capsule_);
  nlp_solver_ = kinematic_bicycle_time_acados_get_nlp_solver(capsule_);

  // Initial-state equality (lbx_0 = ubx_0 = 0 placeholder; updated each solve).
  double lbx0[NX] = {0};
  double ubx0[NX] = {0};
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", lbx0);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", ubx0);
}

AcadosInterfaceTime::~AcadosInterfaceTime()
{
  kinematic_bicycle_time_acados_free(capsule_);
  kinematic_bicycle_time_acados_free_capsule(capsule_);
}

void AcadosInterfaceTime::setParameters(int stage, const std::array<double, NP> & params)
{
  kinematic_bicycle_time_acados_update_params(
    capsule_, stage, const_cast<double *>(params.data()), NP);
}

void AcadosInterfaceTime::setParametersAllStages(const std::array<double, NP> & params)
{
  for (int i = 0; i <= nlp_dims_->N; ++i) {
    kinematic_bicycle_time_acados_update_params(
      capsule_, i, const_cast<double *>(params.data()), NP);
  }
}

void AcadosInterfaceTime::setStageReference(int stage, const std::array<double, NY> & yref)
{
  ocp_nlp_cost_model_set(
    nlp_config_, nlp_dims_, nlp_in_, stage, "yref", const_cast<double *>(yref.data()));
}

void AcadosInterfaceTime::setTerminalReference(const std::array<double, NYN> & yref_e)
{
  ocp_nlp_cost_model_set(
    nlp_config_, nlp_dims_, nlp_in_, static_cast<int>(N), "yref",
    const_cast<double *>(yref_e.data()));
}

void AcadosInterfaceTime::setStageCostDiagonalAllStages(
  double w_x, double w_y, double w_psi, double w_delta, double w_u_delta, double w_v)
{
  std::array<double, NY * NY> W{};
  // Diagonal entries; layout is identical in row-/column-major for diagonal matrices.
  W[0 * NY + 0] = w_x;
  W[1 * NY + 1] = w_y;
  W[2 * NY + 2] = w_psi;
  W[3 * NY + 3] = w_delta;
  W[4 * NY + 4] = w_u_delta;
  W[5 * NY + 5] = w_v;
  for (int k = 0; k < nlp_dims_->N; ++k) {
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, k, "W", W.data());
  }
}

void AcadosInterfaceTime::setTerminalCostDiagonal(
  double w_x, double w_y, double w_psi, double w_delta)
{
  std::array<double, NYN * NYN> W_e{};
  W_e[0 * NYN + 0] = w_x;
  W_e[1 * NYN + 1] = w_y;
  W_e[2 * NYN + 2] = w_psi;
  W_e[3 * NYN + 3] = w_delta;
  ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, static_cast<int>(N), "W", W_e.data());
}

void AcadosInterfaceTime::setInitialState(const std::array<double, NX> & x0)
{
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", const_cast<double *>(x0.data()));
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", const_cast<double *>(x0.data()));
}

void AcadosInterfaceTime::setDeltaBoxAllStages(double delta_max)
{
  // At codegen time idxbx = [3] (delta only); NBX = 1 for stages 1..N and the terminal.
  double lb[1] = {-delta_max};
  double ub[1] = {delta_max};
  for (int i = 1; i <= nlp_dims_->N; ++i) {
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i, "lbx", lb);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i, "ubx", ub);
  }
}

void AcadosInterfaceTime::setInputBoxAllStages(double u_delta_max, double v_min, double v_max)
{
  // idxbu = [0, 1] -> [u_delta, v]
  double lb[2] = {-u_delta_max, v_min};
  double ub[2] = {u_delta_max, v_max};
  for (int i = 0; i < nlp_dims_->N; ++i) {
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i, "lbu", lb);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i, "ubu", ub);
  }
}

void AcadosInterfaceTime::setWarmStart(
  const std::array<std::array<double, NX>, N + 1> & xtraj,
  const std::array<std::array<double, NU>, N> & utraj)
{
  for (int i = 0; i < nlp_dims_->N; ++i) {
    ocp_nlp_out_set(
      nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", const_cast<double *>(xtraj[i].data()));
    ocp_nlp_out_set(
      nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", const_cast<double *>(utraj[i].data()));
  }
  ocp_nlp_out_set(
    nlp_config_, nlp_dims_, nlp_out_, nlp_in_, nlp_dims_->N, "x",
    const_cast<double *>(xtraj[nlp_dims_->N].data()));
}

void AcadosInterfaceTime::setWarmStartUniform(const std::array<double, NX> & x_ref)
{
  std::array<double, NU> u0{};
  for (int i = 0; i < nlp_dims_->N; ++i) {
    ocp_nlp_out_set(
      nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", const_cast<double *>(x_ref.data()));
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", u0.data());
  }
  ocp_nlp_out_set(
    nlp_config_, nlp_dims_, nlp_out_, nlp_in_, nlp_dims_->N, "x",
    const_cast<double *>(x_ref.data()));
}

std::array<std::array<double, NX>, N + 1> AcadosInterfaceTime::getStateTrajectory() const
{
  std::array<std::array<double, NX>, N + 1> xtraj;
  for (size_t ii = 0; ii <= static_cast<size_t>(nlp_dims_->N); ii++) {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &xtraj[ii]);
  }
  return xtraj;
}

std::array<std::array<double, NU>, N> AcadosInterfaceTime::getControlTrajectory() const
{
  std::array<std::array<double, NU>, N> utraj;
  for (size_t ii = 0; ii < static_cast<size_t>(N); ii++) {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &utraj[ii]);
  }
  return utraj;
}

AcadosTimeSolution AcadosInterfaceTime::getControl(const std::array<double, NX> & x0)
{
  setInitialState(x0);

  int status = kinematic_bicycle_time_acados_solve(capsule_);

  double kkt_norm_inf = 0.0;
  double elapsed_time = 0.0;
  int sqp_iter = 0;
  ocp_nlp_get(nlp_solver_, "time_tot", &elapsed_time);
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);
  ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iter);

  std::stringstream ss;
  ss << "SQP iter " << sqp_iter << ", solve " << elapsed_time * 1000.0 << " ms, KKT "
     << kkt_norm_inf;

  AcadosTimeSolution solution;
  solution.xtraj = getStateTrajectory();
  solution.utraj = getControlTrajectory();
  solution.sqp_iter = sqp_iter;
  solution.kkt_norm_inf = kkt_norm_inf;
  solution.elapsed_time = elapsed_time;
  solution.status = status;
  solution.info = ss.str();
  return solution;
}

}  // namespace time_mpt
