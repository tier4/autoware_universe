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

#include "autoware/trajectory_processor/acados_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <vector>

namespace temporal_mpt
{
namespace
{
bool acados_mpt_verbose()
{
  static const bool verbose = [] {
    const char * env = std::getenv("ACADOS_MPT_VERBOSE");
    return env != nullptr && env[0] != '\0' && env[0] != '0';
  }();
  return verbose;
}

void limit_blas_threads_once()
{
  // Tiny HPIPM QPs lose more to thread-pool wakeup than they gain from extra cores.
  // Do not override a value the user already set.
  static const bool once = [] {
    ::setenv("OPENBLAS_NUM_THREADS", "1", 0);
    ::setenv("MKL_NUM_THREADS", "1", 0);
    return true;
  }();
  (void)once;
}
}  // namespace

AcadosInterface::AcadosInterface()
{
  limit_blas_threads_once();
  capsule_ = kinematic_bicycle_temporal_acados_create_capsule();
  kinematic_bicycle_temporal_acados_create(capsule_);

  double lbx0[KINEMATIC_BICYCLE_TEMPORAL_NX] = {0};
  double ubx0[KINEMATIC_BICYCLE_TEMPORAL_NX] = {0};

  nlp_config_ = kinematic_bicycle_temporal_acados_get_nlp_config(capsule_);
  nlp_dims_ = kinematic_bicycle_temporal_acados_get_nlp_dims(capsule_);
  nlp_in_ = kinematic_bicycle_temporal_acados_get_nlp_in(capsule_);
  nlp_out_ = kinematic_bicycle_temporal_acados_get_nlp_out(capsule_);
  nlp_solver_ = kinematic_bicycle_temporal_acados_get_nlp_solver(capsule_);

  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", lbx0);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", ubx0);
}

AcadosInterface::~AcadosInterface()
{
  kinematic_bicycle_temporal_acados_free(capsule_);
  kinematic_bicycle_temporal_acados_free_capsule(capsule_);
}

void AcadosInterface::setParameters(int stage, std::array<double, NP> params)
{
  kinematic_bicycle_temporal_acados_update_params(
    capsule_, stage, const_cast<double *>(params.data()), NP);
}

void AcadosInterface::setParametersAllStages(std::array<double, NP> params)
{
  ocp_nlp_dims * nlp_dims = kinematic_bicycle_temporal_acados_get_nlp_dims(capsule_);
  for (int i = 0; i <= nlp_dims->N; ++i) {
    kinematic_bicycle_temporal_acados_update_params(
      capsule_, i, const_cast<double *>(params.data()), NP);
  }
}

void AcadosInterface::setSteerRateLimit(double max_steer_rate_rad_s)
{
  const double lim = std::max(1.0e-6, max_steer_rate_rad_s);
  double lh[1] = {-lim};
  double uh[1] = {lim};
  for (int i = 0; i < static_cast<int>(N); ++i) {
    // Stage 0 uses nh_0; skip when codegen has NH0==0.
    if (i == 0 && KINEMATIC_BICYCLE_TEMPORAL_NH0 == 0) {
      continue;
    }
    if (i > 0 && KINEMATIC_BICYCLE_TEMPORAL_NH == 0) {
      continue;
    }
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i, "lh", lh);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, i, "uh", uh);
  }
}

namespace
{
// Path-frame LINEAR_LS: e_lon/e_lat from (x,y); identity on psi,v,a,delta.
// Stage ny=8 (nx+nu), terminal ny=6. Acados stores Vx column-major.

void fill_path_frame_vx_stage(double psi, double * vx_colmajor)
{
  for (int i = 0; i < static_cast<int>(NY) * static_cast<int>(NX); ++i) {
    vx_colmajor[i] = 0.0;
  }
  const double c = std::cos(psi);
  const double s = std::sin(psi);
  const int ny = static_cast<int>(NY);
  vx_colmajor[0] = c;
  vx_colmajor[1] = -s;
  vx_colmajor[ny] = s;
  vx_colmajor[ny + 1] = c;
  vx_colmajor[2 * ny + 2] = 1.0;  // psi
  vx_colmajor[3 * ny + 3] = 1.0;  // v
  vx_colmajor[4 * ny + 4] = 1.0;  // a
  vx_colmajor[5 * ny + 5] = 1.0;  // delta
}

void fill_path_frame_vx_terminal(double psi, double * vx_colmajor)
{
  for (int i = 0; i < static_cast<int>(NYN) * static_cast<int>(NX); ++i) {
    vx_colmajor[i] = 0.0;
  }
  const double c = std::cos(psi);
  const double s = std::sin(psi);
  const int ny = static_cast<int>(NYN);
  vx_colmajor[0] = c;
  vx_colmajor[1] = -s;
  vx_colmajor[ny] = s;
  vx_colmajor[ny + 1] = c;
  vx_colmajor[2 * ny + 2] = 1.0;
  vx_colmajor[3 * ny + 3] = 1.0;
  vx_colmajor[4 * ny + 4] = 1.0;
  vx_colmajor[5 * ny + 5] = 1.0;
}

std::array<double, NY> path_frame_yref_stage(const std::array<double, NY> & yref_world)
{
  const double xr = yref_world[0];
  const double yr = yref_world[1];
  const double psi = yref_world[2];
  const double c = std::cos(psi);
  const double s = std::sin(psi);
  return {c * xr + s * yr, -s * xr + c * yr, psi,           yref_world[3],
          yref_world[4],   yref_world[5],    yref_world[6], yref_world[7]};
}

std::array<double, NYN> path_frame_yref_terminal(const std::array<double, NYN> & yref_world)
{
  const double xr = yref_world[0];
  const double yr = yref_world[1];
  const double psi = yref_world[2];
  const double c = std::cos(psi);
  const double s = std::sin(psi);
  return {c * xr + s * yr, -s * xr + c * yr, psi, yref_world[3], yref_world[4], yref_world[5]};
}
}  // namespace

void AcadosInterface::setStageReference(int stage, std::array<double, NY> yref)
{
  double vx[NY * NX];
  fill_path_frame_vx_stage(yref[2], vx);
  ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, stage, "Vx", vx);
  const auto yref_pf = path_frame_yref_stage(yref);
  ocp_nlp_cost_model_set(
    nlp_config_, nlp_dims_, nlp_in_, stage, "yref", const_cast<double *>(yref_pf.data()));
}

void AcadosInterface::setTerminalReference(std::array<double, NYN> yref_e)
{
  double vx[NYN * NX];
  fill_path_frame_vx_terminal(yref_e[2], vx);
  ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, static_cast<int>(N), "Vx", vx);
  const auto yref_pf = path_frame_yref_terminal(yref_e);
  ocp_nlp_cost_model_set(
    nlp_config_, nlp_dims_, nlp_in_, static_cast<int>(N), "yref",
    const_cast<double *>(yref_pf.data()));
}

void AcadosInterface::setWarmStart(std::array<double, NX> x0, std::array<double, NU> u0)
{
  ocp_nlp_dims * nlp_dims = kinematic_bicycle_temporal_acados_get_nlp_dims(capsule_);
  for (int i = 0; i < nlp_dims->N; ++i) {
    ocp_nlp_out_set(
      nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", const_cast<double *>(x0.data()));
    ocp_nlp_out_set(
      nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", const_cast<double *>(u0.data()));
  }
  ocp_nlp_out_set(
    nlp_config_, nlp_dims_, nlp_out_, nlp_in_, nlp_dims->N, "x", const_cast<double *>(x0.data()));
}

void AcadosInterface::setWarmStartTrajectory(
  const std::array<std::array<double, NX>, N + 1> & xtraj,
  const std::array<std::array<double, NU>, N> & utraj)
{
  ocp_nlp_dims * nlp_dims = kinematic_bicycle_temporal_acados_get_nlp_dims(capsule_);
  for (int i = 0; i < nlp_dims->N; ++i) {
    ocp_nlp_out_set(
      nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x",
      const_cast<double *>(xtraj[static_cast<size_t>(i)].data()));
    ocp_nlp_out_set(
      nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u",
      const_cast<double *>(utraj[static_cast<size_t>(i)].data()));
  }
  ocp_nlp_out_set(
    nlp_config_, nlp_dims_, nlp_out_, nlp_in_, nlp_dims->N, "x",
    const_cast<double *>(xtraj[static_cast<size_t>(nlp_dims->N)].data()));
}

void AcadosInterface::setInitialState(std::array<double, NX> x0)
{
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", const_cast<double *>(x0.data()));
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", const_cast<double *>(x0.data()));
}

std::array<std::array<double, NX>, N + 1> AcadosInterface::getStateTrajectory() const
{
  std::array<std::array<double, NX>, N + 1> xtraj;
  for (size_t ii = 0; ii <= static_cast<size_t>(nlp_dims_->N); ii++) {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &xtraj[ii]);
  }
  return xtraj;
}

std::array<std::array<double, NU>, N> AcadosInterface::getControlTrajectory() const
{
  std::array<std::array<double, NU>, N> utraj;
  for (size_t ii = 0; ii < static_cast<size_t>(N); ii++) {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &utraj[ii]);
  }
  return utraj;
}

void AcadosInterface::appendInequalityLambdaReport(std::ostream & ss) const
{
  constexpr double kAMin = -3.5;
  constexpr double kAMax = 3.5;
  constexpr double kDMin = -0.7;
  constexpr double kDMax = 0.7;

  double a_min = 1e9;
  double a_max = -1e9;
  double d_min = 1e9;
  double d_max = -1e9;
  int near_a_lo = 0;
  int near_a_hi = 0;
  int near_d_lo = 0;
  int near_d_hi = 0;

  for (int k = 0; k < static_cast<int>(N); ++k) {
    double u[NU] = {0.0, 0.0};
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, k, "u", u);
    a_min = std::min(a_min, u[0]);
    a_max = std::max(a_max, u[0]);
    d_min = std::min(d_min, u[1]);
    d_max = std::max(d_max, u[1]);
    if (u[0] <= kAMin + 1e-3) {
      ++near_a_lo;
    }
    if (u[0] >= kAMax - 1e-3) {
      ++near_a_hi;
    }
    if (u[1] <= kDMin + 1e-3) {
      ++near_d_lo;
    }
    if (u[1] >= kDMax - 1e-3) {
      ++near_d_hi;
    }
  }

  ss << "[acados] plant: first-order Dubins (no delay FIFOs); h=steer_rate\n";
  ss << "[acados] u_cmd: a∈[" << a_min << "," << a_max << "]  delta∈[" << d_min << "," << d_max
     << "]\n";
  ss << "[acados] u within 1e-3 of bound: a_lo=" << near_a_lo << " a_hi=" << near_a_hi
     << " d_lo=" << near_d_lo << " d_hi=" << near_d_hi << "\n";
}

int AcadosInterface::solve()
{
  return kinematic_bicycle_temporal_acados_solve(capsule_);
}

AcadosSolution AcadosInterface::getControl(std::array<double, NX> x0)
{
  double kkt_norm_inf = 0.0;
  double elapsed_time = 0.0;
  int sqp_iter = 0;

  setInitialState(x0);

  int status = kinematic_bicycle_temporal_acados_solve(capsule_);
  ocp_nlp_get(nlp_solver_, "time_tot", &elapsed_time);
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);
  ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iter);

  AcadosSolution solution;
  solution.xtraj = getStateTrajectory();
  solution.utraj = getControlTrajectory();
  solution.sqp_iter = sqp_iter;
  solution.kkt_norm_inf = kkt_norm_inf;
  solution.elapsed_time = elapsed_time;
  solution.status = status;

  const bool dump = acados_mpt_verbose() || status != 0;
  if (dump) {
    if (acados_mpt_verbose()) {
      kinematic_bicycle_temporal_acados_print_stats(capsule_);
    }
    std::stringstream ss;
    ss << "\nSolver info:\n";
    ss << " status " << status << "\n SQP iterations " << sqp_iter << "\n time "
       << elapsed_time * 1000 << " [ms]\n KKT " << kkt_norm_inf << "\n";
    appendInequalityLambdaReport(ss);
    std::cerr << ss.str() << std::flush;
    solution.info = ss.str();
  }
  return solution;
}

}  // namespace temporal_mpt
