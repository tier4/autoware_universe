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

#pragma once

#include <array>
#include <cstddef>
#include <string>

#ifdef MAX_ITER
#undef MAX_ITER
#endif

extern "C" {
#include "acados_solver_combined_longitudinal_lateral.h"
#include "acados_sim_solver_combined_longitudinal_lateral.h"
}

constexpr size_t NX = COMBINED_LONGITUDINAL_LATERAL_NX;
constexpr size_t NP = COMBINED_LONGITUDINAL_LATERAL_NP;
constexpr size_t NU = COMBINED_LONGITUDINAL_LATERAL_NU;
constexpr size_t N = COMBINED_LONGITUDINAL_LATERAL_N;

struct AcadosSolution
{
  std::array<std::array<double, NX>, N + 1> xtraj;
  std::array<std::array<double, NU>, N> utraj;

  int sqp_iter;
  double kkt_norm_inf;
  double elapsed_time;

  int status;
  std::string info;
  /** Per-iteration SQP stats table (when requested). Empty if not filled. */
  std::string solver_stats;
  /** Inequality constraint Lagrange multipliers (lam) for stage 0,1,2 when requested. Non-zero => constraint active. */
  std::string lam_summary;
};

class AcadosInterface
{
public:
  AcadosInterface(int max_iter = 20, double tol = 1e-4);
  ~AcadosInterface();

  int solve();
  AcadosSolution getControl(std::array<double, NX> x0);

  /**
   * Predict state after delay_time using the acados sim integrator (same dynamics as OCP).
   * Use with p = [tau_equiv, kappa_ref, lf, lr]. Result is x0 for the OCP; then apply stage 0.
   */
  std::array<double, NX> predictStateAfterDelay(
    const std::array<double, NX> & x,
    double delay_time,
    const std::array<double, NP> & p);

  /**
   * Solve with delay compensation: predict state after delay_time (acados sim), use as x0,
   * solve, return solution. Apply sol.utraj[0] as the command (it takes effect at t+delay_time).
   * If delay_time <= 0, uses current_state as x0 (no prediction).
   */
  AcadosSolution getControlWithDelayCompensation(
    const std::array<double, NX> & current_state,
    double delay_time,
    const std::array<double, NP> & p);

  /** Convenience: build p = [tau_equiv, kappa_ref, lf, lr] and call getControlWithDelayCompensation. */
  AcadosSolution getControlWithDelayCompensation(
    const std::array<double, NX> & current_state,
    double delay_time,
    double kappa_ref,
    double tau_equiv = 1.5,
    double lf = 1.0,
    double lr = 1.0);

  void setSolverOptions(int max_iter, double tol);
  void setInitialState(std::array<double, NX> x0);
  void setParameters(int stage, std::array<double, NP> params);
  void setParametersAllStages(std::array<double, NP> params);
  void setWarmStart(std::array<double, NX> x0, std::array<double, NU> u0);

  /**
   * Set path cost reference so the MPC tracks velocity and advancing path position.
   * OCP cost is LINEAR_LS with y = [s, v, a, eY, ePsi, u_cmd, delta]. Without this,
   * yref is zero and the MPC drives v→0 and s→0 (vehicle never moves).
   * Call before getControl/getControlWithDelayCompensation.
   * @param s0 current path position [m]
   * @param v_ref desired velocity from trajectory [m/s]
   */
  void setCostReference(double s0, double v_ref);

private:
  std::array<std::array<double, NX>, N + 1> getStateTrajectory() const;
  std::array<std::array<double, NU>, N> getControlTrajectory() const;
  /** Build per-iteration SQP stats table from ocp_nlp "statistics". */
  std::string getSolverStatsString() const;
  /** Format inequality multipliers (lam) for stages 0,1,2 for debugging active constraints. */
  std::string getLambdasString() const;

  combined_longitudinal_lateral_solver_capsule * capsule_;
  ocp_nlp_config * nlp_config_;
  ocp_nlp_dims * nlp_dims_;
  ocp_nlp_in * nlp_in_;
  ocp_nlp_out * nlp_out_;
  ocp_nlp_solver * nlp_solver_;

  combined_longitudinal_lateral_sim_solver_capsule * sim_capsule_;
};
