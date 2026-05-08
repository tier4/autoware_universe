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

#pragma once

#include <array>
#include <cstddef>
#include <string>

#ifdef MAX_ITER
#undef MAX_ITER
#endif

extern "C" {
#include "c_generated_code/acados_solver_kinematic_bicycle_time.h"
}

namespace time_mpt
{
constexpr size_t NX = KINEMATIC_BICYCLE_TIME_NX;    // 4: (X, Y, psi, delta)
constexpr size_t NU = KINEMATIC_BICYCLE_TIME_NU;    // 2: (u_delta = d delta / d t, v)
constexpr size_t NP = KINEMATIC_BICYCLE_TIME_NP;    // 2: (L, dt)
constexpr size_t N = KINEMATIC_BICYCLE_TIME_N;      // 40 stages
constexpr size_t NY = KINEMATIC_BICYCLE_TIME_NY;    // 6: (X, Y, psi, delta, u_delta, v)
constexpr size_t NYN = KINEMATIC_BICYCLE_TIME_NYN;  // 4: (X, Y, psi, delta)

struct AcadosTimeSolution
{
  std::array<std::array<double, NX>, N + 1> xtraj;
  std::array<std::array<double, NU>, N> utraj;

  int sqp_iter;
  double kkt_norm_inf;
  double elapsed_time;

  int status;
  std::string info;
};

class AcadosInterfaceTime
{
public:
  AcadosInterfaceTime();
  ~AcadosInterfaceTime();

  // Solve and read out solution. Sets initial-state equality constraint to x0 first.
  AcadosTimeSolution getControl(const std::array<double, NX> & x0);

  // ---- Reference / Parameters ----
  void setStageReference(int stage, const std::array<double, NY> & yref);
  void setTerminalReference(const std::array<double, NYN> & yref_e);
  void setParameters(int stage, const std::array<double, NP> & params);
  void setParametersAllStages(const std::array<double, NP> & params);

  // ---- Cost weights (diagonal). Updated each cycle from YAML. ----
  // Stage cost W is NY x NY (NY = 6: X, Y, psi, delta, u_delta, v).
  void setStageCostDiagonalAllStages(
    double w_x, double w_y, double w_psi, double w_delta, double w_u_delta, double w_v);
  // Terminal cost W_e is NYN x NYN (NYN = 4: X, Y, psi, delta).
  void setTerminalCostDiagonal(double w_x, double w_y, double w_psi, double w_delta);

  // ---- Constraints ----
  void setInitialState(const std::array<double, NX> & x0);
  // Set the |delta| <= delta_max state box on every non-initial stage and the terminal stage.
  void setDeltaBoxAllStages(double delta_max);
  // Set the input box on every stage:
  //   |u_delta| <= u_delta_max   (rad/s)
  //    v_min <= v <= v_max       (m/s)
  void setInputBoxAllStages(double u_delta_max, double v_min, double v_max);

  // ---- Warm start ----
  // Provide explicit warm start with the previous solution.
  void setWarmStart(
    const std::array<std::array<double, NX>, N + 1> & xtraj,
    const std::array<std::array<double, NU>, N> & utraj);
  // Set every stage to (x_ref, 0). Used for cold start or after large state jumps.
  void setWarmStartUniform(const std::array<double, NX> & x_ref);

private:
  std::array<std::array<double, NX>, N + 1> getStateTrajectory() const;
  std::array<std::array<double, NU>, N> getControlTrajectory() const;

  kinematic_bicycle_time_solver_capsule * capsule_;
  ocp_nlp_config * nlp_config_;
  ocp_nlp_dims * nlp_dims_;
  ocp_nlp_in * nlp_in_;
  ocp_nlp_out * nlp_out_;
  ocp_nlp_solver * nlp_solver_;
};

}  // namespace time_mpt
