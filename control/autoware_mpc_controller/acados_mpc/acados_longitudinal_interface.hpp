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

#pragma once

#include <array>
#include <cstddef>
#include <string>

extern "C" {
#include "acados_solver_longitudinal_only.h"
#include "acados_sim_solver_longitudinal_only.h"
}

constexpr size_t LON_NX = LONGITUDINAL_ONLY_NX;
constexpr size_t LON_NP = LONGITUDINAL_ONLY_NP;
constexpr size_t LON_NU = LONGITUDINAL_ONLY_NU;
constexpr size_t LON_N = LONGITUDINAL_ONLY_N;

struct AcadosLongitudinalSolution
{
  double u_cmd{0.0};
  int status{0};
  int sqp_iter{0};
  double elapsed_time{0.0};
  std::string info;
};

class AcadosLongitudinalInterface
{
public:
  AcadosLongitudinalInterface(int max_iter = 20, double tol = 1e-4);
  ~AcadosLongitudinalInterface();

  void setCostReference(double s0, double v_ref);
  void setInitialState(std::array<double, LON_NX> x0);
  void setParameters(double tau_equiv);
  int solve();
  AcadosLongitudinalSolution getControl(std::array<double, LON_NX> x0);

  std::array<double, LON_NX> predictStateAfterDelay(
    const std::array<double, LON_NX> & x,
    double delay_time,
    double tau_equiv,
    double u_hold);

  void setSolverOptions(int max_iter, double tol);

private:
  longitudinal_only_solver_capsule * capsule_{nullptr};
  ocp_nlp_config * nlp_config_{nullptr};
  ocp_nlp_dims * nlp_dims_{nullptr};
  ocp_nlp_in * nlp_in_{nullptr};
  ocp_nlp_out * nlp_out_{nullptr};
  ocp_nlp_solver * nlp_solver_{nullptr};
  longitudinal_only_sim_solver_capsule * sim_capsule_{nullptr};
};
