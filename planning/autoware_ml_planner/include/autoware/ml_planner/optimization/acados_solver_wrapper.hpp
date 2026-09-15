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

#ifndef AUTOWARE__ML_PLANNER__OPTIMIZATION__ACADOS_SOLVER_WRAPPER_HPP_
#define AUTOWARE__ML_PLANNER__OPTIMIZATION__ACADOS_SOLVER_WRAPPER_HPP_

#include "autoware/ml_planner/optimization/optimizer_params.hpp"

#include <array>
#include <cstddef>
#include <memory>

namespace autoware::ml_planner::optimization
{

// OCP dimensions. Must match scripts/generate_solver.py (static_asserts in the .cpp
// verify them against the generated code).
constexpr size_t opt_horizon = 80;  // = OUTPUT_T model output steps
constexpr size_t opt_nx = 5;        // x, y, yaw, velocity, steering angle
constexpr size_t opt_nu = 2;        // acceleration, steering rate
constexpr double opt_dt_s = 0.1;    // = PREDICTION_TIME_STEP_S

/// Per-stage tracking reference (positions in the solver's local frame).
/// Only pose is tracked: the model outputs positions and headings only, so no velocity
/// reference exists. The velocity profile emerges from the time-indexed position tracking
/// and the input regularization/bounds.
struct StageReference
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct SolverSolution
{
  int status{-1};  // acados status, 0 on success
  int sqp_iterations{0};
  double solve_time_s{0.0};
  std::array<std::array<double, opt_nx>, opt_horizon + 1> states{};
  std::array<std::array<double, opt_nu>, opt_horizon> inputs{};

  [[nodiscard]] bool success() const { return status == 0; }
};

/**
 * @brief Thin RAII wrapper around the generated acados OCP solver.
 *
 * Owns the solver capsule and hides the generated C headers (pimpl), so this header
 * stays usable without the code-generated sources. Weights and bounds are written into
 * the solver once at construction.
 */
class AcadosSolverWrapper
{
public:
  AcadosSolverWrapper(
    const TrajectoryOptimizationParams & params, double wheelbase_m, double max_steering_angle_rad);
  ~AcadosSolverWrapper();

  AcadosSolverWrapper(const AcadosSolverWrapper &) = delete;
  AcadosSolverWrapper & operator=(const AcadosSolverWrapper &) = delete;
  AcadosSolverWrapper(AcadosSolverWrapper &&) = delete;
  AcadosSolverWrapper & operator=(AcadosSolverWrapper &&) = delete;

  /**
   * @brief Solve the OCP.
   *
   * @param initial_state Stage-0 state (equality constrained): x, y, yaw, v, delta.
   * @param references Tracking references for stages 1..N; the last entry doubles as the
   *                   terminal reference. Positions must share the frame of initial_state.
   * @param warm_start Previous solution in the same frame, or nullptr for a cold start.
   */
  SolverSolution solve(
    const std::array<double, opt_nx> & initial_state,
    const std::array<StageReference, opt_horizon> & references, const SolverSolution * warm_start);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autoware::ml_planner::optimization

#endif  // AUTOWARE__ML_PLANNER__OPTIMIZATION__ACADOS_SOLVER_WRAPPER_HPP_
