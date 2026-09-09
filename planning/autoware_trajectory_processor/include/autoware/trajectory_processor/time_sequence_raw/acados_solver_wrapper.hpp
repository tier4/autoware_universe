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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TIME_SEQUENCE_RAW__ACADOS_SOLVER_WRAPPER_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TIME_SEQUENCE_RAW__ACADOS_SOLVER_WRAPPER_HPP_

#include "autoware/trajectory_processor/time_sequence_raw/optimizer_params.hpp"

#include <array>
#include <cstddef>
#include <memory>

namespace autoware::trajectory_processor::time_sequence_raw
{

constexpr size_t opt_horizon = 80;
constexpr size_t opt_nx = 6;  // x, y, yaw, velocity, steering angle, acceleration
constexpr size_t opt_nu = 2;  // jerk, steering rate
constexpr double opt_dt_s = 0.1;

// State indices in the acados model.
constexpr size_t kX = 0;
constexpr size_t kY = 1;
constexpr size_t kPsi = 2;
constexpr size_t kV = 3;
constexpr size_t kDelta = 4;
constexpr size_t kA = 5;

// Input indices in the acados model.
constexpr size_t kJerk = 0;
constexpr size_t kDeltaRate = 1;

// LINEAR_LS output indices: y = [x, y, psi, v, delta, a, jerk, delta_rate].
constexpr size_t kYJerk = opt_nx + kJerk;
constexpr size_t kYDeltaRate = opt_nx + kDeltaRate;

/// Per-stage tracking reference (positions in the solver's local frame).
/// Only pose is tracked: incoming trajectory speed is ignored.
struct StageReference
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct SolverSolution
{
  int status{-1};
  int sqp_iterations{0};
  double solve_time_s{0.0};
  std::array<std::array<double, opt_nx>, opt_horizon + 1> states{};
  std::array<std::array<double, opt_nu>, opt_horizon> inputs{};

  [[nodiscard]] bool success() const { return status == 0; }
};

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

  SolverSolution solve(
    const std::array<double, opt_nx> & initial_state,
    const std::array<StageReference, opt_horizon> & references, const SolverSolution * warm_start);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autoware::trajectory_processor::time_sequence_raw

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TIME_SEQUENCE_RAW__ACADOS_SOLVER_WRAPPER_HPP_
