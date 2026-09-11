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

#ifndef AUTOWARE__DIFFUSION_PLANNER__OPTIMIZATION__OPTIMIZER_PARAMS_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__OPTIMIZATION__OPTIMIZER_PARAMS_HPP_

namespace autoware::diffusion_planner::optimization
{

/**
 * @brief Runtime parameters for the acados-based trajectory optimization.
 *
 * Cost weights and constraint bounds are injected into the generated solver at startup,
 * so tuning them does not require regenerating the acados code
 * (see scripts/generate_solver.py for the baked-in defaults and the OCP definition).
 */
struct TrajectoryOptimizationParams
{
  bool enable{false};

  // Cost weights (LINEAR_LS).
  // The position error is split along the reference heading: longitudinal errors
  // (ahead/behind the time schedule, i.e. velocity-profile freedom) and lateral errors
  // (path deviation) are weighted separately via a per-stage rotated 2x2 weight block.
  // Velocity and steering angle carry no tracking weight: the model outputs positions
  // and headings only, so no reference exists for them.
  double weight_longitudinal{0.5};
  double weight_lateral{0.5};
  double weight_yaw{0.05};
  double weight_acceleration{0.1};
  double weight_steering_rate{10.0};
  // Terminal state weight = terminal_weight_scale * stage state weight.
  double terminal_weight_scale{2.5};

  // State bounds (stages 1..N). min_velocity_mps >= 0 prevents backward motion.
  double min_velocity_mps{0.0};
  double max_velocity_mps{30.0};

  // Input bounds.
  double min_acceleration_mps2{-4.0};
  double max_acceleration_mps2{3.0};
  double max_steering_rate_rps{1.0};

  // Soft nonlinear constraint |v^2 * tan(delta) / wheelbase| <= max_lateral_acceleration_mps2.
  double max_lateral_acceleration_mps2{3.0};

  int max_sqp_iterations{50};
};

}  // namespace autoware::diffusion_planner::optimization

#endif  // AUTOWARE__DIFFUSION_PLANNER__OPTIMIZATION__OPTIMIZER_PARAMS_HPP_
