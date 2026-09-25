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

#ifndef AUTOWARE__ML_PLANNER__OPTIMIZATION__OPTIMIZER_PARAMS_HPP_
#define AUTOWARE__ML_PLANNER__OPTIMIZATION__OPTIMIZER_PARAMS_HPP_

namespace autoware::ml_planner::optimization
{

/**
 * @brief Runtime parameters for the acados-based trajectory optimization.
 *
 * Cost weights and constraint bounds are injected when the solver is constructed or
 * reconfigured, so tuning them does not require regenerating the acados code
 * (see scripts/generate_solver.py for the baked-in defaults and the OCP definition).
 */
struct TrajectoryOptimizationParams
{
  bool enable{false};

  // Cost weights (LINEAR_LS).
  // The position error is split along the reference heading: longitudinal errors
  // (ahead/behind the time schedule, i.e. velocity-profile freedom) and lateral errors
  // (path deviation) are weighted separately via a per-stage rotated 2x2 weight block.
  double weight_longitudinal{0.5};
  double weight_lateral{0.5};
  double weight_yaw{0.05};
  // Penalize velocity and steering-angle magnitude relative to zero.
  double weight_velocity{0.01};
  double weight_steering_angle{1.0};
  double weight_acceleration{0.1};
  double weight_steering_rate{10.0};
  // Terminal state weight = terminal_weight_scale * stage state weight.
  double terminal_weight_scale{2.5};
  struct GoalParams
  {
    double weight_longitudinal{5.0};
    double weight_lateral{5.0};
    double weight_yaw{0.5};
    double weight_velocity{0.1};
    double snap_distance_m{1.0};
  } goal;

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

  /**
   * @brief Temporal consistency: weakly track the previous cycle's own plan.
   *
   * The model output is re-sampled every cycle and is not temporally coherent, so
   * consecutive plans can differ even when nothing in the scene changed. These weights add a
   * second tracking term whose reference is the previous solution of the same candidate,
   * shifted by the time elapsed since it was computed, which makes the published trajectory
   * evolve smoothly instead of jumping between equally good interpretations of the scene.
   *
   * The term is a low-pass filter on the plan and trades consistency against responsiveness
   * to new model output. Same longitudinal/lateral split as the tracking weights - a lower
   * longitudinal weight lets the timing float while the path stays put.
   *
   * The weights decay over the horizon. The near stages are what the controller actually
   * executes, so they should be held still, while the far stages are a prediction that is
   * *supposed* to move as new information arrives - pinning them down is what makes the
   * planner slow to react. The weights are therefore scaled per stage by
   *
   *   alpha(t) = far_weight_ratio + (1 - far_weight_ratio) * exp(-t / decay_time_constant_s)
   *
   * where t is the stage's time from the start of the horizon.
   *
   * The reference is the previous *solver solution*, not the previous published trajectory,
   * so postprocessing (e.g. stop point fixing) does not feed back into the optimization. It
   * is only applied while a previous solution is available and fresh, i.e. under exactly the
   * conditions that also make it usable as a warm start.
   */
  struct TemporalConsistencyParams
  {
    bool enable{false};
    // These are the weights at t = 0; the decay below scales them down over the horizon, so
    // they may exceed the tracking weights without making the planner sluggish overall.
    double weight_longitudinal{0.4};
    double weight_lateral{2.0};
    double weight_yaw{0.2};
    double weight_velocity{0.4};
    /// Time constant of the decay over the horizon [s]. <= 0 keeps the weights uniform.
    double decay_time_constant_s{1.0};
    /// Fraction of the weight that remains at the far end of the horizon, in [0, 1].
    double far_weight_ratio{0.05};
  } temporal_consistency;

  /**
   * @brief Bypass the solver while stopped and publish a held (or goal-zeroed) steering angle.
   *
   * Enter Hold when ego speed and the reference trajectory are both below the stopped
   * thresholds. The steering angle is latched on entry and kept until the vehicle is moving
   * again. Enter Zero when ego is stopped within goal_steer_zero_distance_m of the route goal;
   * that regime stays latched until the goal is no longer near, so a brief creep does not
   * restore the held angle.
   */
  struct SteerStopHoldParams
  {
    bool enable{true};
    double stopped_velocity_threshold_mps{0.15};
    double stopped_trajectory_max_length_m{1.5};
    bool goal_steer_zero_enable{true};
    double goal_steer_zero_distance_m{5.0};
    /// When true, zero-steer is entered only after ego is stopped. When false, proximity to
    /// the goal is enough.
    bool goal_steer_zero_requires_stopped{true};
  } steer_stop_hold;
};

}  // namespace autoware::ml_planner::optimization

#endif  // AUTOWARE__ML_PLANNER__OPTIMIZATION__OPTIMIZER_PARAMS_HPP_
