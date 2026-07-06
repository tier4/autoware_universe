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

#ifndef AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__PSEUDO_CONTROLLER_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__PSEUDO_CONTROLLER_HPP_

#include <Eigen/Dense>

#include <vector>

namespace autoware::diffusion_planner::postprocess
{

/**
 * @brief Tunable parameters for `StanleyPseudoController`.
 */
struct StanleyControllerParams
{
  double max_steer_rad;            // Maximum steering (tire) angle [rad]
  double max_steer_rate_rad_s;     // Steering rate limit [rad/s]
  double stop_velocity_threshold;  // [m/s] below this, freeze steering tracking
  double heading_gain;             // Weight on the heading-error steering term
  double cross_track_gain;         // Weight on the cross-track-error steering term
};

/**
 * @brief Abstract interface for a "pseudo controller" that simulates ego motion converging from
 * its current state onto a network-predicted path/velocity plan.
 *
 * The network output path is not guaranteed to start at the ego base-link (data augmentation
 * during training shifts the origin of the predicted path), so instead of using the predicted
 * poses directly, implementations forward-simulate the ego state so it converges smoothly onto
 * the plan rather than jumping onto it. This is abstracted behind an interface because the
 * convergence algorithm/vehicle model used here is expected to change over time.
 */
class PseudoController
{
public:
  PseudoController() = default;
  PseudoController(const PseudoController &) = default;
  PseudoController(PseudoController &&) = default;
  PseudoController & operator=(const PseudoController &) = default;
  PseudoController & operator=(PseudoController &&) = default;
  virtual ~PseudoController() = default;

  /**
   * @brief Simulates a trajectory converging from the current ego state onto the predicted path.
   *
   * The reference target for each time step is derived internally by resampling `path_poses` at
   * the travel distance obtained from integrating `velocities`, so the caller passes the raw
   * network path and the velocity plan without any temporal resampling.
   *
   * @param path_poses The raw predicted path poses (map frame) describing the spatial path.
   * @param velocities The commanded longitudinal velocity at each time step [m/s].
   * @param ego_pose The current ego base-link pose (map frame) to start the simulation from.
   * @param wheel_base The ego vehicle wheel base [m].
   * @param initial_steering The current steering angle [rad] to start the simulation from, so the
   * simulated trajectory continues smoothly from the vehicle's actual current wheel angle instead
   * of assuming it starts straight.
   * @return The simulated poses, one per element of `velocities`.
   */
  virtual std::vector<Eigen::Matrix4d> simulate(
    const std::vector<Eigen::Matrix4d> & path_poses, const std::vector<float> & velocities,
    const Eigen::Matrix4d & ego_pose, double wheel_base, double initial_steering) const = 0;
};

/**
 * @brief Converges onto the predicted path/velocity plan using a Stanley lateral controller
 * (heading + cross-track error steering law) driving a kinematic bicycle model.
 *
 * Steering is rate-limited, and steering tracking is frozen (decayed towards straight) below a
 * stopping speed so the wheel angle does not chatter while (nearly) stopped.
 */
class StanleyPseudoController : public PseudoController
{
public:
  explicit StanleyPseudoController(const StanleyControllerParams & params);

  std::vector<Eigen::Matrix4d> simulate(
    const std::vector<Eigen::Matrix4d> & path_poses, const std::vector<float> & velocities,
    const Eigen::Matrix4d & ego_pose, double wheel_base, double initial_steering) const override;

private:
  StanleyControllerParams params_;
};

/**
 * @brief Tunable parameters for `FeedbackLinearizationPseudoController`.
 */
struct FeedbackLinearizationControllerParams
{
  double max_steer_rad;            // Maximum steering (tire) angle delta_max [rad]
  double max_steer_rate_rad_s;     // Steering rate limit delta_dot_max [rad/s]
  double stop_velocity_threshold;  // [m/s] below this, freeze steering tracking (v_stop)
  double lateral_gain;             // Lateral-error feedback gain k_y (> 0)
  double heading_gain;             // Heading-error feedback gain k_psi (> 0)
};

/**
 * @brief Converges onto the predicted path/velocity plan using an exact feedback-linearization
 * state-feedback tracking law for the kinematic bicycle model.
 *
 * The kinematic bicycle model (state (x, y, psi), inputs (v, delta)) is treated as a unicycle
 * model (state (x, y, psi), inputs (v, omega)) by introducing the yaw rate omega = psi_dot. Given
 * a reference trajectory (x_r(t), y_r(t), psi_r(t), v_r(t)) with reference yaw rate
 * omega_r(t) = psi_r_dot(t), the position error is expressed in the ego frame as
 *
 *   e_x   =  cos(psi) * (x_r - x) + sin(psi) * (y_r - y)   (longitudinal error, unused here)
 *   e_y   = -sin(psi) * (x_r - x) + cos(psi) * (y_r - y)   (lateral error)
 *   e_psi = normalize(psi_r - psi)                         (heading error)
 *
 * The commanded velocity follows the reference exactly (v_cmd = v_r), and the commanded yaw rate
 * is the feedback-linearizing control law
 *
 *   omega_cmd = omega_r + v_r * (k_y * e_y + k_psi * sin(e_psi))
 *
 * which is converted back to a kinematic-bicycle steering angle via
 * delta_des = atan2(L * omega_cmd, v_cmd). Below `stop_velocity_threshold`, this inversion is
 * numerically unstable, so path-tracking steering is suspended (delta_des = 0). Steering is
 * clamped to `max_steer_rad` and rate-limited to `max_steer_rate_rad_s`.
 */
class FeedbackLinearizationPseudoController : public PseudoController
{
public:
  explicit FeedbackLinearizationPseudoController(
    const FeedbackLinearizationControllerParams & params);

  std::vector<Eigen::Matrix4d> simulate(
    const std::vector<Eigen::Matrix4d> & path_poses, const std::vector<float> & velocities,
    const Eigen::Matrix4d & ego_pose, double wheel_base, double initial_steering) const override;

private:
  FeedbackLinearizationControllerParams params_;
};

}  // namespace autoware::diffusion_planner::postprocess

#endif  // AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__PSEUDO_CONTROLLER_HPP_
