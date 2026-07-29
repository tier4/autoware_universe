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

#ifndef MPT_OPTIMIZER_HPP_
#define MPT_OPTIMIZER_HPP_

#include "type_alias.hpp"

#include <autoware/acados_mpt_optimizer/optimizer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <optional>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

namespace mpt = autoware::acados_mpt_optimizer;

/**
 * @brief Build the OptimizerParams from the node parameters and the vehicle info.
 *
 * The corridor is left at its default (disabled): the first integration step optimises the path
 * shape and its speed profile alone, with no drivable-area or object constraint.
 */
mpt::OptimizerParams make_mpt_optimizer_params(const Params & params, const VehicleInfo & vehicle);

/**
 * @brief Turn one planned trajectory into the NLP input.
 *
 * The velocities carried by `traj_points` are the caps the planner has decided so far (map speed
 * limits, and zeros from the modifier / map-based stop points), so they become the per-point
 * velocity cap and, at the first zero, the stop station. They are *not* a speed profile: the NLP
 * builds one, jointly with the shape.
 */
mpt::OptimizationInput make_optimization_input(
  const TrajectoryPoints & traj_points, const Odometry & odometry, double acceleration,
  const std::optional<double> & ego_curvature);

//! Convert the solved trajectory back. `z` and the pose come from the solver; the velocity and the
//! acceleration are its states, not a finite difference; `time_from_start` is the solver's own,
//! recovered from the arc length and the speed profile.
TrajectoryPoints to_trajectory_points(const std::vector<mpt::OptimizedPoint> & points);

/**
 * @brief Minimum speed to hand the controller when pulling out from standstill.
 *
 * The NLP pins its initial speed to the ego's, so a trajectory planned from a standstill starts at
 * exactly zero - and the longitudinal controller only leaves its stopped state once the speed at
 * the ego pose exceeds its engage threshold. Without this the vehicle plans a perfectly good
 * departure and then never takes it. The velocity smoother has the same mechanism
 * (`set_engage_speed`), and these are the same parameters, so both paths engage alike.
 */
struct EngageParams
{
  bool enable{true};
  double speed{0.25};                        //!< [m/s]
  double acceleration{0.5};                  //!< [m/s^2]
  double stop_dist_to_prohibit_engage{0.5};  //!< [m] a stop this close means "stay stopped"
};

/**
 * @brief Joint path-shape / speed-profile optimiser (acados NLP), used in place of the elastic
 * band smoother plus the jerk-filtered velocity smoother.
 *
 * The two stages it replaces decide shape first and speed second, so a shape that violates the
 * curvature / lateral acceleration / steering limits can only be paid for by decelerating - and
 * curvature itself can never be fixed by a velocity filter. Here both are decided in one problem
 * with those limits as constraints.
 *
 * A solved problem is not a satisfied one (the nonlinear limits are slacked), so `optimize`
 * returns nullopt whenever the result must not be used and the caller has to fall back.
 */
class MptOptimizer
{
public:
  MptOptimizer(
    const mpt::OptimizerParams & params, const EngageParams & engage, const rclcpp::Logger & logger,
    rclcpp::Clock::SharedPtr clock);

  void update_params(const mpt::OptimizerParams & params);

  //! @param ego_curvature measured path curvature (tan(steer) / wheel_base). Unset falls back to
  //! the curvature of the input path at the ego, which is estimated over a couple of metres and is
  //! therefore very sensitive to the ego's lateral offset from the path.
  //! @return the optimised trajectory, or nullopt when the input is unusable or the solve failed
  //! its own verification. Never throws.
  std::optional<TrajectoryPoints> optimize(
    const TrajectoryPoints & traj_points, const Odometry & odometry, double acceleration,
    const std::optional<double> & ego_curvature);

private:
  //! Raise the profile to the engage speed when pulling out; see EngageParams.
  void apply_engage_speed(
    TrajectoryPoints & traj_points, const TrajectoryPoints & input,
    const Odometry & odometry) const;

  mpt::Optimizer optimizer_;
  EngageParams engage_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // MPT_OPTIMIZER_HPP_
