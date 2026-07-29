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
#include "velocity_smoother.hpp"

#include <autoware/acados_mpt_optimizer/optimizer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <optional>
#include <string>
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
 * @brief Drop the part of the trajectory the vehicle has already driven.
 *
 * The planned path is drawn from `path_planning.path_length.backward` metres behind the ego (5 m as
 * shipped), so the trajectory reaching the velocity optimisation stage does not start at the
 * vehicle. The NLP has no use for that part - it anchors its horizon at the ego pose - and every
 * station the caller measures on the input (the stop station below, the engage distance) is
 * measured from the first point, so cropping here is what makes those stations and the horizon
 * share one origin instead of relying on two independent nearest-point searches agreeing.
 */
TrajectoryPoints crop_behind_ego(
  const TrajectoryPoints & traj_points, const geometry_msgs::msg::Pose & ego_pose);

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
  /**
   * @param velocity_smoother optional. When given, its jerk-filtered profile seeds the NLP's speed
   * guess: a recorded run spent ~12 ms and 4 SQP iterations per solve and 0.2 ms when handed its
   * own solution, so the solve time is the guess quality, and the smoother costs about 1 ms. It is
   * fed the *curvature-capped* speeds rather than the raw map limits, because in a corner the cap
   * is the better guess (the smoother's own lateral filter has a 2.74 m/s floor) while the smoother
   * is the better one through the acceleration transitions.
   */
  MptOptimizer(
    const mpt::OptimizerParams & params, const EngageParams & engage, const rclcpp::Logger & logger,
    rclcpp::Clock::SharedPtr clock, std::shared_ptr<VelocitySmoother> velocity_smoother = nullptr);

  void update_params(const mpt::OptimizerParams & params);

  //! @param ego_curvature measured path curvature (tan(steer) / wheel_base). Unset falls back to
  //! the curvature of the input path at the ego, which is estimated over a couple of metres and is
  //! therefore very sensitive to the ego's lateral offset from the path.
  //! @return the optimised trajectory, or nullopt when the input is unusable or the solve failed
  //! its own verification. Never throws.
  std::optional<TrajectoryPoints> optimize(
    const TrajectoryPoints & traj_points, const Odometry & odometry, double acceleration,
    const std::optional<double> & ego_curvature);

  /**
   * @brief What the last `optimize` call was given and what it produced, for rviz.
   *
   * Namespaces `ego`, `input` and `output`, in the frame the caller stamps them with. Rebuilt on
   * every call - including the ones that fall back, where the output is the last SQP iterate and is
   * exactly what one needs to look at. Every namespace is always present, empty when it has nothing
   * to show, so a marker from a previous cycle is overwritten rather than left on screen.
   */
  const visualization_msgs::msg::MarkerArray & debug_markers() const { return debug_markers_; }

  /**
   * @brief One human-readable block per call: the ego state the NLP was pinned to, what it was
   * given, what the solver reported and how the result measured up against the limits.
   *
   * The numbers that decide whether a solve is good are spread over the input, the solver status
   * and the ConstraintReport, and none of them survives into the published trajectory - so without
   * this the only way to tell a 26 ms straight solve from a 140 ms one that violated the lateral
   * acceleration limit is to correlate log lines by hand. Rebuilt on every call, including the ones
   * that fall back, where it carries the reason.
   */
  const std::string & debug_info() const { return debug_info_; }

private:
  //! Raise the profile to the engage speed when pulling out; see EngageParams.
  void apply_engage_speed(
    TrajectoryPoints & traj_points, const TrajectoryPoints & input,
    const Odometry & odometry) const;

  //! Fill debug_markers_ from the ego, the (cropped) NLP input and the solver's points.
  void build_debug_markers(
    const Odometry & odometry, const TrajectoryPoints & input,
    const std::vector<mpt::OptimizedPoint> & output);

  /**
   * @brief The jerk-filtered speed profile to seed the NLP with, sampled at `traj_points`.
   *
   * Empty when there is no smoother. The smoother resamples internally, so its result is mapped
   * back onto the input stations by arc length. It never sees - and never updates - the state the
   * fallback path relies on.
   */
  std::vector<double> smoothed_guess_velocities(
    const TrajectoryPoints & traj_points, const Odometry & odometry, double acceleration) const;

  //! Fill debug_info_ from one solve. The paths that never reach the solver set it to their reason.
  void build_debug_info(const mpt::OptimizationInput & input, const mpt::Result & result);

  mpt::Optimizer optimizer_;
  //! Borrowed, and only for the initial guess; the node keeps using it for the fallback.
  std::shared_ptr<VelocitySmoother> velocity_smoother_;
  visualization_msgs::msg::MarkerArray debug_markers_{};
  std::string debug_info_{};
  EngageParams engage_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // MPT_OPTIMIZER_HPP_
