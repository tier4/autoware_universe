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

#ifndef AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__SSC_QP_TRAJECTORY_OPTIMIZER_HPP_
#define AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__SSC_QP_TRAJECTORY_OPTIMIZER_HPP_

#include "ssc_corridor.hpp"
#include "trajectory_optimizer_interface.hpp"

#include <optional>
#include <string>
#include <vector>

namespace autoware::safety_planner
{

// Trajectory optimization as the convex QP of the spatio-temporal semantic corridor
// (arXiv:1906.09788).
//
// The semantic constraints become a sequence of (s, l, t) cubes, each cube gets one piece of a
// quintic Bezier, and the control points of s(t) and l(t) are solved in a single QP. Through the
// two properties of the Bernstein basis,
//   (P1) convex hull: putting the control points in a box puts the whole curve in that box
//   (P2) hodograph: the control points of a derivative are a linear map of the original ones
// the free space and the longitudinal and lateral speeds and accelerations become **linear
// inequalities** in the control points, while the objective, the squared jerk integral, is a
// quadratic form in them: a convex QP.
//
// Because these conditions are sufficient, the constraints hold **over the whole interval** rather
// than at sample points. Unlike the NLP formulation, neither the heading nor the curvature is a
// variable, the curvature being a ratio of derivatives and hence not expressible linearly. The ego
// is treated as a point, and its shape is absorbed by inflating the obstacles when carving the
// cubes.

//! Parameters of the QP (ROS namespace `trajectory_optimizer.ssc_qp.*`)
struct SscQpParams
{
  SscCorridorParams corridor;

  double lateral_rate_max_mps{2.0};    //!< [m/s] bound on the lateral speed, (P2) at k = 1
  double lateral_accel_max_mps2{2.0};  //!< [m/s^2] bound on the lateral acceleration, (P2) at k = 2

  double weight_jerk_s{1.0};    //!< weight of the longitudinal squared jerk integral
  double weight_jerk_l{1.0};    //!< weight of the lateral one
  double regularization{1e-8};  //!< added to the diagonal of P; the jerk Hessian has the
                                //!< polynomials of degree two and below in its null space

  double osqp_eps_abs{1e-5};
};

//! The raw solution of the QP: six control points each for s and l, per cube
struct SscQpSolution
{
  //! ordered as [the six s of cube 0, the six l of cube 0, the six s of cube 1, ...]
  std::vector<double> control_points;
  double alpha{0.0};  //!< [s] length of a piece, the same for every cube
};

class SscQpTrajectoryOptimizer : public TrajectoryOptimizerInterface
{
public:
  std::string get_name() const override { return "ssc_qp"; }

  TrajectoryOptimizerResult optimize(const TrajectoryOptimizerInput & input) override;

private:
  //! Reads the ROS parameters; on_initialize must have run
  SscQpParams read_params() const;
};

// ---------------------------------------------------------------------------------------------
// free functions, so that they can be tested outside the plugin
// ---------------------------------------------------------------------------------------------

//! The initial and terminal state in Frenet coordinates (position, speed, acceleration), which
//! enter the QP as equality constraints
struct SscBoundaryState
{
  double s{0.0};
  double s_dot{0.0};
  double s_ddot{0.0};
  double l{0.0};
  double l_dot{0.0};
  double l_ddot{0.0};
};

//! Converts one point of the rough plan, with its world speed, acceleration and heading, into the
//! Frenet derivatives of s and l. It is a small-deviation approximation scaled by (1 - k_ref l),
//! and the acceleration ignores the change of heading.
SscBoundaryState to_frenet_boundary_state(
  const PlannerContext & context, const RoughPlanPoint & point, double s, double l);

//! [m/s] Upper bound on the longitudinal speed over the s range of a cube: the interval VELOCITY
//! constraints of the IR together with sqrt(a_lat_nom / |k|) from the reference curvature. The
//! curvature is not a variable of SSC, so the only place this can land is a bound on the
//! longitudinal speed of each cube.
double cube_velocity_upper(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const SemanticCube & cube, const KinematicLimits & limits);

//! Builds the QP from the cubes and the boundary conditions and solves it, returning nullopt when
//! it is infeasible or the solver fails. velocity_upper holds one longitudinal speed bound per
//! cube.
std::optional<SscQpSolution> solve_ssc_qp(
  const std::vector<SemanticCube> & cubes, const std::vector<double> & velocity_upper,
  const SscBoundaryState & initial, const SscBoundaryState & terminal,
  const KinematicLimits & limits, const SscQpParams & params);

//! Samples the solution on the time grid of the rough plan and converts it back to world
//! coordinates
OptimizedTrajectory sample_ssc_solution(
  const PlannerContext & context, const std::vector<SemanticCube> & cubes,
  const SscQpSolution & solution, const std::vector<double> & sample_times);

}  // namespace autoware::safety_planner

#endif  // AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__SSC_QP_TRAJECTORY_OPTIMIZER_HPP_
