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

#ifndef AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__TRAJECTORY_OPTIMIZER_INTERFACE_HPP_
#define AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__TRAJECTORY_OPTIMIZER_INTERFACE_HPP_

// trajectory_optimizer: given the homotopy the rough planner settled on (which side to pass, who
// yields, whether to stop), it produces one refined trajectory that satisfies the constraints. It
// does **no** non-convex search of its own.
//
// Two formulations are being tried, so the layer is a pluginlib plugin selected by the parameter
// `trajectory_optimizer.plugin`, in the same shape as the constraint generators:
//
// | plugin                   | decision variables                              | problem    |
// |---|---|---|
// | NlpTrajectoryOptimizer   | states (px,py,theta,k,v,a) and inputs (w,j)     | non-convex |
// | SscQpTrajectoryOptimizer | piecewise Bezier control points of s(t) and l(t)| convex QP  |
//                              SSC (arXiv:1906.09788)
//
// Conventions of the interface:
// - the time grid is the one of the rough plan (t_k = k*dt), so no grid conversion is needed
// - coordinates are world coordinates (the planning frame); a formulation working in Frenet
//   converts back before returning
// - the constraints of record are compiled_constraints.raw_constraints; the projected views are for
//   coarse evaluation, and the exact one (carving the corridor, validation) reads raw

#include "../../constraint.hpp"
#include "../../context.hpp"
#include "../../type_alias.hpp"
#include "constraints_compiler.hpp"
#include "rough_planner.hpp"

#include <autoware_utils_debug/time_keeper.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::safety_planner
{

// ---------------------------------------------------------------------------------------------
// output types
// ---------------------------------------------------------------------------------------------

//! One point of an optimized trajectory: the state (px, py, theta, k, v, a) and the input (w, j).
//! The input is the one held from this point to the next, and is 0 at the last point (k = N).
struct OptimizedTrajectoryPoint
{
  double t{0.0};      //!< [s] relative to the planning reference time
  Pose2d pose{};      //!< world coordinates (px, py, theta)
  double kappa{0.0};  //!< [1/m] path curvature
  double v{0.0};      //!< [m/s]
  double a{0.0};      //!< [m/s^2]
  double w{0.0};      //!< [1/(m·s)] dκ/dt
  double j{0.0};      //!< [m/s³]  da/dt
};

struct OptimizedTrajectory
{
  std::vector<OptimizedTrajectoryPoint> points;  //!< N+1 points (t_k = k*dt)
};

//! Outcome of optimize(). On anything but SUCCESS the caller picks a fallback, such as emitting
//! the rough plan as it is.
enum class TrajectoryOptimizerStatus : std::uint8_t {
  SUCCESS,          //!< a solution was found and passed the validation
  INFEASIBLE,       //!< no solution satisfies the constraints, fallbacks included
  SOLVER_ERROR,     //!< an internal error of the solver, a NaN, or the budget was exceeded
  NOT_IMPLEMENTED,  //!< not implemented yet
};

//! For the report and the markers only; nothing here changes the behavior
struct TrajectoryOptimizerDebug
{
  std::string message;  //!< human readable explanation, such as the reason for a failure
  double elapsed_ms{0.0};
  int which_level{0};            //!< which fallback level succeeded (0 = none was needed)
  bool used_certificate{false};  //!< whether the last-resort stop trajectory was returned
  MarkerArray debug_markers;
};

struct TrajectoryOptimizerResult
{
  OptimizedTrajectory trajectory;  //!< only valid when status == SUCCESS
  TrajectoryOptimizerStatus status{TrajectoryOptimizerStatus::NOT_IMPLEMENTED};
  TrajectoryOptimizerDebug debug;
};

// ---------------------------------------------------------------------------------------------
// input types
// ---------------------------------------------------------------------------------------------

//! Everything optimize() takes. The references live for that one call, and a plugin must not hold
//! them across cycles.
struct TrajectoryOptimizerInput
{
  const PlannerContext & context;
  //! The constraint IR: raw is the source of truth, the views are for coarse evaluation
  const CompiledConstraints & compiled_constraints;
  //! The homotopy chosen by the rough planner, which is both the tracking reference and the
  //! initial guess
  const RoughPlan & rough_plan;
  //! The result of the previous cycle, for the warm start; nullopt on the first cycle and after a
  //! failure
  const std::optional<OptimizedTrajectory> & prev_trajectory;
};

// ---------------------------------------------------------------------------------------------
// the plugin base
// ---------------------------------------------------------------------------------------------

class TrajectoryOptimizerInterface
{
public:
  TrajectoryOptimizerInterface() = default;
  virtual ~TrajectoryOptimizerInterface() = default;

  void on_initialize(const std::shared_ptr<TimeKeeper> time_keeper, const Params & params)
  {
    time_keeper_ = time_keeper;
    params_ = params;
  }

  virtual std::string get_name() const = 0;

  //! Produces one trajectory that satisfies the constraints, within the homotopy of the rough
  //! plan
  virtual TrajectoryOptimizerResult optimize(const TrajectoryOptimizerInput & input) = 0;

protected:
  mutable std::shared_ptr<TimeKeeper> time_keeper_{nullptr};
  Params params_;
};

}  // namespace autoware::safety_planner

#endif  // AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__TRAJECTORY_OPTIMIZER_INTERFACE_HPP_
