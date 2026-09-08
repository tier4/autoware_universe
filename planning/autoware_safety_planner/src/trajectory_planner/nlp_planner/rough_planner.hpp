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

#ifndef AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__ROUGH_PLANNER_HPP_
#define AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__ROUGH_PLANNER_HPP_

// rough_planner owns the choice of the homotopy: which side to pass on, whether to yield or go
// first, whether to stop or drive through. The optimizer downstream performs no non-convex search.
//
// - the layer is **stateless**. What carries across cycles (PreviousPlanningResult) is owned by the
//   node and only read here through a const reference; the node writes the next one back from the
//   output
// - the discrete decisions are not carried across cycles either: they are **derived again from the
//   geometry of the trajectory every cycle**, and the ones carried over are used only for the
//   hysteresis comparison. Their keys are the ids that stay stable across cycles (the ones in
//   Source), never an arc length or an array index
// - the hysteresis has two layers: the ordering of the candidates keeps the decision as long as the
//   previous solution still holds, and on top of that a switch towards the unsafe side faces a
//   barrier while a switch towards more margin does not
// - the output is a **list of candidates in priority order**. Committing to the first feasible one
//   is a weakness, so the interface admits a top-K; for now K = 1 and only the first is consumed

#include "../../constraint.hpp"
#include "../../context.hpp"
#include "constraints_compiler.hpp"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace autoware::safety_planner
{

// ---------------------------------------------------------------------------------------------
// the discrete decisions
// ---------------------------------------------------------------------------------------------

enum class LeadLag : std::uint8_t { LEAD, FOLLOW };
enum class StopGo : std::uint8_t { STOP, GO };

//! The discrete decisions that define the homotopy, keyed by the ids that stay stable across
//! cycles (Source::target_id: a perception UUID, a lanelet id, ...). They are derived again from
//! the geometry of the trajectory every cycle, by derive_decisions().
struct Decisions
{
  std::map<std::string, Side> side;         //!< which side of a static object to pass on
  std::map<std::string, LeadLag> lead_lag;  //!< whether to lead or to follow a dynamic object
  std::map<std::string, StopGo> stop_go;    //!< whether to stop at or drive through a stop line
};

// ---------------------------------------------------------------------------------------------
// output types
// ---------------------------------------------------------------------------------------------

enum class RoughPlanSource : std::uint8_t {
  PREVIOUS_SOLUTION,  //!< the solution of the previous cycle, reused
  SPATIOTEMPORAL_DP,  //!< the space-time DP
  REFERENCE_FOLLOW,   //!< following the centerline, a stand-in that ignores the geometry
  STOP,               //!< the stop plan, the last resort and always feasible
};

//! One point of a rough plan. The time grid is the one of the optimizer downstream (t_k = k*dt),
//! so no grid conversion is needed.
struct RoughPlanPoint
{
  double t{0.0};      //!< [s]
  Pose2d pose{};      //!< world coordinates
  double kappa{0.0};  //!< [1/m] filled in here as the initial state of the optimizer
  double v{0.0};      //!< [m/s]
  double a{0.0};      //!< [m/s^2]
};

struct RoughPlan
{
  std::vector<RoughPlanPoint> points;  //!< N+1 points (t_k = k*dt)
  std::vector<double> s;               //!< same length; s(t_k), so that nothing has to reproject
  Decisions decisions;                 //!< what derive_decisions() returned
  RoughPlanSource source{RoughPlanSource::STOP};
  bool blocked{false};          //!< whether the stop is caused by a safety constraint
  int blocked_first_stage{-1};  //!< when blocked, the first stage that causes it, for the report
};

//! For the report and the markers only; nothing here changes the behavior
struct RoughPlanDebug
{
  std::vector<std::string> rejected;  //!< the rejected candidates and why
  double elapsed_ms{0.0};
  MarkerArray debug_markers;
};

//! What plan_rough_trajectories() returns: the candidates and the debug information
struct RoughPlanResult
{
  std::vector<RoughPlan> plans;
  RoughPlanDebug debug;
};

// ---------------------------------------------------------------------------------------------
// carried across cycles; owned by the node and only read here
// ---------------------------------------------------------------------------------------------

struct PreviousPlanningResult
{
  //! The rough plan taken in the previous cycle, in map coordinates. It is the first candidate of
  //! this cycle and the reference of the hysteresis comparison
  std::optional<RoughPlan> plan;
  //! For how many cycles in a row the previous solution was unusable; past a threshold the side
  //! stops being held on to
  int consecutive_fallbacks{0};
};

// ---------------------------------------------------------------------------------------------
// parameters
// ---------------------------------------------------------------------------------------------

//! Parameters of the layer (ROS namespace `rough_planner.*`), filled in by the node from the
//! generated parameters.
struct RoughPlannerParams
{
  double time_step_s{0.1};  //!< [s] output time grid, the one of the optimizer downstream
  int num_points{101};      //!< N + 1
  int max_candidates{1};    //!< upper bound on the number of candidates; 1 for now

  //! The (s, l, t, v) grid of the DP and its transitions
  struct Dp
  {
    double s_max_m{150.0};   //!< [m] how far ahead the grid reaches
    double s_step_m{2.0};    //!< [m]
    double l_range_m{3.0};   //!< [m] the grid spans this far to each side of the centerline
    double l_step_m{0.5};    //!< [m]
    double t_step_s{1.0};    //!< [s] spacing of the layers
    double horizon_s{10.0};  //!< [s] keep it equal to the T of the optimizer downstream
    double v_step_mps{1.0};  //!< [m/s]

    double lateral_slope_max{0.3};     //!< [-] heading deviation allowed, ~17 deg
    double lateral_rate_max_mps{1.5};  //!< [m/s] upper bound on the lateral speed

    //! Cost weights, scaled so that one meter of progress is worth 1
    struct Weights
    {
      double progress{1.0};       //!< [1/m]
      double lateral{0.5};        //!< [1/(m^2 s)] holding the lateral offset
      double lateral_rate{1.0};   //!< [s/m^2] lateral speed, which keeps the path from zigzagging
      double velocity{0.2};       //!< [s^3/m^2] deviation from the target speed
      double accel{0.1};          //!< [s^5/m^2] smoothing, weakly
      double accel_nominal{2.0};  //!< [s^5/m^2] penalty for leaving the comfortable range
    } weights;
  } dp;
};

// ---------------------------------------------------------------------------------------------
// RoughPlanner
// ---------------------------------------------------------------------------------------------

class RoughPlanner
{
public:
  explicit RoughPlanner(const RoughPlannerParams & params);

  //! Returns the candidates in priority order. There is **always at least one**, the stop plan
  //! being unconditionally feasible. For now only the first one is consumed.
  RoughPlanResult plan_rough_trajectories(
    const PlannerContext & context, const CompiledConstraints & compiled_constraints,
    const PreviousPlanningResult & prev_planning_result) const;

private:
  RoughPlannerParams params_;
};

//! Derives the discrete decisions from the geometry, shared by every rough planner. They are
//! derived again every cycle, and prev_decisions is used only for the hysteresis comparison.
Decisions derive_decisions(
  const RoughPlan & plan, const CompiledConstraints & compiled_constraints,
  const Decisions & prev_decisions);

}  // namespace autoware::safety_planner

#endif  // AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__ROUGH_PLANNER_HPP_
