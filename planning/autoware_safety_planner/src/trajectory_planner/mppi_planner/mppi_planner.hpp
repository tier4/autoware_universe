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

#ifndef TRAJECTORY_PLANNER__MPPI_PLANNER__MPPI_PLANNER_HPP_
#define TRAJECTORY_PLANNER__MPPI_PLANNER__MPPI_PLANNER_HPP_

// MPPI trajectory planner plugin. The reference that autoware_mppi_optimizer
// (FirstOrderDubinsMppiInterface) tracks is the reference_path itself, sampled in time with a
// longitudinal profile built from the constraints (speed bounds, curvature, stop bars, goal); the
// lateral planning is left to MPPI. MPPI reads the reference by index, one point per kMppiDt, so a
// path alone cannot be handed over. The constraints are passed to MPPI as far as its API takes
// them (constraints_compiler.hpp IR -> road_borders / drivable_area segments, kinematic limits,
// tracked objects); whatever it cannot represent (timed occupancy, lateral acceleration, soft
// weights per constraint) is checked on the output. There is no planner behind this one: when
// the output fails or MPPI rejects it, a trajectory of the ego point alone is returned.

#include "../../utils/boundary_simplifier.hpp"
#include "../../utils/turn_indicator_decider.hpp"
#include "../frenet_sampling_based_planner/constraints_compiler.hpp"
#include "../trajectory_planner_interface.hpp"

#include <autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::safety_planner
{
class PathProjector;
}  // namespace autoware::safety_planner

namespace autoware::safety_planner::experiment
{

class MppiPlanner : public TrajectoryPlannerInterface
{
public:
  std::string get_name() const override { return "mppi_planner"; }

  void on_initialize(
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    const Params & params) override;

  TrajectoryPlannerResult plan_trajectories(const TrajectoryPlannerInput & input) override;

private:
  using MppiInterface = autoware::mppi_optimizer::FirstOrderDubinsMppiInterface;

  PlannedTrajectory plan_one_side(
    MppiInterface & optimizer, TurnIndicatorDecider & turn_indicator_decider,
    const PlannerContext & context, const std::vector<Constraint> & constraints,
    TrajectoryPlannerDebug & debug);

  //! The ego at t = 0 followed by the reference_path centerline, one point per kMppiDt over the
  //! MPPI horizon, driven at the fastest profile under the speed bounds, the lateral acceleration,
  //! the steer rate, the stop bars and the goal
  Trajectory make_reference_trajectory(
    const PlannerContext & context, const CompiledConstraints & compiled_constraints) const;

  //! Refines reference in place with optimizer; returns the failure reason (reference untouched)
  //! when MPPI rejects its result or the result fails the constraint check
  std::optional<std::string> refine(
    MppiInterface & optimizer, const PlannerContext & context,
    const CompiledConstraints & compiled_constraints, const PathProjector & projector,
    Trajectory & reference, TrajectoryPlannerDebug & debug);

  //! The GPU resources are allocated on the first call, once the steer bounds are known
  void ensure_initialized(
    MppiInterface & optimizer, const PlannerContext & context,
    const CompiledConstraints & compiled_constraints);

  bool satisfies_constraints(
    const PlannerContext & context, const CompiledConstraints & compiled_constraints,
    const PathProjector & projector, const Trajectory & trajectory, std::string & reason) const;

  // One decider per output, since each holds its own anti-chatter and latch state
  TurnIndicatorDecider normal_turn_indicator_decider_{TurnSignalParams{}};
  TurnIndicatorDecider cautious_turn_indicator_decider_{TurnSignalParams{}};
  //! One optimizer per output: the warm start (previous control sequence) is internal state
  std::unique_ptr<MppiInterface> normal_optimizer_;
  std::unique_ptr<MppiInterface> cautious_optimizer_;
  std::unique_ptr<BoundarySimplifier> boundary_simplifier_;
};

}  // namespace autoware::safety_planner::experiment

#endif  // TRAJECTORY_PLANNER__MPPI_PLANNER__MPPI_PLANNER_HPP_
