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

// MPPI trajectory planner plugin. The trajectory of FrenetSamplingBasedPlanner is the reference
// that autoware_mppi_optimizer (FirstOrderDubinsMppiInterface) tracks and refines; MPPI alone has
// no gradient away from moving objects and pulls towards its reference, so the avoidance geometry
// and the stop decisions stay with the sampling planner. The constraints are handed to MPPI as far
// as its API takes them (constraints_compiler.hpp IR -> road_borders / drivable_area segments,
// kinematic limits, tracked objects); whatever it cannot represent (timed occupancy, lateral
// acceleration, soft weights per constraint) is checked on the output, and the Frenet trajectory
// is returned unchanged when the output fails or MPPI rejects it.

#include "../frenet_sampling_based_planner/constraints_compiler.hpp"
#include "../frenet_sampling_based_planner/frenet_sampling_based_planner.hpp"
#include "../trajectory_planner_interface.hpp"

#include <autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

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

  //! Refines reference in place with optimizer; returns false (reference untouched) when MPPI
  //! is skipped, rejects its result, or the result fails the constraint check
  bool refine_one_side(
    MppiInterface & optimizer, const PlannerContext & context,
    const std::vector<Constraint> & constraints, Trajectory & reference,
    TrajectoryPlannerDebug & debug);

  //! The GPU resources are allocated on the first call, once the steer bounds are known
  void ensure_initialized(
    MppiInterface & optimizer, const PlannerContext & context,
    const CompiledConstraints & compiled_constraints);

  bool satisfies_constraints(
    const PlannerContext & context, const CompiledConstraints & compiled_constraints,
    const Trajectory & trajectory, std::string & reason) const;

  FrenetSamplingBasedPlanner frenet_planner_;
  //! One optimizer per output: the warm start (previous control sequence) is internal state
  std::unique_ptr<MppiInterface> normal_optimizer_;
  std::unique_ptr<MppiInterface> cautious_optimizer_;
};

}  // namespace autoware::safety_planner::experiment

#endif  // TRAJECTORY_PLANNER__MPPI_PLANNER__MPPI_PLANNER_HPP_
