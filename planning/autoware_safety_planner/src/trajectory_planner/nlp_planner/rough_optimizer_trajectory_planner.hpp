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

#ifndef AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__ROUGH_OPTIMIZER_TRAJECTORY_PLANNER_HPP_
#define AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__ROUGH_OPTIMIZER_TRAJECTORY_PLANNER_HPP_

// The default trajectory planner plugin. For each constraint set it runs the rough planner, which
// settles the homotopy with a space-time DP, followed by the trajectory optimizer, and so produces
// the normal and the cautious trajectory. The optimizer is a pluginlib plugin, selected by
// `trajectory_optimizer.plugin`.

#include "../trajectory_planner_interface.hpp"
#include "rough_planner.hpp"
#include "trajectory_optimizer_interface.hpp"

#include <memory>
#include <optional>
#include <string>

namespace autoware::safety_planner
{

class RoughOptimizerTrajectoryPlanner : public TrajectoryPlannerInterface
{
public:
  void on_initialize(
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    const Params & params) override;

  std::string get_name() const override { return "rough_optimizer"; }

  TrajectoryPlannerResult plan(const TrajectoryPlannerInput & input) override;

private:
  //! Carried across cycles. The normal and the cautious side see different constraints and reach
  //! different solutions, so each keeps its own
  struct SideState
  {
    PreviousPlanningResult prev_planning_result;
    std::optional<OptimizedTrajectory> prev_optimized_trajectory;
  };

  //! Runs the rough planner and the optimizer for one constraint set; nullopt unless it succeeds
  std::optional<Trajectory> plan_one_side(
    const PlannerContext & context, const CompiledConstraints & compiled_constraints,
    SideState & state, RoughPlanResult & rough_plan_result,
    TrajectoryOptimizerResult & optimizer_result);

  void load_trajectory_optimizer_plugin();

  TrajectoryOptimizerResult optimize_trajectory(
    const PlannerContext & context, const CompiledConstraints & compiled_constraints,
    const RoughPlanResult & rough_plan_result, const SideState & state) const;

  std::optional<RoughPlanner> rough_planner_;

  using TrajectoryOptimizerLoader = pluginlib::ClassLoader<TrajectoryOptimizerInterface>;
  //! Must outlive the loaded instance: destroying the loader unloads it
  std::unique_ptr<TrajectoryOptimizerLoader> trajectory_optimizer_loader_;
  std::shared_ptr<TrajectoryOptimizerInterface> trajectory_optimizer_;

  SideState normal_state_;
  SideState cautious_state_;
};

}  // namespace autoware::safety_planner

#endif  // AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__ROUGH_OPTIMIZER_TRAJECTORY_PLANNER_HPP_
