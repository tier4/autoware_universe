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

#ifndef TRAJECTORY_PLANNER__NLP_PLANNER__ROUGH_OPTIMIZER_TRAJECTORY_PLANNER_HPP_
#define TRAJECTORY_PLANNER__NLP_PLANNER__ROUGH_OPTIMIZER_TRAJECTORY_PLANNER_HPP_

// 既定の軌道プランナープラグイン。制約セットごとに
// rough_planner (時空間 DP によるホモトピー解決) → trajectory optimizer (精緻化) を回し、
// normal / cautious の 2 本の軌道を作る。optimizer は pluginlib で差し替え可能
// (`trajectory_optimizer.plugin`)

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
  //! 周期間持ち越し。normal / cautious は制約セットが違い解も別物なので、別々に持つ
  struct SideState
  {
    PreviousPlanningResult prev_planning_result;
    std::optional<OptimizedTrajectory> prev_optimized_trajectory;
  };

  //! 1 つの制約セットについて rough → optimize を回す。SUCCESS でなければ nullopt
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
  //! ロード済みインスタンスより長生きさせること (unload はローダの破棄で起こる)
  std::unique_ptr<TrajectoryOptimizerLoader> trajectory_optimizer_loader_;
  std::shared_ptr<TrajectoryOptimizerInterface> trajectory_optimizer_;

  SideState normal_state_;
  SideState cautious_state_;
};

}  // namespace autoware::safety_planner

#endif  // TRAJECTORY_PLANNER__NLP_PLANNER__ROUGH_OPTIMIZER_TRAJECTORY_PLANNER_HPP_
