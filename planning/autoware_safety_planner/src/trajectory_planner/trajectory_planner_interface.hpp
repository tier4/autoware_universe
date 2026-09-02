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

#ifndef TRAJECTORY_PLANNER__TRAJECTORY_PLANNER_INTERFACE_HPP_
#define TRAJECTORY_PLANNER__TRAJECTORY_PLANNER_INTERFACE_HPP_

// 軌道プランナープラグインの共通 IF。SafetyPlanner は制約の生成と certainty ごとの
// 振り分けまでを行い、制約のコンパイル・rough_planner / optimizer の呼び出し方は
// プラグインの実装詳細とする。
// 入力 = 2 つの制約セット (normal / cautious、生の Constraint 列)、出力 = 2 本の軌道

#include "../constraint.hpp"
#include "../context.hpp"
#include "../type_alias.hpp"
#include "constraints_compiler.hpp"
#include "rough_planner.hpp"
#include "trajectory_optimizer_interface.hpp"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

struct TrajectoryPlannerInput
{
  const PlannerContext & context;
  const std::vector<Constraint> & normal_constraints;    //!< certainty = DEFINITE のみ
  const std::vector<Constraint> & cautious_constraints;  //!< DEFINITE + POSSIBLE
};

struct TrajectoryPlannerResult
{
  std::optional<Trajectory> normal_trajectory;
  std::optional<Trajectory> cautious_trajectory;

  //! デバッグは当面 normal 側のパイプラインのみ (cautious 側の可視化は未整備)
  struct Debug
  {
    CompiledConstraints compiled_constraints;  //!< normal 側のコンパイル結果
    RoughPlanResult rough_plan_result;
    TrajectoryOptimizerResult trajectory_optimizer_result;
  } debug;
};

class TrajectoryPlannerInterface
{
public:
  TrajectoryPlannerInterface() = default;
  virtual ~TrajectoryPlannerInterface() = default;

  //! ロード直後に 1 回呼ばれる。内部プラグイン (optimizer 等) のロードは派生側の override で行う
  virtual void on_initialize(
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper, const Params & params)
  {
    time_keeper_ = time_keeper;
    params_ = params;
  }

  virtual std::string get_name() const = 0;
  virtual TrajectoryPlannerResult plan(const TrajectoryPlannerInput & input) = 0;

protected:
  mutable std::shared_ptr<TimeKeeper> time_keeper_{nullptr};
  Params params_;
};

}  // namespace autoware::safety_planner

#endif  // TRAJECTORY_PLANNER__TRAJECTORY_PLANNER_INTERFACE_HPP_
