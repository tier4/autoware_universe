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

#ifndef SAFETY_PLANNER_HPP_
#define SAFETY_PLANNER_HPP_

// パイプライン本体 (ロジック層)。ROS インターフェース (購読・配信・タイマ) は
// SafetyPlannerNode が持ち、このクラスは PlannerContext を受け取って結果を返すだけ。
// publisher・clock を持たない (msg 型と TimeKeeper・pluginlib は許容)。
// 制約の生成と certainty ごとの振り分けまでがこのクラスの仕事で、制約のコンパイルと
// rough_planner / optimizer の呼び出しは trajectory_planner プラグインが行う

#include "constraint_generator/constraint_generator_interface.hpp"
#include "context.hpp"
#include "trajectory_planner/trajectory_planner_interface.hpp"
#include "type_alias.hpp"

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::safety_planner
{

struct SafetyPlannerResult
{
  std::optional<Trajectory> normal_trajectory;
  std::optional<Trajectory> cautious_trajectory;

  struct Debug
  {
    //! プラグイン名 (get_name()) → 出力。debug_markers の publish は Node の仕事
    std::map<std::string, ConstraintGeneratorOutput> constraint_generator_outputs;
    CompiledConstraints compiled_constraints;  //!< normal 側 (cautious 側の可視化は未整備)
    RoughPlanResult rough_plan_result;
    TrajectoryOptimizerResult trajectory_optimizer_result;
  } debug;
};

class SafetyPlanner
{
public:
  SafetyPlanner(const Params & params, std::shared_ptr<TimeKeeper> time_keeper);

  SafetyPlannerResult plan(const PlannerContext & context);

  //! ロード済みプラグインの名前 (get_name()) のリスト。Node が debug marker publisher
  //! を作るのに使う
  std::vector<std::string> get_constraint_generator_plugin_names() const;

  //! ロード済みの軌道プランナープラグインの名前 (get_name())。未ロードなら空文字
  std::string get_trajectory_planner_plugin_name() const;

private:
  Params params_;
  std::shared_ptr<TimeKeeper> time_keeper_;

  /**
   ***********************************************************
   * @defgroup Constraint plugins
   * @{
   */

  void load_constraint_generator_plugins();

  std::map<std::string, ConstraintGeneratorOutput> calculate_constraints(
    const PlannerContext & context);

  using ConstraintGeneratorLoader = pluginlib::ClassLoader<ConstraintGeneratorInterface>;
  //! ロード済みインスタンスより長生きさせること (unload はローダの破棄で起こる)
  std::unique_ptr<ConstraintGeneratorLoader> constraint_generator_loader_;
  std::vector<std::shared_ptr<ConstraintGeneratorInterface>> constraint_generator_plugins_;

  /** @* */

private:
  /**
   ***********************************************************
   * @defgroup Trajectory planning
   * @{
   */

  void load_trajectory_planner_plugin();

  using TrajectoryPlannerLoader = pluginlib::ClassLoader<TrajectoryPlannerInterface>;
  std::unique_ptr<TrajectoryPlannerLoader> trajectory_planner_loader_;
  std::shared_ptr<TrajectoryPlannerInterface> trajectory_planner_;

  /** @* */
};

}  // namespace autoware::safety_planner

#endif  // SAFETY_PLANNER_HPP_
