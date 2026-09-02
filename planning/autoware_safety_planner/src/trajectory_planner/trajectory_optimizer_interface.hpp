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

#ifndef TRAJECTORY_PLANNER__TRAJECTORY_OPTIMIZER_INTERFACE_HPP_
#define TRAJECTORY_PLANNER__TRAJECTORY_OPTIMIZER_INTERFACE_HPP_

// trajectory_optimizer: rough_planner が決めたホモトピー (どちら側を抜けるか / 譲るか /
// 止まるか) の中で、制約を満たす精緻な軌道を 1 本作る層。**非凸探索はしない**。
//
// 定式化を 2 通り試すため (docs/safety_planner_arch_design/formulation_ssc_vs_poc.md)、
// この層は pluginlib のプラグインとして実装し、パラメータ `trajectory_optimizer.plugin`
// で切り替える。制約ジェネレーターと同じ枠組み (constraint_generator_interface.hpp)。
//
// | プラグイン | 決定変数 | 最適化の型 | 出典 |
// |---|---|---|---|
// | NlpTrajectoryOptimizer   | 状態 (px,py,θ,κ,v,a) と入力 (w,j) の時系列 | 非凸 NLP |
//                              PoC (docs/spec/3_trajectory_optimizer/trajectory_optimizer.md)
// | SscQpTrajectoryOptimizer | Frenet s(t), l(t) の区分 Bézier 制御点 | 凸 QP |
//                              SSC (arXiv:1906.09788)
//
// 入出力の規約:
// - 時間グリッドは rough_plan と同一 (t_k = k·dt)。格子変換の曖昧さを持たない
//   (rough_planner.hpp の RoughPlanPoint と同じ規約)
// - 座標は世界座標 (planning frame)。Frenet を使う定式化でも、戻り値は世界座標へ戻す
// - 制約の正は compiled_constraints.raw_constraints。射影ビューは粗い評価用で、
//   精密評価 (コリドー彫り込み・検証) は raw を読む (constraints_compiler.hpp)

#include "../constraint.hpp"
#include "../context.hpp"
#include "../type_alias.hpp"
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
// 出力型
// ---------------------------------------------------------------------------------------------

//! 最適化された軌道の 1 点。状態 (px, py, θ, κ, v, a) と入力 (w, j)。
//! 入力は「その点から次の点へ進む間」の値で、終端点 (k = N) では 0
struct OptimizedTrajectoryPoint
{
  double t{0.0};      //!< [s] 計画基準時刻からの相対
  Pose2d pose{};      //!< 世界座標 (px, py, θ)
  double kappa{0.0};  //!< [1/m] 経路曲率 κ
  double v{0.0};      //!< [m/s]
  double a{0.0};      //!< [m/s²]
  double w{0.0};      //!< [1/(m·s)] dκ/dt
  double j{0.0};      //!< [m/s³]  da/dt
};

struct OptimizedTrajectory
{
  std::vector<OptimizedTrajectoryPoint> points;  //!< N+1 点 (t_k = k·dt)
};

//! optimize() の結果種別。SUCCESS 以外は consumer 側 (RuleBasedPlanner) が
//! フォールバック (rough_plan をそのまま出す等) を選ぶ
enum class TrajectoryOptimizerStatus : std::uint8_t {
  SUCCESS,          //!< 解が得られ、検証を通った
  INFEASIBLE,       //!< 制約を満たす解が無い (フォールバックを尽くしても検証に落ちた)
  SOLVER_ERROR,     //!< ソルバーの内部エラー・NaN・予算超過
  NOT_IMPLEMENTED,  //!< 未実装 (スケルトン段階)
};

//! report・マーカー用。意味論には関与しない
struct TrajectoryOptimizerDebug
{
  std::string message;  //!< 失敗理由・落とした tier などの人間可読な説明
  double elapsed_ms{0.0};
  int which_level{0};            //!< 何段目のフォールバックで成立したか (0 = 未実施)
  bool used_certificate{false};  //!< 最終手段の停止軌道を返したか
  MarkerArray debug_markers;
};

struct TrajectoryOptimizerResult
{
  OptimizedTrajectory trajectory;  //!< status == SUCCESS のときのみ有効
  TrajectoryOptimizerStatus status{TrajectoryOptimizerStatus::NOT_IMPLEMENTED};
  TrajectoryOptimizerDebug debug;
};

// ---------------------------------------------------------------------------------------------
// 入力型
// ---------------------------------------------------------------------------------------------

//! optimize() の入力一式。参照の寿命は 1 回の optimize() 呼び出しの間だけ
//! (プラグインは周期を越えて保持しない)
struct TrajectoryOptimizerInput
{
  const PlannerContext & context;
  //! 制約 IR。raw が唯一の正で、射影ビューは粗い評価用 (constraints_compiler.hpp)
  const CompiledConstraints & compiled_constraints;
  //! rough_planner が選んだホモトピー。追従参照 (p, θ, v_ref) と初期値を兼ねる
  const RoughPlan & rough_plan;
  //! 前周期に採用した最適化結果 (warm start 用)。初回・前周期が失敗した周期は nullopt
  const std::optional<OptimizedTrajectory> & prev_trajectory;
};

// ---------------------------------------------------------------------------------------------
// プラグイン基底
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

  //! rough_plan のホモトピーを保ったまま、制約を満たす軌道を 1 本作る
  virtual TrajectoryOptimizerResult optimize(const TrajectoryOptimizerInput & input) = 0;

protected:
  mutable std::shared_ptr<TimeKeeper> time_keeper_{nullptr};
  Params params_;
};

}  // namespace autoware::safety_planner

#endif  // TRAJECTORY_PLANNER__TRAJECTORY_OPTIMIZER_INTERFACE_HPP_
