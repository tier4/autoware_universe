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

#ifndef TRAJECTORY_PLANNER__ROUGH_PLANNER_HPP_
#define TRAJECTORY_PLANNER__ROUGH_PLANNER_HPP_

// rough_planner: ホモトピー選択 (どちら側を抜けるか / 譲るか先に行くか / 止まるか通るか) の
// 全責任を持つ層。下流の optimizer は非凸探索をしない。
//
// 設計の出典 (docs/safety_planner_arch_design/homotopy_resolution.md, v0 spec S3):
// - rough_planner は**無状態**。周期間の持ち越し (PreviousPlanningResult) の所有は Node で、
//   const 参照で受け取るだけ。次周期分は Node が出力から書き戻す
// - 離散決定 (Decisions) は持ち越さず、**毎周期軌道の幾何から再導出**する。
//   持ち越した Decisions はヒステリシス比較にのみ使う。キーは周期間で安定な ID
//   (Source の ID)。s や配列添字に紐づけない
// - ヒステリシスは二層: 第一層 = 編成 (前周期解が通る限り決定は変わらない)、
//   第二層 = 危険側への切替は無条件・安全余裕を減らす側の切替には障壁
// - 出力は**優先順位付きの候補列**。v0 の「最初に成立した 1 本」は早期 commit が弱点
//   (homotopy_resolution.md 論点 2) なので、IF は top-K を許す形にしておく。
//   当面の運用は K = 1 (consumer は先頭のみ消費)

#include "../constraint.hpp"
#include "../context.hpp"
#include "constraints_compiler.hpp"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace autoware::safety_planner
{

// ---------------------------------------------------------------------------------------------
// 離散決定
// ---------------------------------------------------------------------------------------------

enum class LeadLag : std::uint8_t { LEAD, FOLLOW };
enum class StopGo : std::uint8_t { STOP, GO };

//! ホモトピーを定める離散決定。キーは周期間で安定な ID (Source::target_id: perception UUID /
//! lanelet id 等)。毎周期 derive_decisions() で軌道の幾何から再導出する
struct Decisions
{
  std::map<std::string, Side> side;         //!< 静的物体ごとの左右
  std::map<std::string, LeadLag> lead_lag;  //!< 動的物体ごとの先行/追従
  std::map<std::string, StopGo> stop_go;    //!< 停止線・譲り対象ごとの停止/通過
};

// ---------------------------------------------------------------------------------------------
// 出力型
// ---------------------------------------------------------------------------------------------

enum class RoughPlanSource : std::uint8_t {
  PREVIOUS_SOLUTION,  //!< 前周期解の再利用
  SPATIOTEMPORAL_DP,  //!< 時空間 DP
  REFERENCE_FOLLOW,   //!< 中心線追従 (幾何制約を見ない暫定実装。DP 実装後に置き換える)
  STOP,               //!< 停止 rough_plan (最終手段、無条件成立)
};

//! 粗軌道の 1 点。時間グリッドは下流 optimizer のステージと同一 (t_k = k·dt) にし、
//! 格子変換の曖昧さを持たない
struct RoughPlanPoint
{
  double t{0.0};      //!< [s]
  Pose2d pose{};      //!< 世界座標
  double kappa{0.0};  //!< [1/m] optimizer 状態の初期値として rough_planner 層が埋める
  double v{0.0};      //!< [m/s]
  double a{0.0};      //!< [m/s²]
};

struct RoughPlan
{
  std::vector<RoughPlanPoint> points;  //!< N+1 点 (t_k = k·dt)
  std::vector<double> s;               //!< 同長。reference_path 弧長 s(t_k)。再投影の曖昧さ排除用
  Decisions decisions;                 //!< derive_decisions() の結果
  RoughPlanSource source{RoughPlanSource::STOP};
  bool blocked{false};          //!< 停止の原因が SAFETY 障害 (raw への逆参照で判定)
  int blocked_first_stage{-1};  //!< blocked 時、停止の原因となる最初のステージ (report 用)
};

//! report・マーカー用。意味論には関与しない
struct RoughPlanDebug
{
  std::vector<std::string> rejected;  //!< 不成立候補とその理由
  double elapsed_ms{0.0};
  MarkerArray debug_markers;
};

//! plan_rough_trajectories() の戻り値。候補列とデバッグ情報を束ねる
struct RoughPlanResult
{
  std::vector<RoughPlan> plans;
  RoughPlanDebug debug;
};

// ---------------------------------------------------------------------------------------------
// 周期間持ち越し (所有は Node。rough_planner は const 参照で読むだけ)
// ---------------------------------------------------------------------------------------------

struct PreviousPlanningResult
{
  //! 前周期に採用された rough_plan (地図座標)。編成の第一候補 (前周期解の再利用) と
  //! ヒステリシス比較 (decisions) に使う
  std::optional<RoughPlan> plan;
  //! 前周期解が使えなかった連続周期数。閾値超過で side 固執を解く (ヒステリシス材料)
  int consecutive_fallbacks{0};
};

// ---------------------------------------------------------------------------------------------
// パラメータ
// ---------------------------------------------------------------------------------------------

//! rough_planner 層のパラメータ (ROS ns `rough_planner.*`。S3 §7)。
//! 既定値は S3 の初期値表。Node が生成パラメータから詰めて渡す
struct RoughPlannerParams
{
  double time_step_s{0.1};  //!< [s] 出力時間グリッド (下流 optimizer のステージと同一)
  int num_points{101};      //!< N + 1
  int max_candidates{1};    //!< 出力候補数の上限 (当面 1。top-K 比較は将来拡張)

  //! (s, l, t, v) DP 格子と遷移条件 (S3 §3.1 / §3.3)
  struct Dp
  {
    double s_max_m{150.0};   //!< [m] 前方範囲
    double s_step_m{2.0};    //!< [m]
    double l_range_m{3.0};   //!< [m] 中心線から ± この範囲
    double l_step_m{0.5};    //!< [m]
    double t_step_s{1.0};    //!< [s] 層間隔
    double horizon_s{10.0};  //!< [s] 下流 optimizer の T と一致させる
    double v_step_mps{1.0};  //!< [m/s] v 軸刻み

    double lateral_slope_max{0.3};     //!< [-] 参照からの許容ヘディング偏差 (≈ 17°)
    double lateral_rate_max_mps{1.5};  //!< [m/s] 横速度上限

    //! コスト重み (S3 §3.4。「進行 1 m の価値 = 1」の等価換算)
    struct Weights
    {
      double progress{1.0};       //!< [1/m]
      double lateral{0.5};        //!< [1/(m²·s)] 横オフセット維持
      double lateral_rate{1.0};   //!< [s/m²] 横速度 (格子ジグザグ抑制)
      double velocity{0.2};       //!< [s³/m²] 目標速度からの乖離
      double accel{0.1};          //!< [s⁵/m²] 平滑化 (弱く)
      double accel_nominal{2.0};  //!< [s⁵/m²] 快適域超過の抑制
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

  //! 優先順位付きの rough_plan 候補列を返す。plans は**必ず 1 本以上** (停止 rough_plan が
  //! 無条件成立)。当面 consumer は先頭のみ消費する (top-K 比較は将来拡張)
  RoughPlanResult plan_rough_trajectories(
    const PlannerContext & context, const CompiledConstraints & compiled_constraints,
    const PreviousPlanningResult & prev_planning_result) const;

private:
  RoughPlannerParams params_;
};

//! 全 rough_planner 共通の離散決定の幾何導出。決定は毎周期ここで再導出し、
//! prev_decisions はヒステリシス比較にのみ使う
Decisions derive_decisions(
  const RoughPlan & plan, const CompiledConstraints & compiled_constraints,
  const Decisions & prev_decisions);

}  // namespace autoware::safety_planner

#endif  // TRAJECTORY_PLANNER__ROUGH_PLANNER_HPP_
