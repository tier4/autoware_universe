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

#ifndef TRAJECTORY_PLANNER__NLP_PLANNER__CONSTRAINTS_COMPILER_HPP_
#define TRAJECTORY_PLANNER__NLP_PLANNER__CONSTRAINTS_COMPILER_HPP_

// 制約のコンパイル: プラグイン出力 (世界座標の Constraint 列) を、消費側が読める
// IR = CompiledConstraints に変換する。
//
// IR は 2 層構造 (docs/safety_planner_arch_design/constraint_desing.md の続きの設計):
// - raw: フラット化した全 Constraint。世界座標のまま。**これが唯一の正**
// - 射影ビュー (scalar_bounds / lateral_bounds / stop_bars / occupancies):
//   raw を context.reference_path へ射影した導出データ。粗い consumer (rough_planner の
//   DP 等) 用で、raw_index で raw を逆参照する
//
// 射影の規約:
// - 弧長 s の基準は **その周期の context.reference_path 1 本だけ**。始点 (s = 0) は
//   reference_path の後方端 (ego の backward_length_m 後方)、終端は goal。
//   consumer は各自でルート基準を作らない
// - 射影ビューは DP の格子解像度と同程度の誤差まで**保守側**に丸めてよい。
//   ビューで通って raw で落ちるのは許す (外側の反復・フォールバックが扱う) が、
//   過保守で解を消す丸めは警告なしに行わない
// - Frenet 射影が一意なのは中心線の曲率半径の範囲内だけ。多価・不定になる要素は
//   ビューから落として unprojected に載せる (raw には残るので精密評価では効く)
// - 点の射影は**最近傍の足**で決める。中心線の折れ点では隣り合う区間の垂線帯が一致せず、
//   曲がりの外側に幅 |l·Δθ| の楔形の隙間が開くが、そこに落ちた点も足を折れ点へ
//   クランプして必ず受け止める。落とすと、境界の頂点が中心線サンプルに揃っている
//   生成器 (simple_drivable_area) では外側の頂点が丸ごと消え、その s 範囲の横制約が
//   無くなる = fail-open になる (docs/issues/85)
// - IR は毎周期使い捨て。s に周期間の意味を持たせない (周期間の照合は Source の ID で行う)

#include "../../constraint.hpp"
#include "../../context.hpp"

#include <cstddef>
#include <vector>

namespace autoware::safety_planner
{

//! (s, l) 座標の点。s は reference_path 弧長 [m]、l は中心線からの横オフセット [m] (左が正)
struct SlPoint
{
  double s{0.0};
  double l{0.0};
};

//! ScalarBound の射影。region 有りは region × 中心線の交差から得た弧長区間、
//! region 無しは全区間 (s0 = -INF, s1 = +INF)。region が複数区間で交わる場合は区間ごとに 1 件
struct ScalarBoundEntry
{
  BoundedQuantity quantity{BoundedQuantity::VELOCITY};
  double s0{-INF};  //!< [m] 有効弧長区間 (閉区間)
  double s1{+INF};
  double min{-INF};
  double max{+INF};
  std::size_t raw_index{0};
};

//! Boundary の射影。折れ線を (s, l) へ落としたサンプル列 (s 昇順)。
//! 左右の包絡 (l_min(s) / l_max(s)) の合成は consumer の仕事
//! (ここでは境界 1 本 = 1 エントリのまま保つ)
struct LateralBoundEntry
{
  std::vector<SlPoint> polyline;    //!< s 昇順
  Side forbidden_side{Side::LEFT};  //!< 元の折れ線の進行向き基準 (raw と同じ)
  double margin{0.0};               //!< [m] raw の margin の写し
  std::size_t raw_index{0};
};

//! Gate の射影。線分 × 中心線の交点の弧長。交点が無い Gate はビューに現れない (unprojected 行き)
struct StopBarEntry
{
  double s_stop{0.0};  //!< [m] この s から先が進入禁止 (footprint 前端で評価)
  TimeWindow time{};   //!< raw の有効時間帯の写し
  double margin{0.0};  //!< [m] raw の margin の写し
  std::size_t raw_index{0};
};

//! KeepOut の占有を時間スラブごとに (s, l) の範囲へ保守側に丸めたもの。
//! スラブ内の占有 = スラブ両端時刻の補間形状の和を外接する (s, l) box。
//! margin は焼き込まない。consumer が raw の KeepOut::margin_m で膨張する
struct OccupancySlab
{
  double t0{0.0};  //!< [s] スラブ時間帯
  double t1{0.0};
  double s0{0.0};  //!< [m] 占有の弧長範囲
  double s1{0.0};
  double l0{0.0};  //!< [m] 占有の横オフセット範囲
  double l1{0.0};
};

struct OccupancyEntry
{
  std::vector<OccupancySlab> slabs;  //!< t 昇順
  std::size_t raw_index{0};
};

//! コンパイル済み制約 (IR)。raw が唯一の正、ビューは射影の導出データ
struct CompiledConstraints
{
  //! フラット化した全制約 (世界座標のまま)。精密評価 (コリドー彫り・検証) はこちらを読む
  std::vector<Constraint> raw_constraints;

  // ---- 射影ビュー (粗い consumer 用。各要素は raw_index で raw を逆参照する) ----
  std::vector<ScalarBoundEntry> scalar_bounds;
  std::vector<LateralBoundEntry> lateral_bounds;
  std::vector<StopBarEntry> stop_bars;
  std::vector<OccupancyEntry> occupancies;

  //! 射影できなかった raw の添字 (中心線と交わらない Gate、射影が多価になる幾何、不正 IR 等)。
  //! 診断・デバッグマーカーに載せる。raw には残っているので精密評価では効く
  std::vector<std::size_t> unprojected;
};

CompiledConstraints compile_constraint_list(
  const PlannerContext & context, const std::vector<Constraint> & constraints);

}  // namespace autoware::safety_planner

#endif  // TRAJECTORY_PLANNER__NLP_PLANNER__CONSTRAINTS_COMPILER_HPP_
