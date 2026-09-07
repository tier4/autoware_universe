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

#ifndef TRAJECTORY_PLANNER__NLP_PLANNER__SSC_CORRIDOR_HPP_
#define TRAJECTORY_PLANNER__NLP_PLANNER__SSC_CORRIDOR_HPP_

// Spatio-temporal Semantic Corridor の彫り込み (SSC 論文 Algorithm 1)。
//   seed 生成 → cube inflation → 制約の紐付け
//
// seed は rough_plan の (s(t), l(t))。**ホモトピー (どちら側を抜けるか) は seed が決めており、
// cube はそれを保存したまま自由空間を最大限モデル化する**のが役割で、彫り込み自身は
// 非凸探索をしない。
//
// 座標は reference_path 基準の Frenet (s, l) + 時間 t。cube が縛るのは**後軸基準点**で、
// 車両形状は障害物側を footprint 分だけ膨らませて吸収する (SSC は ego を点として扱う)。
//
// 時間分割は固定 (区間長を最適化変数にすると非凸になる) なので、
// **現 cube の t 上限 = 次 cube の t 下限**。重なりは持たせない。

#include "../../context.hpp"
#include "../../utils/sl_view_utils.hpp"
#include "constraints_compiler.hpp"
#include "rough_planner.hpp"

#include <vector>

namespace autoware::safety_planner
{

//! (s, l, t) の直方体。1 つが軌道の 1 区間に対応する
struct SemanticCube
{
  double t0{0.0};  //!< [s]
  double t1{0.0};
  double s0{0.0};  //!< [m] reference_path 弧長 (後軸基準)
  double s1{0.0};
  double l0{0.0};  //!< [m] 中心線からの横オフセット (左が正、後軸基準)
  double l1{0.0};
};

//! 彫り込みのパラメータ (ROS ns `trajectory_optimizer.ssc_qp.*`)
struct SscCorridorParams
{
  double cube_duration_s{1.0};                //!< [s] 1 cube の時間長 α
  double margin_m{0.1};                       //!< [m] 障害物・境界へ足す安全マージン
  double inflation_step_m{0.2};               //!< [m] 膨張の 1 ステップ
  double max_lateral_inflation_m{4.0};        //!< [m] seed から片側へ膨らませる上限
  double max_longitudinal_inflation_m{20.0};  //!< [m] 同 (s 方向)
};

//! seed (rough_plan) の 1 点を (s, l, t) へ落としたもの
struct CorridorSeedPoint
{
  double t{0.0};
  double s{0.0};
  double l{0.0};
};

//! rough_plan を reference_path の Frenet 座標へ落とす (s は rough_plan が持つ値をそのまま使い、
//! l だけ世界座標から射影する)
std::vector<CorridorSeedPoint> make_corridor_seed(
  const PlannerContext & context, const RoughPlan & rough_plan);

//! seed をホモトピーごと包む cube 列を彫る。
//! - 時間分割は [0, T] を cube_duration_s で等分 (T = seed の終端時刻)
//! - 各 cube は「その時間帯の seed を包む最小 box」から出発し、4 面を交互に膨らませる
//! - 膨らませる先が制約 (境界の禁止側・占有・停止線) に当たったら、その面はそこで止める
//!
//! **seed 自身が制約を破っている場合は空を返して棄却する** (SSC Algorithm 1)。
//! SSC が彫れるのは「既に衝突フリーな seed の周りの自由空間」だけで、塞がれた seed を
//! 縮めて助けることはできない (縮めるとホモトピー = 上流の決定が壊れる)。
//! 塞がれている周期に停止 rough_plan を出すのは rough_planner の仕事 (S3)
std::vector<SemanticCube> generate_semantic_corridor(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const std::vector<CorridorSeedPoint> & seed, const SscCorridorParams & params);

//! 後軸基準の box が (s, l, t) 空間で制約に触れていないか。cube の膨張判定そのもの
bool is_cube_free(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const SemanticCube & cube, double margin_m);

//! cube 列をデバッグマーカーへ (s, l) → 世界座標で落とす
MarkerArray make_corridor_markers(
  const PlannerContext & context, const std::vector<SemanticCube> & cubes, double z_base);

}  // namespace autoware::safety_planner

#endif  // TRAJECTORY_PLANNER__NLP_PLANNER__SSC_CORRIDOR_HPP_
