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

#ifndef TRAJECTORY_PLANNER__SSC_QP_TRAJECTORY_OPTIMIZER_HPP_
#define TRAJECTORY_PLANNER__SSC_QP_TRAJECTORY_OPTIMIZER_HPP_

#include "ssc_corridor.hpp"
#include "trajectory_optimizer_interface.hpp"

#include <optional>
#include <string>
#include <vector>

namespace autoware::safety_planner
{

// Spatio-temporal Semantic Corridor (SSC) の凸 QP による軌道最適化。
// 定式化は docs/safety_planner_arch_design/formulation_ssc_vs_poc.md §2
// (原典: arXiv:1906.09788、要約は同 ref/summary_1906.09788_ssc.md)。
//
// 意味制約を (s, l, t) の cube 列へ落とし、各 cube に区分 5 次 Bézier の 1 区間を割り当てて
// s(t), l(t) の制御点を単一の QP で解く。Bernstein 基底の
//   (P1) 凸包性       … 制御点を箱に入れれば曲線全体が箱に入る
//   (P2) hodograph 性 … 微分の制御点は元の制御点の線形写像
// により、自由空間と縦横の速度・加速度が制御点についての**線形不等式**になる。
// 目的関数 (jerk の二乗積分) は制御点についての二次形式。→ 凸 QP。
//
// この十分条件のおかげで、制約充足が**標本点ではなく区間全体**で成り立つ。
// NLP 版との違いは姿勢 θ と曲率 κ が変数に無いこと (κ は微分の比なので線形制約に書けない)。
// ego は点として扱い、車両形状は cube 側 (障害物の膨張) で吸収する。

//! QP のパラメータ (ROS ns `trajectory_optimizer.ssc_qp.*`)
struct SscQpParams
{
  SscCorridorParams corridor;

  double lateral_rate_max_mps{2.0};    //!< [m/s]  |l̇| 上限 (P2 を k=1 に適用)
  double lateral_accel_max_mps2{2.0};  //!< [m/s²] |l̈| 上限 (P2 を k=2 に適用)

  double weight_jerk_s{1.0};    //!< 縦 jerk 二乗積分の重み
  double weight_jerk_l{1.0};    //!< 横 jerk 二乗積分の重み
  double regularization{1e-8};  //!< P の対角に足す正則化 (jerk Hessian は 2 次以下が零空間)

  double osqp_eps_abs{1e-5};
};

//! QP を解いた結果の生の値 (制御点)。cube 1 つあたり s / l 各 6 点
struct SscQpSolution
{
  //! [cube0 の s 6 点, cube0 の l 6 点, cube1 の s 6 点, ...] の順に並べた制御点
  std::vector<double> control_points;
  double alpha{0.0};  //!< [s] 区間長 (全 cube 共通)
};

class SscQpTrajectoryOptimizer : public TrajectoryOptimizerInterface
{
public:
  std::string get_name() const override { return "ssc_qp"; }

  TrajectoryOptimizerResult optimize(const TrajectoryOptimizerInput & input) override;

private:
  //! ROS パラメータを読む (on_initialize 済みであること)
  SscQpParams read_params() const;
};

// ---------------------------------------------------------------------------------------------
// 以下は plugin 外からもテストできるよう自由関数にしてある
// ---------------------------------------------------------------------------------------------

//! Frenet の初期・終端状態 (位置・速度・加速度)。QP の等式制約に入る
struct SscBoundaryState
{
  double s{0.0};
  double s_dot{0.0};
  double s_ddot{0.0};
  double l{0.0};
  double l_dot{0.0};
  double l_ddot{0.0};
};

//! rough_plan の 1 点 (世界座標の v, a と姿勢) を Frenet の (ṡ, l̇, s̈, l̈) へ落とす。
//! 参照曲率による縮尺 (1 − κ_ref·l) を掛けた小偏差近似で、加速度は向きの変化を無視する
SscBoundaryState to_frenet_boundary_state(
  const PlannerContext & context, const RoughPlanPoint & point, double s, double l);

//! cube の s 区間に効く縦速度の上限 [m/s]。IR の区間付き VELOCITY 行と、参照曲率からの
//! 横加速度上限 √(a_lat_nom/|κ|) の両方を掛ける。曲率は SSC の変数に無いので、
//! cube ごとの ṡ 上限という「意味境界」に落とすしかない (formulation_ssc_vs_poc.md §2.6)
double cube_velocity_upper(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const SemanticCube & cube, const KinematicLimits & limits);

//! cube 列 + 境界条件から QP を組んで解く。infeasible / ソルバー失敗なら nullopt。
//! velocity_upper は cube ごとの ṡ 上限 (cubes と同じ長さ)
std::optional<SscQpSolution> solve_ssc_qp(
  const std::vector<SemanticCube> & cubes, const std::vector<double> & velocity_upper,
  const SscBoundaryState & initial, const SscBoundaryState & terminal,
  const KinematicLimits & limits, const SscQpParams & params);

//! QP の解を時間グリッド (rough_plan と同一) 上でサンプルし、世界座標の軌道へ戻す
OptimizedTrajectory sample_ssc_solution(
  const PlannerContext & context, const std::vector<SemanticCube> & cubes,
  const SscQpSolution & solution, const std::vector<double> & sample_times);

}  // namespace autoware::safety_planner

#endif  // TRAJECTORY_PLANNER__SSC_QP_TRAJECTORY_OPTIMIZER_HPP_
