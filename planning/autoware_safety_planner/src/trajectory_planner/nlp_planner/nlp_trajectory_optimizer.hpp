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

#ifndef TRAJECTORY_PLANNER__NLP_PLANNER__NLP_TRAJECTORY_OPTIMIZER_HPP_
#define TRAJECTORY_PLANNER__NLP_PLANNER__NLP_TRAJECTORY_OPTIMIZER_HPP_

#include "ssc_corridor.hpp"
#include "trajectory_optimizer_interface.hpp"

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::safety_planner
{

// 時間パラメタライズ非凸 NLP による軌道最適化 (PoC 路線)。
// 定式化は docs/spec/3_trajectory_optimizer/trajectory_optimizer.md (S5)、比較は
// docs/safety_planner_arch_design/formulation_ssc_vs_poc.md §3。
//
//   状態  x = [px, py, θ, κ, v, a]        入力 u = [w, j]   (w = dκ/dt, j = da/dt)
//   力学  ṗx = v cosθ, ṗy = v sinθ, θ̇ = vκ, κ̇ = w, v̇ = a, ȧ = j   (RK4 / 1 step per stage)
//
// κ を状態に持つので、ステア角 |δ| = |atan(Lκ)| とステアレート δ̇ = Lw/(1+(Lκ)²) を
// そのまま hard box に書ける (SSC 系が構造的に書けないもの)。代償が非凸性で、
// それを三段フォールバック (C 行・B 行を段階的に落とす) と独立検証で埋める。
//
// ソルバーは **acados** (SQP + PARTIAL_CONDENSING_HPIPM)。問題の構造は codegen 時に 1 回だけ
// 決め (generators/nlp_time_ocp.py)、周期処理はパラメータ・境界値・参照の注入と solve だけ。
// 段を落とす操作も行の削除ではなく境界値を ±1e6 へ開く「無効化」で行うので、
// ソルバーの再構築は起きない (S6 §4.1 原則 1・2)。
//
// コリドー (幾何の B 行) の横 2 面は **SSC の semantic corridor を共用**する。v1 にはステージ
// ごとに半空間を彫る層が他に無く、cube (s,l,t) は「その時間帯の自由空間」を保証しているので、
// ステージ点をその cube に閉じ込める形で載せられる。NLP 専用のコリドー彫りを新設していない
// のは、非凸探索 = ホモトピー決定を 2 箇所に持たせないため。前方カットだけは cube を使わず、
// ステージ時刻に前方で効く占有・停止線から直接引く (make_stage_planes)。

//! 三段フォールバックの段 (S5 §8.1)
enum class SolveLevel : std::uint8_t {
  LEVEL_1,  //!< SAFETY + COMFORT 全部。目的 = NOMINAL
  LEVEL_2,  //!< COMFORT を全ステージ無効化。目的 = NOMINAL
  LEVEL_3,  //!< SAFETY も無効化。目的 = STOP、初期値 = certificate
};

//! NLP のパラメータ (ROS ns `trajectory_optimizer.nlp.*`)。既定値は S5 §10 の初期値表
struct NlpParams
{
  //! コリドー彫り込み。SSC と同じ実装を共用するが、パラメータは別 ns で持つ
  //! (α や膨張上限は定式化ごとに調整したいので、片方を触ると他方が動く形にしない)
  SscCorridorParams corridor;

  //! [1/(m·s)] |w| の hard 上限 (S5 §3.1)。IR には κ の変化率を出すプラグインが無いので
  //! ここで持つ。κ 自身の上限はステア角 (IR の VEHICLE STEER_ANGLE) から導出する
  double curvature_rate_max{0.6};
  //! [m] 追従参照の終点を goal そのものに差し替える距離 (S5 §4.3)。0 で差し替えない
  double goal_capture_distance_m{5.0};

  // コスト重み (S5 §4.1)
  double weight_pos{1.0};
  double weight_yaw{1.0};
  double weight_v{0.5};
  double weight_kappa{200.0};
  double weight_w{100.0};
  double weight_a{0.1};
  double weight_j{0.1};
  double weight_terminal_v{10.0};
  double weight_terminal_a{1.0};

  //! スラックの二次ペナルティ係数 (S5 §3.4)。L1 は使わない (収束を壊す実績)
  double slack_safety{1.0e6};
  double slack_comfort{1.0e4};

  // 段ごとの反復上限 (S5 §8.3)
  int max_iter_level1{20};
  int max_iter_level2{8};
  int max_iter_level3{6};
};

//! IR (CompiledConstraints) から読む車両運動限界。KinematicLimits (sl_view) が持たない
//! 躍度・曲率・ステアレートを足したもの。射影ビューの消費側は使わないので sl_view には置かない
struct NlpLimits
{
  KinematicLimits base;          //!< v / a / a_lat (全域 ScalarBound)
  double j_hard{5.0};            //!< [m/s³] |j| の VEHICLE 上限
  double j_nom{1.6};             //!< [m/s³] |j| の COMFORT 上限
  double kappa_max{0.156};       //!< [1/m]  ステア角上限から導出した |κ| 上限
  double steer_rate_hard{3.0};   //!< [rad/s] |δ̇| の VEHICLE 上限
  double steer_rate_nom{0.995};  //!< [rad/s] |δ̇| の COMFORT 上限
  double a_lat_hard{6.0};        //!< [m/s²] v²|κ| の VEHICLE 上限 (検証のみで使う)
};

class NlpTrajectoryOptimizer : public TrajectoryOptimizerInterface
{
public:
  NlpTrajectoryOptimizer();
  ~NlpTrajectoryOptimizer() override;

  std::string get_name() const override { return "nlp"; }

  TrajectoryOptimizerResult optimize(const TrajectoryOptimizerInput & input) override;

private:
  //! ROS パラメータを読む (on_initialize 済みであること)
  NlpParams read_params() const;

  //! 生成された acados ソルバーの薄いラッパ。生成ヘッダをこのヘッダへ出さないための pimpl。
  //! ソルバーは周期をまたいで生き続ける (構造は 1 回だけ作る。S6 §4.1 原則 1)
  class Solver;
  std::unique_ptr<Solver> solver_;
};

// ---------------------------------------------------------------------------------------------
// 以下は plugin 外からもテストできるよう自由関数にしてある
// ---------------------------------------------------------------------------------------------

//! 半空間 n·p ≤ d (n は単位ベクトル、planning frame)
struct HalfPlane
{
  double nx{1.0};
  double ny{0.0};
  double d{0.0};
};

//! 1 ステージ分のコリドー。横 2 面 (cube 由来) + 前方カット 1 面 (占有・停止線由来、この順)。
//! 後方カットは課さない
//! (v ≥ 0 の box があるので後退せず、後方に守る対象も無い。先頭 cube の s0 は経路始端で
//! 切れるので、課すと footprint 後端が構造的に外へ出てスラックが解を前方へ押す)
inline constexpr std::size_t NUM_PLANES = 3;
using StagePlanes = std::array<HalfPlane, NUM_PLANES>;

//! rough_plan を N+1 点の時間格子へ載せ替える。点数が一致していれば恒等写像。
//! 一致しない周期 (rough_planner の格子が acados の codegen ステージ数と違う設定) では
//! t について線形補間する。s も同じ格子へ載せる
RoughPlan resample_rough_plan(const RoughPlan & plan, std::size_t num_points);

//! 全域 ScalarBound から NLP が必要な限界値を集める。ステア角上限だけは車両諸元由来なので
//! IR に無い場合の既定値として wheel_base を使う
NlpLimits collect_nlp_limits(
  const CompiledConstraints & compiled_constraints, const VehicleInfo & vehicle_info);

//! 離散写像 F(x, u) — RK4 陽的 4 段、1 ステップ/ステージ (S5 §1.2)。
//! **NLP の力学等式・certificate の前進シミュレーション・検証の残差はこの 1 実装を共有する**
OptimizedTrajectoryPoint integrate_rk4(
  const OptimizedTrajectoryPoint & state, double w, double j, double dt);

//! 最終手段の停止軌道 (最大快適減速の前進シミュレーション。S5 §7)。
//! 力学等式を厳密に満たし、A の box を全ステージで満たすので「実行可能点の証人」になる
OptimizedTrajectory make_certificate(
  const OptimizedTrajectoryPoint & initial, const NlpLimits & limits, std::size_t num_points,
  double dt);

//! 生値 ego から A の box を満たす初期状態を作る (S5 §6)。a₀ の下限 −√(2 j_hard v₀) が
//! 「減速を j_hard で戻す間に v ≥ 0 が保てる」条件
OptimizedTrajectoryPoint condition_initial_state(
  const OptimizedTrajectoryPoint & raw, const NlpLimits & limits);

//! cube 列をステージごとの半空間へ落とす。ステージ k の anchor は seed の弧長 s_k で、
//! そこでの中心線接線・法線で cube の横 2 面を世界座標の直線に近似する。前方カットは
//! 時刻 t_k に横バンド内で前方に効く占有・停止線の手前 (margin_m 込み) に置き、無ければ
//! max_longitudinal_inflation_m だけ先に置く
std::vector<StagePlanes> make_stage_planes(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const std::vector<SemanticCube> & cubes, const std::vector<CorridorSeedPoint> & seed,
  const SscCorridorParams & corridor);

//! 検証の結果 (S5 §9)。ソルバーのステータスは成否判定に使わない (solved ≠ satisfied)
struct NlpVerification
{
  bool dynamics_ok{false};  //!< 力学残差 r̃ ≤ ε_dyn
  bool vehicle_ok{false};   //!< A tier (box + ステアレート hard)
  bool safety_ok{false};    //!< B tier (コリドー幾何 + 区間速度)
  bool comfort_ok{false};   //!< C tier (快適スカラー)
  double max_dynamics_residual{0.0};
  std::string message;  //!< 最初に落ちた行の説明 (report 用)

  //! 段の成功条件 (S5 §9.3)
  bool passes(SolveLevel level) const;
};

//! 素の C++ で測り直す独立検証。幾何は射影ビュー (sl_view) を通して評価する
NlpVerification verify_trajectory(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const OptimizedTrajectory & trajectory, const NlpLimits & limits);

}  // namespace autoware::safety_planner

#endif  // TRAJECTORY_PLANNER__NLP_PLANNER__NLP_TRAJECTORY_OPTIMIZER_HPP_
