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

#ifndef TRAJECTORY_PLANNER__FRENET_SAMPLER__FRENET_SAMPLER_HPP_
#define TRAJECTORY_PLANNER__FRENET_SAMPLER__FRENET_SAMPLER_HPP_

// サンプリングベースの軌道プランナープラグイン (sampling_based_planner/autoware_path_sampler
// の Frenet 経路サンプリングを踏襲)。
// 経路と速度を分離してサンプルする:
// - 経路 l(s): reference_path 上の Frenet 座標で、終端 (弧長 L・横位置 l_T) を格子状にサンプル
//   し quintic 多項式 l(s) で結ぶ。初期勾配 l'(0) は ego の heading から取るので開始 heading が
//   ego と一致し、曲率は空間サンプル (path_resolution_m 間隔) から幾何的に求まる
// - 速度 s(t): 終端 (時間長 T・縦速度 v_T) をサンプルし quintic 多項式 s(t) で結ぶ
// 候補 = 経路 × 速度。制約 IR の射影ビュー (sl_view_utils) で hard 制約を評価し、通った
// もののうちコスト最小を採用する。座標基準は constraints_compiler.hpp の規約どおり、その
// 周期の context.reference_path 1 本だけ (autoware_frenet_planner の Spline2D は使わない)。
// 時間パラメタライズの l(t) を直接サンプルしないのは、停止発進時に s も l も t³ で立ち上がって
// 開始 heading が ego とずれ、車両運動チェックで全候補が落ちるため

#include "../../utils/sl_view_utils.hpp"
#include "../nlp_planner/constraints_compiler.hpp"
#include "../trajectory_planner_interface.hpp"

#include <optional>
#include <string>
#include <vector>

namespace autoware::safety_planner
{

class FrenetSamplingBasedPlanner : public TrajectoryPlannerInterface
{
public:
  std::string get_name() const override { return "frenet_sampler"; }

  TrajectoryPlannerResult plan(const TrajectoryPlannerInput & input) override;

private:
  //! ego 状態 (Frenet)。多項式の初期条件
  struct InitialState
  {
    double s{0.0};
    double l{0.0};
    double dl_ds{0.0};  //!< [-] 横位置の弧長勾配 tan(ego heading − 中心線接線)
    double v{0.0};      //!< [m/s] 縦速度 ds/dt
    double a{0.0};      //!< [m/s²]
  };

  //! 空間サンプルされた経路 (s 昇順・等間隔 path_resolution_m、s0 から reference_path 終端まで)
  struct PathCandidate
  {
    std::vector<double> s;
    std::vector<double> l;
    std::vector<double> yaw;    //!< [rad] 世界座標の heading
    std::vector<double> kappa;  //!< [1/m]
    std::string tag;
  };

  //! 時間サンプルされた縦プロファイル (t_k = k·dt)
  struct VelocityProfile
  {
    std::vector<double> t;
    std::vector<double> s;
    std::vector<double> v;
    std::vector<double> a;
    std::string tag;
  };

  //! 経路 × 速度を合成した軌道候補
  struct Candidate
  {
    std::vector<double> s;  //!< [m] s(t_k)
    std::vector<double> l;  //!< [m] l(s(t_k))
    std::vector<OptimizedTrajectoryPoint> points;
    double cost{0.0};
    bool valid{true};
    std::string tag;
  };

  std::optional<Trajectory> plan_one_side(
    const PlannerContext & context, const CompiledConstraints & compiled_constraints,
    MarkerArray & debug_markers) const;

  InitialState compute_initial_state(const PlannerContext & context) const;

  std::vector<PathCandidate> generate_paths(
    const PlannerContext & context, const InitialState & initial_state) const;

  std::vector<VelocityProfile> generate_velocity_profiles(
    const PlannerContext & context, const InitialState & initial_state,
    const CompiledConstraints & compiled_constraints) const;

  //! 有効な候補が 1 本も無いときの最終手段: 現在の横位置を保ったまま最大減速で止まる
  VelocityProfile make_stop_profile(
    const InitialState & initial_state, const KinematicLimits & limits) const;

  //! 経路上の s(t_k) に沿って l / yaw / κ を補間し、世界座標の点列を作る
  Candidate combine(
    const PlannerContext & context, const PathCandidate & path,
    const VelocityProfile & profile) const;

  //! hard 制約の評価と soft コストの計算。valid / cost を書き込む
  void evaluate(
    const PlannerContext & context, const CompiledConstraints & compiled_constraints,
    Candidate & candidate) const;

  Trajectory to_trajectory_msg(const PlannerContext & context, const Candidate & candidate) const;

  void append_debug_markers(
    const PlannerContext & context, const std::vector<Candidate> & candidates,
    MarkerArray & debug_markers) const;
};

}  // namespace autoware::safety_planner

#endif  // TRAJECTORY_PLANNER__FRENET_SAMPLER__FRENET_SAMPLER_HPP_
