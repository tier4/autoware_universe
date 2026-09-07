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

#ifndef UTILS__SL_VIEW_UTILS_HPP_
#define UTILS__SL_VIEW_UTILS_HPP_

// CompiledConstraints の射影ビュー ((s, l) 空間) を読む側の共通ヘルパー。
// rough_planner (時空間 DP) と trajectory_optimizer (SSC のコリドー彫り込み) の
// どちらもここを通す。**弧長 s の基準はその周期の context.reference_path 1 本だけ**
// (constraints_compiler.hpp の規約)。

#include "../context.hpp"
#include "../trajectory_planner/nlp_planner/constraints_compiler.hpp"
#include "../type_alias.hpp"

#include <vector>

namespace autoware::safety_planner
{

// ---------------------------------------------------------------------------------------------
// 車両運動限界 (IR から読む)
// ---------------------------------------------------------------------------------------------

//! 全域 (region 無し・常時) の ScalarBound を IR から集約したもの。発行元は
//! vehicle_kinematics プラグイン (値の正はその ROS パラメータ)。
//! Tier 削除に伴い IR から読むのはハード上限 (v_hard / a_hard_*) のみで、
//! nominal 系 (v_nom / a_nom_* / a_lat_nom) はこの struct の既定値を使う
//! (IR 経由の快適上限の受け渡しは廃止。設定可能にしたくなったら params 経由で渡す)
struct KinematicLimits
{
  double v_hard{16.7};      //!< [m/s] 速度ハード上限 (VELOCITY)
  double a_hard_min{-6.0};  //!< [m/s²] 最大減速 (LON_ACCEL)
  double a_hard_max{6.0};   //!< [m/s²] 最大加速
  double v_nom{13.88};      //!< [m/s] 巡航目標速度
  double a_nom_min{-1.0};   //!< [m/s²] 快適減速
  double a_nom_max{1.0};    //!< [m/s²] 快適加速
  double a_lat_nom{2.0};    //!< [m/s²] コーナー減速の横加速度上限 (S3 §3.3)
};

KinematicLimits collect_kinematic_limits(const CompiledConstraints & compiled_constraints);

// ---------------------------------------------------------------------------------------------
// reference_path 上の Frenet 座標
// ---------------------------------------------------------------------------------------------

//! reference_path 上の弧長と横オフセット (左が正)
struct EgoFrenetState
{
  double s{0.0};
  double l{0.0};
};

//! ego の位置を reference_path へ射影する
EgoFrenetState compute_ego_frenet_state(const PlannerContext & context);

//! 弧長 s の中心線上の点から見た、世界座標の点 q の横オフセット l
double lateral_offset_at(const PathPointTrajectory & path, double s, const Point2d & q);

//! (s, l) を世界座標へ戻す。yaw は中心線接線 (l 方向の傾きは含まない)
Pose2d to_world_pose(const PathPointTrajectory & path, double s, double l);

// ---------------------------------------------------------------------------------------------
// footprint の (s, l) 外接 box
// ---------------------------------------------------------------------------------------------

//! footprint を (s, l) 空間で外接する box (射影ビュー評価用の保守近似。
//! θ = θ_ref で代表し、ヘディング偏差分の膨らみは margin が吸収する。S3 §3.2)
struct SlBox
{
  double s_min{0.0};
  double s_max{0.0};
  double l_min{0.0};
  double l_max{0.0};
};

//! 基準点 (後軸) が (s, l) にあるときの footprint 外接 box
SlBox footprint_sl_box(const VehicleInfo & vehicle_info, double s, double l);

//! 基準点が (s, l) の box を動く場合の footprint 掃引 box
SlBox footprint_sl_box(const VehicleInfo & vehicle_info, const SlBox & reference_box);

// ---------------------------------------------------------------------------------------------
// 射影ビューの評価
// ---------------------------------------------------------------------------------------------

//! 境界折れ線の l を弧長 s で線形補間する (polyline は s 昇順)
double interpolate_boundary_l(const std::vector<SlPoint> & polyline, double s);

//! 弧長区間 [s_lo, s_hi] における境界 l の極値。
//! LEFT 禁止 (境界より左が禁止) なら最も許容が狭い min(l)、RIGHT なら max(l)。
//! 区間が折れ線の s 範囲と重ならない場合は false を返し、l は書き換えない
bool lateral_bound_extreme_l(
  const LateralBoundEntry & bound, double s_lo, double s_hi, double & extreme_l);

//! footprint box が境界の禁止側 (margin 膨張込み) に踏み込むか
bool violates_lateral_bound(const LateralBoundEntry & bound, const SlBox & box);

//! footprint box が時刻窓 [t0, t1] に占有スラブと重なるか (KeepOut の定数 margin_m 膨張込み)
bool violates_occupancy(
  const OccupancyEntry & occupancy, const CompiledConstraints & compiled_constraints,
  const SlBox & box, double t0, double t1);

//! footprint 前端が時刻窓 [t0, t1] に有効な停止線を越えるか
bool violates_stop_bar(const StopBarEntry & stop_bar, const SlBox & box, double t0, double t1);

}  // namespace autoware::safety_planner

#endif  // UTILS__SL_VIEW_UTILS_HPP_
