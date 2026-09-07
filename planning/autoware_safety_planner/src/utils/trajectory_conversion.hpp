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

#ifndef UTILS__TRAJECTORY_CONVERSION_HPP_
#define UTILS__TRAJECTORY_CONVERSION_HPP_

// 内部表現 (RoughPlanPoint / OptimizedTrajectoryPoint) から出力 msg への変換と、
// 出力軌道への後処理。node (デバッグ配信) と trajectory_planner プラグインの
// どちらの .so からも呼ぶので common に置く

#include "../trajectory_planner/nlp_planner/rough_planner.hpp"
#include "../trajectory_planner/nlp_planner/trajectory_optimizer_interface.hpp"
#include "../type_alias.hpp"

namespace autoware::safety_planner
{

//! RoughPlan の 1 点を出力軌道の点へ落とす。
//! rough_plan は 2D (Pose2d + kappa) なので、z は基準 pose (ego) の高さを流用する
TrajectoryPoint to_trajectory_point(
  const RoughPlanPoint & rough_point, const double z, const double wheel_base_m);

//! 最適化された軌道の 1 点を出力軌道の点へ落とす。2D なので z は基準 pose (ego) の高さを流用する
TrajectoryPoint to_trajectory_point(
  const OptimizedTrajectoryPoint & optimized_point, const double z, const double wheel_base_m);

//! 発進区間に速度の下限 (engage 速度) を敷く。停止状態からの発進周期は先頭点の v が
//! 0 付近になり、縦制御が「距離 0 の位置に停止点がある」と解釈して STOPPED から抜けないため。
//! 先頭から「v が下限以上になる最初の点」までだけ書き換えるので、終端 (goal) の減速・停止
//! 区間には触れない。進行距離が僅かな周期 (goal 目前・停止 plan) は何もしない
Trajectory set_engage_speed(const Trajectory & trajectory, const double engage_velocity_mps);

}  // namespace autoware::safety_planner

#endif  // UTILS__TRAJECTORY_CONVERSION_HPP_
