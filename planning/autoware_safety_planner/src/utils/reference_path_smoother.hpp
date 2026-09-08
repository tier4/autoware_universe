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

#ifndef UTILS__REFERENCE_PATH_SMOOTHER_HPP_
#define UTILS__REFERENCE_PATH_SMOOTHER_HPP_

#include "../type_alias.hpp"

#include <lanelet2_core/primitives/Lanelet.h>

#include <optional>

namespace autoware::safety_planner
{

//! reference_path を elastic band 型の QP でなめらかにする (autoware_path_smoother の EB と同じ
//! 定式化: 1 m 再サンプル点の 2 階差分の二乗和を最小化し、各点は法線方向にだけ動く)。
//! 可動範囲は ±clearance_m と、lanelets の左右 bound までの距離 − vehicle_half_width_m − マージン
//! の小さい方。終端 2 点 (goal 位置・向き) は動かさない。QP が解けない・点数が足りないときは
//! nullopt
std::optional<PathPointTrajectory> smooth_reference_path(
  const PathPointTrajectory & path, const lanelet::ConstLanelets & lanelets,
  double vehicle_half_width_m, double clearance_m);

}  // namespace autoware::safety_planner

#endif  // UTILS__REFERENCE_PATH_SMOOTHER_HPP_
