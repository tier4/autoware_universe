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

#ifndef CONSTRAINT_GENERATOR__LANE_FOLLOWING_DRIVABLE_AREA_HPP_
#define CONSTRAINT_GENERATOR__LANE_FOLLOWING_DRIVABLE_AREA_HPP_

#include "constraint_generator_interface.hpp"

#include <string>

namespace autoware::safety_planner
{

//! 地図から走行可能領域の制約 (Boundary) を生成する。
//! route 上のレーン列の各 lanelet について、左右それぞれ:
//! - 側方に並走車線 (対向含む。road subtype・方位 ±45°) があれば **自レーンの bound** を
//!   hard 境界にする (車線変更を禁止する)
//! - 無ければ最寄りの **road_border** (物理的な道路外縁) を hard 境界にする
//!   (路肩・ゼブラ等の上は走行可能領域に含まれる)
//!
//! Why not 路肩 lanelet (left/right_shoulder_lanelet) ベースの拡張: 隣接判定が線分オブジェクトの
//! 共有を前提としており、線分を lanelet ごとに複製する作りの地図では一度も発火しない。
//! 並走車線の有無も同じ理由で routing graph の隣接には頼らず、幾何 (点サンプル) で判定する。
class LaneFollowingDrivableAreaConstraintGenerator : public ConstraintGeneratorInterface
{
public:
  std::string get_name() const override { return "lane_following_drivable_area"; }
  ConstraintGeneratorOutput generate_constraints(const PlannerContext & context) override;
};

}  // namespace autoware::safety_planner

#endif  // CONSTRAINT_GENERATOR__LANE_FOLLOWING_DRIVABLE_AREA_HPP_
