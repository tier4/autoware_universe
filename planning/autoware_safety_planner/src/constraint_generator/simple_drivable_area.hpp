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

#ifndef CONSTRAINT_GENERATOR__SIMPLE_DRIVABLE_AREA_HPP_
#define CONSTRAINT_GENERATOR__SIMPLE_DRIVABLE_AREA_HPP_

#include "constraint_generator_interface.hpp"

#include <string>
#include <vector>

namespace autoware::safety_planner
{

//! reference_path を左右へ定数幅、前後へ定数長だけ広げた 1 枚のポリゴンを
//! 「走行可能領域」とみなす簡易プラグイン (地図を引かない暫定版。本来は lanelet の
//! 車線境界・road_border から作る)。
//!
//! 制約 IR には「ポリゴンの内側に居ろ」という payload が無いので (constraint.hpp)、
//! ポリゴンの左辺・右辺を Boundary 2 本 (禁止側 = 外側) に分解して発行する。
//! 前後端は Boundary の端点がそのまま領域の前後端になる。
//! ポリゴンそのものは debug marker で可視化する。
class SimpleDrivableAreaConstraintGenerator : public ConstraintGeneratorInterface
{
public:
  std::string get_name() const override { return "simple_drivable_area"; }
  ConstraintGeneratorOutput generate_constraints(const PlannerContext & context) override;
};

//! 中心線サンプル (位置 + 方位) から左右のオフセット折れ線と閉ポリゴンを作る。
//! ポリゴンは CW・閉 (bg::correct 済み) で返す (constraint.hpp §3.5 決定 4)
struct DrivableAreaShape
{
  std::vector<Point2d> left;   //!< 進行方向順
  std::vector<Point2d> right;  //!< 進行方向順
  Polygon2d polygon{};
};

DrivableAreaShape make_drivable_area_shape(
  const std::vector<Pose2d> & centerline, double half_width_m, double forward_extension_m,
  double backward_extension_m);

}  // namespace autoware::safety_planner

#endif  // CONSTRAINT_GENERATOR__SIMPLE_DRIVABLE_AREA_HPP_
