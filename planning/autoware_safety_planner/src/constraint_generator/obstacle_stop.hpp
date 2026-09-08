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

#ifndef CONSTRAINT_GENERATOR__OBSTACLE_STOP_HPP_
#define CONSTRAINT_GENERATOR__OBSTACLE_STOP_HPP_

#include "constraint_generator_interface.hpp"

#include <rclcpp/time.hpp>

#include <string>
#include <vector>

namespace autoware::safety_planner
{

//! 予測物体 1 件ごとに KeepOut (剛体) を certainty = DEFINITE で発行するプラグイン
//! (S7 §3-2 obstacle_stop)。waypoints は最尤の予測経路 1 本の翻訳で、予測経路が無い物体は
//! waypoint 1 点 = 静的物体として出す。
//! Why not 全予測モードの発行: 最尤以外まで DEFINITE で縛ると Nominal 軌道が過剰に硬くなる。
//! 低確度モードは POSSIBLE を出す別プラグイン (run_out 等) の領分
class ObstacleStopConstraintGenerator : public ConstraintGeneratorInterface
{
public:
  std::string get_name() const override { return "obstacle_stop"; }
  ConstraintGeneratorOutput generate_constraints(const PlannerContext & context) override;
};

//! PredictedObjects → KeepOut 制約列。t_plan は計画基準時刻 = odometry の stamp (S1 §2-1)。
//! perception のスタンプずれは waypoints の t にオフセットとして織り込む。
//! 退化した shape・非有限の pose を持つ物体は落として続行する (S7 §2)
std::vector<Constraint> make_obstacle_keep_out_constraints(
  const PredictedObjects & objects, const rclcpp::Time & t_plan, const double margin_m);

//! One Gate (stop line) per slow object whose footprint overlaps the reference-path corridor
//! ahead of the ego, placed stop_distance_m before the object's nearest point along the path.
//! Objects faster than max_object_speed_mps get no stop line (the KeepOut handles them).
//! Why not a stop line for every object: stopping 6 m behind a moving lead vehicle is wrong;
//! following is the KeepOut's job.
std::vector<Constraint> make_obstacle_stop_line_constraints(
  const PredictedObjects & objects, const PathPointTrajectory & reference_path, const double s_ego,
  const double corridor_half_width_m, const double stop_distance_m,
  const double max_object_speed_mps);

}  // namespace autoware::safety_planner

#endif  // CONSTRAINT_GENERATOR__OBSTACLE_STOP_HPP_
