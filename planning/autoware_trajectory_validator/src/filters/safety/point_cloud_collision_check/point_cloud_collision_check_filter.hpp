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

#ifndef FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__POINT_CLOUD_COLLISION_CHECK_FILTER_HPP_
#define FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__POINT_CLOUD_COLLISION_CHECK_FILTER_HPP_

#include "autoware/trajectory_validator/validator_interface.hpp"
#include "parameter.hpp"
#include "planner_data_lite.hpp"
#include "types.hpp"

#include <geometry_msgs/msg/twist.hpp>

#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
/**
 * @brief PointCloudCollisionCheckFilter class - checks the trajectory against the semantic
 * segmentation point cloud produced by the perception pipeline.
 */
class PointCloudCollisionCheckFilter final : public plugin::ValidatorInterface
{
public:
  PointCloudCollisionCheckFilter() : ValidatorInterface("point_cloud_collision_check_filter") {}
  ~PointCloudCollisionCheckFilter() override = default;

  result_t is_feasible(
    const CandidateTrajectory & candidate_trajectory, const FilterContext & context) final;

  void update_parameters(const validator::Params & params) final;

private:
  /// @brief 評価に必要な入力が揃っているかを判定する。
  /// false のとき is_feasible は評価せず feasible（ValidationResult{}）を返す。
  bool is_available_data(
    const CandidateTrajectory & candidate_trajectory, const FilterContext & context) const;

  /// @brief planner_data_ のパラメータ由来フィールドを設定する。
  /// 移植元では PlannerData のコンストラクタと on_set_param が担う。
  void set_planner_data_param();

  /// @brief planner_data_ のトピック由来フィールドを更新し、点群の前処理まで行う。
  /// 移植元では node の update_planner_data が担う。
  void update_planner_data(
    const std::vector<TrajectoryPoint> & raw_trajectory_points, const FilterContext & context);

  /// @brief 点群から停止対象を抽出する。移植元 ObstacleStopModule の plan() の点群経路。
  std::vector<point_cloud_collision_check::StopObstacle> calc_obstacle_stop(
    const std::vector<TrajectoryPoint> & raw_trajectory_points);

  /// @brief 停止対象から停止可否を判定する。未実装のため現状は常に true を返す。
  bool judge_stop_feasibility(
    const std::vector<point_cloud_collision_check::StopObstacle> & stop_obstacles,
    const geometry_msgs::msg::Twist & twist) const;

  point_cloud_collision_check::Params params_{};
  point_cloud_collision_check::PlannerData planner_data_{};
};
}  // namespace autoware::trajectory_validator::plugin::safety

#endif  // FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__POINT_CLOUD_COLLISION_CHECK_FILTER_HPP_
