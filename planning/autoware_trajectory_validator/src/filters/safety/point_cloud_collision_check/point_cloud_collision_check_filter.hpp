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
#include "debug_marker.hpp"
#include "parameter.hpp"
#include "planner_data_lite.hpp"
#include "types.hpp"

#include <rclcpp/logger.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <deque>
#include <map>
#include <optional>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
using point_cloud_collision_check::CollisionPointWithDist;
using point_cloud_collision_check::CommonParam;
using point_cloud_collision_check::DebugData;
using point_cloud_collision_check::DetectionPolygon;
using point_cloud_collision_check::ObstacleFilteringParam;
using point_cloud_collision_check::Odometry;
using point_cloud_collision_check::PlannerData;
using point_cloud_collision_check::PointcloudSegmentationParam;
using point_cloud_collision_check::PointcloudStopCandidate;
using point_cloud_collision_check::Polygon2d;
using point_cloud_collision_check::PolygonParam;
using point_cloud_collision_check::StopObstacle;
using point_cloud_collision_check::StopObstacleClassification;
using point_cloud_collision_check::StopPlanningParam;
using point_cloud_collision_check::TrajectoryPolygonCollisionCheck;

/**
 * @brief PointCloudCollisionCheckFilter class - checks the trajectory against the semantic
 * segmentation point cloud produced by the perception pipeline.
 */
class PointCloudCollisionCheckFilter final : public plugin::ValidatorInterface
{
public:
  PointCloudCollisionCheckFilter() : ValidatorInterface("point_cloud_collision_check_filter") {}

  result_t is_feasible(
    const CandidateTrajectory & candidate_trajectory, const FilterContext & context) final;

  void update_parameters(const validator::Params & params) final;

private:
  std::optional<double> calc_ego_forwarding_braking_distance(
    const std::vector<TrajectoryPoint> & traj_points, const Odometry & odometry) const;

  DetectionPolygon get_trajectory_polygon(
    const std::vector<TrajectoryPoint> & decimated_traj_points, const VehicleInfo & vehicle_info,
    const geometry_msgs::msg::Pose & current_ego_pose, const PolygonParam & polygon_param,
    const bool enable_to_consider_current_pose, const double time_to_convergence,
    const double decimate_trajectory_step_length) const;

  std::optional<CollisionPointWithDist> get_nearest_collision_point(
    const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
    const PlannerData::Pointcloud & point_cloud, const double x_offset_to_bumper,
    const VehicleInfo & vehicle_info) const;

  // 前提: is_feasible は1サイクルに1候補のみを処理する（単一候補前提）。
  void upsert_pointcloud_stop_candidates(
    const CollisionPointWithDist & nearest_collision_point,
    const std::vector<TrajectoryPoint> & traj_points, rclcpp::Time latest_point_cloud_time);

  std::vector<StopObstacle> filter_stop_obstacle_for_point_cloud(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const PlannerData::Pointcloud & point_cloud, const VehicleInfo & vehicle_info,
    const double x_offset_to_bumper,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check);

  std::vector<StopObstacle> calc_obstacle_stop(
    const std::vector<TrajectoryPoint> & raw_trajectory_points, const PlannerData & planner_data);

  /// @brief 評価に必要な入力が揃っているかを判定する。
  /// false のとき is_feasible は評価せず feasible（ValidationResult{}）を返す。
  bool is_available_data(
    const CandidateTrajectory & candidate_trajectory, const FilterContext & context) const;

  /// @brief planner_data_ のパラメータ由来フィールドを設定する。
  /// 移植元では PlannerData のコンストラクタと on_set_param が担う。
  void set_planner_data_param(const validator::Params::PointCloudCollisionCheck & p);

  /// @brief planner_data_ のトピック由来フィールドを更新し、点群の前処理まで行う。
  /// 移植元では node の update_planner_data が担う。
  void update_planner_data(
    const std::vector<TrajectoryPoint> & raw_trajectory_points, const FilterContext & context);

  /// @brief 最も手前の衝突距離が必要制動距離 + stop_margin を下回れば false（STOP REQUIRED）。
  /// @param[out] required_distance 自車の停止距離 + stop_margin（debug 表示用）
  bool judge_stop_feasibility(
    const std::vector<StopObstacle> & stop_obstacles, const geometry_msgs::msg::Twist & twist,
    double & required_distance) const;

  CommonParam common_param_{};
  StopPlanningParam stop_planning_param_{};
  std::unordered_map<StopObstacleClassification::Type, ObstacleFilteringParam>
    obstacle_filtering_params_{};
  PointcloudSegmentationParam pointcloud_segmentation_param_{};

  // 停止候補 deque はサイクルをまたいで保持する。
  std::deque<PointcloudStopCandidate> pointcloud_stop_candidates{};
  mutable std::map<PolygonParam, DetectionPolygon> trajectory_polygon_for_inside_map_{};
  rclcpp::Logger logger_{rclcpp::get_logger("point_cloud_collision_check_filter")};

  bool enable_debug_markers_{};
  DebugData debug_data_{};
  PlannerData planner_data_{};
};
}  // namespace autoware::trajectory_validator::plugin::safety

#endif  // FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__POINT_CLOUD_COLLISION_CHECK_FILTER_HPP_
