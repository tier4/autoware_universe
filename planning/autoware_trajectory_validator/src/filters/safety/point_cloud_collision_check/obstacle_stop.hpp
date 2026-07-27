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

#ifndef FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__OBSTACLE_STOP_HPP_
#define FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__OBSTACLE_STOP_HPP_

#include "parameter.hpp"
#include "planner_data_lite.hpp"
#include "types.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{

// 移植元 ObstacleStopModule の点群経路（`filter_stop_obstacle_for_point_cloud` 系）に対応する。
class ObstacleStop
{
public:
  void update_parameters(const Params & params);

  std::vector<StopObstacle> calc_obstacle_stop(
    const std::vector<TrajectoryPoint> & raw_trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data);

private:
  DetectionPolygon get_trajectory_polygon(
    const std::vector<TrajectoryPoint> & decimated_traj_points, const VehicleInfo & vehicle_info,
    const geometry_msgs::msg::Pose & current_ego_pose, const PolygonParam & polygon_param,
    const bool enable_to_consider_current_pose, const double time_to_convergence,
    const double decimate_trajectory_step_length) const;

  // 前提: is_feasible は1サイクルに1候補のみを処理する（単一候補前提）。同一 stamp で
  // 複数の異なる軌道が来ると global return と arc-length 基準の距離が混線し誤判定し得る。
  void upsert_pointcloud_stop_candidates(
    const CollisionPointWithDist & nearest_collision_point,
    const std::vector<TrajectoryPoint> & traj_points, rclcpp::Time latest_point_cloud_time);

  std::vector<StopObstacle> filter_stop_obstacle_for_point_cloud(
    const nav_msgs::msg::Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points, const Pointcloud & point_cloud,
    const VehicleInfo & vehicle_info, const double x_offset_to_bumper,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check);

  std::optional<double> calc_ego_forwarding_braking_distance(
    const std::vector<TrajectoryPoint> & traj_points,
    const nav_msgs::msg::Odometry & odometry) const;

  std::optional<CollisionPointWithDist> get_nearest_collision_point(
    const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
    const Pointcloud & point_cloud, const double x_offset_to_bumper,
    const VehicleInfo & vehicle_info) const;

  CommonParam common_param_{};
  StopPlanningParam stop_planning_param_{};
  std::unordered_map<StopObstacleClassification::Type, ObstacleFilteringParam>
    obstacle_filtering_params_{};
  PointcloudSegmentationParam pointcloud_segmentation_param_;

  std::deque<PointcloudStopCandidate> pointcloud_stop_candidates{};
  mutable std::map<PolygonParam, DetectionPolygon> trajectory_polygon_for_inside_map_{};
  rclcpp::Logger logger_{rclcpp::get_logger("point_cloud_collision_check_filter")};
};

}  // namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check

#endif  // FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__OBSTACLE_STOP_HPP_
