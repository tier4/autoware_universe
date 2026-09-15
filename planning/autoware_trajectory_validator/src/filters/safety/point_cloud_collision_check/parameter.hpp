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

#ifndef FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__PARAMETER_HPP_
#define FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__PARAMETER_HPP_

#include "planner_data_lite.hpp"

#include <autoware_trajectory_validator/autoware_trajectory_validator_param.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{

struct CommonParam
{
  double max_accel{};
  double min_accel{};
  double max_jerk{};
  double min_jerk{};
};

struct ObstacleFilteringParam
{
  bool check_inside{};

  struct TrimTrajectoryParam
  {
    bool enable_trimming{};
    double min_trajectory_length{};
    double braking_distance_scale_factor{};
  } trim_trajectory;

  struct LateralMarginParam
  {
    double nominal_margin{};
    double additional_wheel_off_track_scale{};
    double is_moving_threshold_velocity{};
    double additional_is_stop_margin{};
    double additional_is_moving_margin{};

    double max_margin(const VehicleInfo & vehicle_info) const
    {
      return nominal_margin + additional_wheel_off_track_scale * vehicle_info.wheel_base_m +
             std::max(additional_is_stop_margin, additional_is_moving_margin);
    };
  } lateral_margin;

  double stop_obstacle_hold_time_threshold{};
};

struct PointcloudSegmentationParam
{
  struct
  {
    double max_time_diff{};
    double min_velocity{};
    double max_velocity{};
    double position_diff{};
  } time_series_association;
  struct
  {
    bool use_estimated_velocity{};
    double min_clamp_velocity{};
    double max_clamp_velocity{};
    size_t required_velocity_count{};
    double lpf_gain{};
  } velocity_estimation;
  struct
  {
    double margin_from_bottom{};
    double margin_from_top{};
  } height_margin;
};

struct RSSParam
{
  bool use_rss_stop{};

  double two_wheel_objects_deceleration{};
  double vehicle_objects_deceleration{};
  double no_wheel_objects_deceleration{};
  double pointcloud_deceleration{};
  double velocity_offset{};
};

struct StopPlanningParam
{
  double stop_margin{};
  RSSParam rss_params;
  double obstacle_velocity_threshold_enter_fixed_stop{};
};

struct TrajectoryPolygonParam
{
  bool enable_to_consider_current_pose{};
  double time_to_convergence{};
  double ego_nearest_dist_threshold{};
  double ego_nearest_yaw_threshold{};
  double decimate_trajectory_step_length{};
  double goal_extended_trajectory_length{};
};

struct Params
{
  CommonParam common{};
  ObstacleFilteringParam obstacle_filtering{};
  PointcloudSegmentationParam pointcloud_segmentation{};
  StopPlanningParam stop_planning{};
  TrajectoryPolygonParam trajectory_polygon{};
  PointcloudPreprocessParams preprocess{};
  std::vector<std::int64_t> excluded_class_ids{};
  bool enable_debug_markers{};

  Params() = default;
  explicit Params(const validator::Params::PointCloudCollisionCheck & p)
  {
    common.max_accel = p.common.max_accel;
    common.min_accel = p.common.min_accel;
    common.max_jerk = p.common.max_jerk;
    common.min_jerk = p.common.min_jerk;

    obstacle_filtering.check_inside = p.obstacle_filtering.check_inside;
    obstacle_filtering.stop_obstacle_hold_time_threshold =
      p.obstacle_filtering.stop_obstacle_hold_time_threshold;
    obstacle_filtering.trim_trajectory.enable_trimming =
      p.obstacle_filtering.trim_trajectory.enable_trimming;
    obstacle_filtering.trim_trajectory.min_trajectory_length =
      p.obstacle_filtering.trim_trajectory.min_trajectory_length;
    obstacle_filtering.trim_trajectory.braking_distance_scale_factor =
      p.obstacle_filtering.trim_trajectory.braking_distance_scale_factor;
    obstacle_filtering.lateral_margin.nominal_margin =
      p.obstacle_filtering.lateral_margin.nominal_margin;
    obstacle_filtering.lateral_margin.additional_wheel_off_track_scale =
      p.obstacle_filtering.lateral_margin.additional_wheel_off_track_scale;
    obstacle_filtering.lateral_margin.is_moving_threshold_velocity =
      p.obstacle_filtering.lateral_margin.is_moving_threshold_velocity;
    obstacle_filtering.lateral_margin.additional_is_stop_margin =
      p.obstacle_filtering.lateral_margin.additional_is_stop_margin;
    obstacle_filtering.lateral_margin.additional_is_moving_margin =
      p.obstacle_filtering.lateral_margin.additional_is_moving_margin;

    pointcloud_segmentation.time_series_association.max_time_diff =
      p.time_series_association.max_time_diff;
    pointcloud_segmentation.time_series_association.min_velocity =
      p.time_series_association.min_velocity;
    pointcloud_segmentation.time_series_association.max_velocity =
      p.time_series_association.max_velocity;
    pointcloud_segmentation.time_series_association.position_diff =
      p.time_series_association.position_diff;
    pointcloud_segmentation.velocity_estimation.use_estimated_velocity =
      p.velocity_estimation.use_estimated_velocity;
    pointcloud_segmentation.velocity_estimation.min_clamp_velocity =
      p.velocity_estimation.min_clamp_velocity;
    pointcloud_segmentation.velocity_estimation.max_clamp_velocity =
      p.velocity_estimation.max_clamp_velocity;
    pointcloud_segmentation.velocity_estimation.required_velocity_count =
      static_cast<size_t>(p.velocity_estimation.required_velocity_count);
    pointcloud_segmentation.velocity_estimation.lpf_gain = p.velocity_estimation.lpf_gain;
    pointcloud_segmentation.height_margin.margin_from_bottom = p.height_margin.margin_from_bottom;
    pointcloud_segmentation.height_margin.margin_from_top = p.height_margin.margin_from_top;

    stop_planning.stop_margin = p.stop_margin;
    stop_planning.rss_params.use_rss_stop = p.rss_params.use_rss_stop;
    stop_planning.rss_params.two_wheel_objects_deceleration =
      p.rss_params.two_wheel_objects_deceleration;
    stop_planning.rss_params.vehicle_objects_deceleration =
      p.rss_params.vehicle_objects_deceleration;
    stop_planning.rss_params.no_wheel_objects_deceleration =
      p.rss_params.no_wheel_objects_deceleration;
    stop_planning.rss_params.pointcloud_deceleration = p.rss_params.pointcloud_deceleration;
    stop_planning.rss_params.velocity_offset = p.rss_params.velocity_offset;
    stop_planning.obstacle_velocity_threshold_enter_fixed_stop =
      p.obstacle_velocity_threshold_enter_fixed_stop;

    trajectory_polygon.enable_to_consider_current_pose =
      p.trajectory_polygon.enable_to_consider_current_pose;
    trajectory_polygon.time_to_convergence = p.trajectory_polygon.time_to_convergence;
    trajectory_polygon.ego_nearest_dist_threshold = p.trajectory_polygon.ego_nearest_dist_threshold;
    trajectory_polygon.ego_nearest_yaw_threshold = p.trajectory_polygon.ego_nearest_yaw_threshold;
    trajectory_polygon.decimate_trajectory_step_length =
      p.trajectory_polygon.decimate_trajectory_step_length;
    trajectory_polygon.goal_extended_trajectory_length =
      p.trajectory_polygon.goal_extended_trajectory_length;

    preprocess.filter_by_trajectory_polygon.enable_monolithic_crop_box =
      p.pointcloud_preprocessing.filter_by_trajectory_polygon.enable_monolithic_crop_box;
    preprocess.filter_by_trajectory_polygon.enable_multi_polygon_filtering =
      p.pointcloud_preprocessing.filter_by_trajectory_polygon.enable_multi_polygon_filtering;
    preprocess.filter_by_trajectory_polygon.min_trajectory_length =
      p.pointcloud_preprocessing.filter_by_trajectory_polygon.min_trajectory_length;
    preprocess.filter_by_trajectory_polygon.braking_distance_scale_factor =
      p.pointcloud_preprocessing.filter_by_trajectory_polygon.braking_distance_scale_factor;
    preprocess.filter_by_trajectory_polygon.lateral_margin =
      p.pointcloud_preprocessing.filter_by_trajectory_polygon.lateral_margin;
    preprocess.filter_by_trajectory_polygon.height_margin =
      p.pointcloud_preprocessing.filter_by_trajectory_polygon.height_margin;
    preprocess.downsample_by_voxel_grid.enable_downsample =
      p.pointcloud_preprocessing.downsample_by_voxel_grid.enable_downsample;
    preprocess.downsample_by_voxel_grid.voxel_size_x =
      p.pointcloud_preprocessing.downsample_by_voxel_grid.voxel_size_x;
    preprocess.downsample_by_voxel_grid.voxel_size_y =
      p.pointcloud_preprocessing.downsample_by_voxel_grid.voxel_size_y;
    preprocess.downsample_by_voxel_grid.voxel_size_z =
      p.pointcloud_preprocessing.downsample_by_voxel_grid.voxel_size_z;
    preprocess.euclidean_clustering.enable_clustering =
      p.pointcloud_preprocessing.euclidean_clustering.enable_clustering;
    preprocess.euclidean_clustering.cluster_tolerance =
      p.pointcloud_preprocessing.euclidean_clustering.cluster_tolerance;
    preprocess.euclidean_clustering.min_cluster_size =
      static_cast<int>(p.pointcloud_preprocessing.euclidean_clustering.min_cluster_size);
    preprocess.euclidean_clustering.max_cluster_size =
      static_cast<int>(p.pointcloud_preprocessing.euclidean_clustering.max_cluster_size);

    excluded_class_ids = p.obstacle_filtering.excluded_class_ids;
  }
};

}  // namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check

#endif  // FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__PARAMETER_HPP_
