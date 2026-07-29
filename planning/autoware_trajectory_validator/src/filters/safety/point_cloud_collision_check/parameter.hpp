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

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{

struct CommonParam
{
  double max_accel{};
  double min_accel{};
  double max_jerk{};
  double min_jerk{};

  CommonParam() = default;
  explicit CommonParam(const validator::Params::PointCloudCollisionCheck & p)
  {
    max_accel = p.common.max_accel;
    min_accel = p.common.min_accel;
    max_jerk = p.common.max_jerk;
    min_jerk = p.common.min_jerk;
  }
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

  ObstacleFilteringParam() = default;
  explicit ObstacleFilteringParam(const validator::Params::PointCloudCollisionCheck & p)
  {
    check_inside = p.obstacle_filtering.check_inside;

    trim_trajectory.enable_trimming = p.obstacle_filtering.trim_trajectory.enable_trimming;
    trim_trajectory.min_trajectory_length =
      p.obstacle_filtering.trim_trajectory.min_trajectory_length;
    trim_trajectory.braking_distance_scale_factor =
      p.obstacle_filtering.trim_trajectory.braking_distance_scale_factor;

    lateral_margin.nominal_margin = p.obstacle_filtering.lateral_margin.nominal_margin;
    lateral_margin.additional_wheel_off_track_scale =
      p.obstacle_filtering.lateral_margin.additional_wheel_off_track_scale;
    lateral_margin.is_moving_threshold_velocity =
      p.obstacle_filtering.lateral_margin.is_moving_threshold_velocity;
    lateral_margin.additional_is_stop_margin =
      p.obstacle_filtering.lateral_margin.additional_is_stop_margin;
    lateral_margin.additional_is_moving_margin =
      p.obstacle_filtering.lateral_margin.additional_is_moving_margin;

    stop_obstacle_hold_time_threshold = p.obstacle_filtering.stop_obstacle_hold_time_threshold;
  }
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

  PointcloudSegmentationParam() = default;
  explicit PointcloudSegmentationParam(const validator::Params::PointCloudCollisionCheck & p)
  {
    time_series_association.max_time_diff =
      p.pointcloud_segmentation.time_series_association.max_time_diff;
    time_series_association.min_velocity =
      p.pointcloud_segmentation.time_series_association.min_velocity;
    time_series_association.max_velocity =
      p.pointcloud_segmentation.time_series_association.max_velocity;
    time_series_association.position_diff =
      p.pointcloud_segmentation.time_series_association.position_diff;
    velocity_estimation.use_estimated_velocity =
      p.pointcloud_segmentation.velocity_estimation.use_estimated_velocity;
    velocity_estimation.min_clamp_velocity =
      p.pointcloud_segmentation.velocity_estimation.min_clamp_velocity;
    velocity_estimation.max_clamp_velocity =
      p.pointcloud_segmentation.velocity_estimation.max_clamp_velocity;
    velocity_estimation.required_velocity_count =
      static_cast<size_t>(p.pointcloud_segmentation.velocity_estimation.required_velocity_count);
    velocity_estimation.lpf_gain = p.pointcloud_segmentation.velocity_estimation.lpf_gain;
    height_margin.margin_from_bottom = p.pointcloud_segmentation.height_margin.margin_from_bottom;
    height_margin.margin_from_top = p.pointcloud_segmentation.height_margin.margin_from_top;
  }
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

  StopPlanningParam() = default;
  explicit StopPlanningParam(const validator::Params::PointCloudCollisionCheck & p)
  {
    stop_margin = p.stop_planning.stop_margin;
    rss_params.use_rss_stop = p.stop_planning.rss_params.use_rss_stop;
    rss_params.two_wheel_objects_deceleration =
      p.stop_planning.rss_params.two_wheel_objects_deceleration;
    rss_params.vehicle_objects_deceleration =
      p.stop_planning.rss_params.vehicle_objects_deceleration;
    rss_params.no_wheel_objects_deceleration =
      p.stop_planning.rss_params.no_wheel_objects_deceleration;
    rss_params.pointcloud_deceleration = p.stop_planning.rss_params.pointcloud_deceleration;
    rss_params.velocity_offset = p.stop_planning.rss_params.velocity_offset;
    obstacle_velocity_threshold_enter_fixed_stop =
      p.stop_planning.obstacle_velocity_threshold_enter_fixed_stop;
  }
};

}  // namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check

#endif  // FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__PARAMETER_HPP_
