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

#include "point_cloud_collision_check_filter.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <rclcpp/time.hpp>

#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{

namespace pcc = point_cloud_collision_check;

using point_cloud_collision_check::emit_debug_markers;
using point_cloud_collision_check::PointcloudPreprocessParams;
using point_cloud_collision_check::process_no_ground_pointcloud;

bool PointCloudCollisionCheckFilter::is_available_data(const FilterContext & context) const
{
  return context.odometry && context.acceleration && vehicle_info_ptr_ &&
         context.segmented_pointcloud && context.tf_buffer && context.clock;
}

// motion_velocity_planner_common/planner_data.cpp:239-260
// および motion_velocity_planner/node.cpp:262-266（set_velocity_smoother_params）
void PointCloudCollisionCheckFilter::set_planner_data_param(
  const validator::Params::PointCloudCollisionCheck & p)
{
  planner_data_.ego_nearest_dist_threshold = p.trajectory_polygon.ego_nearest_dist_threshold;
  planner_data_.ego_nearest_yaw_threshold = p.trajectory_polygon.ego_nearest_yaw_threshold;
  planner_data_.trajectory_polygon_collision_check = {
    p.trajectory_polygon.decimate_trajectory_step_length,
    p.trajectory_polygon.goal_extended_trajectory_length,
    p.trajectory_polygon.enable_to_consider_current_pose, p.trajectory_polygon.time_to_convergence};

  planner_data_.no_ground_pointcloud.preprocess_params_ = PointcloudPreprocessParams{p};

  // motion_velocity_planner/node.cpp:262-266（set_velocity_smoother_params）の代替
  planner_data_.velocity_smoother_.min_decel = p.common.min_accel;
  planner_data_.velocity_smoother_.min_jerk = p.common.min_jerk;
}

bool PointCloudCollisionCheckFilter::update_planner_data(
  const std::vector<TrajectoryPoint> & raw_trajectory_points, const FilterContext & context)
{
  // motion_velocity_planner_common/planner_data.cpp:240-241
  planner_data_.vehicle_info_ = *vehicle_info_ptr_;

  // motion_velocity_planner/node.cpp:160-167
  planner_data_.current_odometry = *context.odometry;
  planner_data_.current_acceleration = *context.acceleration;

  // motion_velocity_planner/node.cpp:211-215
  const auto is_driving_forward =
    autoware::motion_utils::isDrivingForwardWithTwist(raw_trajectory_points);
  if (is_driving_forward) {
    planner_data_.is_driving_forward = is_driving_forward.value();
  }

  // motion_velocity_planner/node.cpp:176-195
  auto no_ground_pointcloud =
    process_no_ground_pointcloud(context.segmented_pointcloud, *context.tf_buffer, context.clock);
  if (!no_ground_pointcloud) {
    return false;
  }
  planner_data_.no_ground_pointcloud.preprocess_pointcloud(
    std::move(*no_ground_pointcloud), raw_trajectory_points, planner_data_.current_odometry,
    planner_data_.calculate_min_deceleration_distance(0.0).value_or(0.0),
    planner_data_.vehicle_info_, planner_data_.trajectory_polygon_collision_check,
    planner_data_.ego_nearest_dist_threshold, planner_data_.ego_nearest_yaw_threshold);
  return true;
}

bool PointCloudCollisionCheckFilter::judge_stop_feasibility(
  [[maybe_unused]] const std::vector<pcc::StopObstacle> & stop_obstacles,
  [[maybe_unused]] const geometry_msgs::msg::Twist & twist) const
{
  // It is not currently implemented. always return true.
  bool is_feasible = true;
  return is_feasible;
}

PointCloudCollisionCheckFilter::result_t PointCloudCollisionCheckFilter::is_feasible(
  const CandidateTrajectory & candidate_trajectory, const FilterContext & context)
{
  // memo: assume trajectory_selector subscribes "/perception/obstacle_segmentation/pointcloud" or
  // "/perception/segmented/pointcloud" that was published from ptv3 node

  if (!is_available_data(context)) {
    return ValidationResult{};
  }

  update_planner_data(candidate_trajectory.points, context);

  // const auto stop_obstacles = calc_obstacle_stop(candidate_trajectory.points, planner_data_);

  ValidationResult result{};

  // result.is_feasible = judge_stop_feasibility(stop_obstacles, context.odometry->twist.twist);
  result.is_feasible = true;  // Placeholder - replace with actual stop feasibility judgment

  // On this branch, PCC debug markers are always enabled.
  emit_debug_markers(
    debug_markers_, debug_data_, planner_data_, result.is_feasible,
    rclcpp::Time{context.odometry->header.stamp});

  return result;
}

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:222-228
// （init のうちパラメータ構築部）
void PointCloudCollisionCheckFilter::update_parameters(const validator::Params & params)
{
  const auto & p = params.point_cloud_collision_check;

  // common_param_ = CommonParam{p};
  // stop_planning_param_ = StopPlanningParam{p};
  // obstacle_filtering_params_ = {
  //   {StopObstacleClassification::Type::POINTCLOUD, ObstacleFilteringParam{p}}};
  // pointcloud_segmentation_param_ = PointcloudSegmentationParam{p};
  set_planner_data_param(p);
}
}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::PointCloudCollisionCheckFilter,
  autoware::trajectory_validator::plugin::ValidatorInterface)
