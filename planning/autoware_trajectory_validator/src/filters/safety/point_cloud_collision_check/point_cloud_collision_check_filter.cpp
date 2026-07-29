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

#include "autoware/trajectory_validator/filters/safety/point_cloud_collision_check_filter.hpp"

#include "point_cloud_collision_check/debug_marker.hpp"
#include "point_cloud_collision_check/obstacle_stop.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <cmath>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
namespace pcc = autoware::trajectory_validator::plugin::safety::point_cloud_collision_check;

namespace
{
// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:45-53
// 一定加減速度で停止するまでに必要な最小距離。
double calc_minimum_distance_to_stop(
  const double initial_vel, const double max_acc, const double min_acc)
{
  if (initial_vel < 0.0) {
    return -std::pow(initial_vel, 2) / 2.0 / max_acc;
  }
  return -std::pow(initial_vel, 2) / 2.0 / min_acc;
}
}  // namespace

PointCloudCollisionCheckFilter::PointCloudCollisionCheckFilter()
: ValidatorInterface("point_cloud_collision_check_filter"),
  obstacle_stop_(std::make_unique<pcc::ObstacleStop>()),
  params_(std::make_unique<pcc::Params>()),
  debug_data_(std::make_unique<pcc::DebugData>()),
  planner_data_(std::make_shared<pcc::PlannerData>())
{
}

PointCloudCollisionCheckFilter::~PointCloudCollisionCheckFilter() = default;

bool PointCloudCollisionCheckFilter::is_available_data(
  const CandidateTrajectory & candidate_trajectory, const FilterContext & context) const
{
  if (
    !context.odometry || !context.acceleration || !vehicle_info_ptr_ ||
    !context.segmented_pointcloud) {
    return false;
  }
  // odometory can transform only map to baselink
  if (context.segmented_pointcloud->header.frame_id != "base_link") {
    return false;
  }
  // The decimation spline collapses duplicate points (dx, dy < 1e-6) and needs >= 2 unique points
  // (autoware_interpolation/src/spline_interpolation_points_2d.cpp:49-67).
  size_t unique_point_num = 0;
  for (size_t i = 0; i < candidate_trajectory.points.size(); ++i) {
    if (i > 0) {
      const auto & prev = candidate_trajectory.points.at(i - 1).pose.position;
      const auto & curr = candidate_trajectory.points.at(i).pose.position;
      if (std::abs(curr.x - prev.x) < 1e-6 && std::abs(curr.y - prev.y) < 1e-6) {
        continue;
      }
    }
    ++unique_point_num;
  }
  return unique_point_num >= 2;
}

void PointCloudCollisionCheckFilter::set_planner_data_param()
{
  const auto & trajectory_polygon_params = params_->trajectory_polygon;

  // motion_velocity_planner_common/planner_data.cpp:239-260
  planner_data_->ego_nearest_dist_threshold = trajectory_polygon_params.ego_nearest_dist_threshold;
  planner_data_->ego_nearest_yaw_threshold = trajectory_polygon_params.ego_nearest_yaw_threshold;
  planner_data_->trajectory_polygon_collision_check = {
    trajectory_polygon_params.decimate_trajectory_step_length,
    trajectory_polygon_params.goal_extended_trajectory_length,
    trajectory_polygon_params.enable_to_consider_current_pose,
    trajectory_polygon_params.time_to_convergence};

  // motion_velocity_planner_common/planner_data.hpp:88
  planner_data_->no_ground_pointcloud.preprocess_params_ = params_->preprocess;

  // motion_velocity_planner/node.cpp:262-266（set_velocity_smoother_params）
  planner_data_->min_accel = params_->common.min_accel;
  planner_data_->min_jerk = params_->common.min_jerk;
}

void PointCloudCollisionCheckFilter::update_planner_data(
  const std::vector<TrajectoryPoint> & raw_trajectory_points, const FilterContext & context)
{
  // motion_velocity_planner_common/planner_data.cpp:240-241
  planner_data_->vehicle_info_ = *vehicle_info_ptr_;

  // motion_velocity_planner/node.cpp:160-167
  planner_data_->current_odometry = *context.odometry;
  planner_data_->current_acceleration = *context.acceleration;

  // motion_velocity_planner/node.cpp:211-215
  const auto is_driving_forward =
    autoware::motion_utils::isDrivingForwardWithTwist(raw_trajectory_points);
  if (is_driving_forward) {
    planner_data_->is_driving_forward = is_driving_forward.value();
  }

  // motion_velocity_planner/node.cpp:176-195
  planner_data_->no_ground_pointcloud.preprocess_pointcloud(
    pcc::convert_pointcloud_to_map_frame(
      *context.segmented_pointcloud, context.odometry->pose.pose, params_->excluded_class_ids),
    raw_trajectory_points, planner_data_->current_odometry,
    planner_data_->calculate_min_deceleration_distance(0.0).value_or(0.0),
    planner_data_->vehicle_info_, planner_data_->trajectory_polygon_collision_check,
    planner_data_->ego_nearest_dist_threshold, planner_data_->ego_nearest_yaw_threshold);
}

bool PointCloudCollisionCheckFilter::judge_stop_feasibility(
  const std::vector<pcc::StopObstacle> & stop_obstacles, const geometry_msgs::msg::Twist & twist,
  double & required_distance) const
{
  // 最も手前の衝突距離（RSS 時は障害物制動距離を加算）を取る。
  std::optional<double> nearest_dist_to_collide;
  for (const auto & stop_obstacle : stop_obstacles) {
    const double dist_to_collide =
      stop_obstacle.dist_to_collide_on_decimated_traj + stop_obstacle.braking_dist.value_or(0.0);
    if (!nearest_dist_to_collide.has_value() || dist_to_collide < *nearest_dist_to_collide) {
      nearest_dist_to_collide = dist_to_collide;
    }
  }

  required_distance = calc_minimum_distance_to_stop(
                        twist.linear.x, params_->common.max_accel, params_->common.min_accel) +
                      params_->stop_planning.stop_margin;

  // 衝突距離が必要制動距離 + stop_margin を下回れば STOP REQUIRED（infeasible）。
  if (nearest_dist_to_collide.has_value() && *nearest_dist_to_collide < required_distance) {
    return false;
  }
  return true;
}

PointCloudCollisionCheckFilter::result_t PointCloudCollisionCheckFilter::is_feasible(
  const CandidateTrajectory & candidate_trajectory, const FilterContext & context)
{
  // memo: assume trajectory_selector subscribes
  // "/perception/obstacle_segmentation/filtered_pointcloud" (X2 post-filter) or
  // "/perception/obstacle_segmentation/pointcloud" / "/perception/segmented/pointcloud"

  if (!is_available_data(candidate_trajectory, context)) {
    return ValidationResult{};
  }

  update_planner_data(candidate_trajectory.points, context);

  std::vector<pcc::StopObstacle> stop_obstacles;
  try {
    stop_obstacles = obstacle_stop_->calc_obstacle_stop(candidate_trajectory.points, planner_data_);
  } catch (const std::exception &) {
    return ValidationResult{};
  }

  ValidationResult result{};
  double required_distance = 0.0;
  result.is_feasible =
    judge_stop_feasibility(stop_obstacles, context.odometry->twist.twist, required_distance);

  if (params_->enable_debug_markers) {
    pcc::emit_debug_markers(
      debug_markers_, *debug_data_, *planner_data_, stop_obstacles, required_distance,
      result.is_feasible, candidate_trajectory.generator_id.uuid,
      rclcpp::Time{context.odometry->header.stamp});
  }

  return result;
}

void PointCloudCollisionCheckFilter::update_parameters(const validator::Params & params)
{
  *params_ = pcc::Params{params.point_cloud_collision_check};
  obstacle_stop_->update_parameters(*params_);
  set_planner_data_param();
}
}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::PointCloudCollisionCheckFilter,
  autoware::trajectory_validator::plugin::ValidatorInterface)
