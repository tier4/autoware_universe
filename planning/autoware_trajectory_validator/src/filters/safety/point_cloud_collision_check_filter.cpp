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
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
namespace pcc = autoware::trajectory_validator::plugin::safety::point_cloud_collision_check;

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
  // moved from  PlannerData constructor
  planner_data_->ego_nearest_dist_threshold = trajectory_polygon_params.ego_nearest_dist_threshold;
  planner_data_->ego_nearest_yaw_threshold = trajectory_polygon_params.ego_nearest_yaw_threshold;
  planner_data_->trajectory_polygon_collision_check = {
    trajectory_polygon_params.decimate_trajectory_step_length,
    trajectory_polygon_params.goal_extended_trajectory_length,
    trajectory_polygon_params.enable_to_consider_current_pose,
    trajectory_polygon_params.time_to_convergence};

  planner_data_->no_ground_pointcloud.preprocess_params_ = params_->preprocess;
  planner_data_->min_accel = params_->common.min_accel;
  planner_data_->min_jerk = params_->common.min_jerk;
}

void PointCloudCollisionCheckFilter::update_planner_data(
  const std::vector<TrajectoryPoint> & raw_trajectory_points, const FilterContext & context)
{
  planner_data_->vehicle_info_ = *vehicle_info_ptr_;
  planner_data_->current_odometry = *context.odometry;
  planner_data_->current_acceleration = *context.acceleration;
  const auto is_driving_forward =
    autoware::motion_utils::isDrivingForwardWithTwist(raw_trajectory_points);
  if (is_driving_forward) {
    planner_data_->is_driving_forward = is_driving_forward.value();
  }
  planner_data_->no_ground_pointcloud.preprocess_pointcloud(
    pcc::convert_pointcloud_to_map_frame(
      *context.segmented_pointcloud, context.odometry->pose.pose, params_->excluded_class_ids),
    raw_trajectory_points, planner_data_->current_odometry,
    planner_data_->calculate_min_deceleration_distance(0.0).value_or(0.0),
    planner_data_->vehicle_info_, planner_data_->trajectory_polygon_collision_check,
    planner_data_->ego_nearest_dist_threshold, planner_data_->ego_nearest_yaw_threshold);
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
  result.is_feasible = judge_stop_feasibility(stop_obstacles, context.odometry->twist.twist);

  if (params_->enable_debug_markers) {
    // judge_stop_feasibility が未実装のため必要制動距離はまだ 0。
    pcc::emit_debug_markers(
      debug_markers_, *debug_data_, *planner_data_, stop_obstacles, 0.0, result.is_feasible,
      candidate_trajectory.generator_id.uuid, rclcpp::Time{context.odometry->header.stamp});
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
