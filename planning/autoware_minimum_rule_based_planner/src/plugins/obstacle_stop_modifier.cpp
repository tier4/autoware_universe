// Copyright 2025 TIER IV, Inc.
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

#include "obstacle_stop_modifier.hpp"

#include "polygon_utils.hpp"

#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner::plugin
{

namespace trajectory_ns = autoware::experimental::trajectory;

ObstacleStopModifier::ObstacleStopModifier(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper,
  const TrajectoryModifierParams & params)
: TrajectoryModifierPluginBase(name, node_ptr, time_keeper, params),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*node_ptr).getVehicleInfo())
{
  set_up_params();
}

bool ObstacleStopModifier::is_trajectory_modification_required(
  const TrajectoryPoints & traj_points, const TrajectoryModifierParams & /*params*/,
  const TrajectoryModifierData & /*data*/) const
{
  if (traj_points.empty()) {
    return false;
  }
  if (!predicted_objects_) {
    return false;
  }
  if (predicted_objects_->objects.empty()) {
    return false;
  }
  return true;
}

void ObstacleStopModifier::modify_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryModifierParams & params,
  const TrajectoryModifierData & data)
{
  if (!is_trajectory_modification_required(traj_points, params, data)) {
    return;
  }

  // 1. Build Trajectory from input points
  auto traj_opt = trajectory_ns::pretty_build(traj_points);
  if (!traj_opt) {
    return;
  }
  auto & traj = *traj_opt;

  // 2. Find ego position on trajectory
  const double s_ego = trajectory_ns::closest(traj, data.current_odometry.pose.pose);

  // 3. Generate decimated arc length values from ego to end
  const auto decimated_s_values =
    traj.base_arange({s_ego, traj.length()}, params_.decimate_step_length_m);
  if (decimated_s_values.empty()) {
    return;
  }

  // 4. Compute discrete TrajectoryPoints at decimated arc lengths (for polygon collision check)
  const auto decimated_traj_points = traj.compute(decimated_s_values);

  // 5. Init variables
  const double x_offset_to_bumper = vehicle_info_.max_longitudinal_offset_m;

  // 6. Filter obstacles using Trajectory + decimated data
  auto stop_obstacles = filter_stop_obstacle_for_predicted_object(
    data, traj, decimated_traj_points, decimated_s_values, x_offset_to_bumper);

  // 7. (skip point cloud filtering)

  // 8. (no concat needed - only predicted objects)

  // 9. Plan stop (modifies velocity on the Trajectory)
  plan_stop(traj, stop_obstacles, data);

  // 10. Restore: convert Trajectory back to vector<TrajectoryPoint>
  traj_points = traj.restore();
}

std::vector<StopObstacle> ObstacleStopModifier::filter_stop_obstacle_for_predicted_object(
  const TrajectoryModifierData & data, const Trajectory & traj,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::vector<double> & decimated_s_values, const double x_offset_to_bumper)
{
  const auto & current_pose = data.current_odometry.pose.pose;
  const auto predicted_objects_stamp = rclcpp::Time(predicted_objects_->header.stamp);

  // Find ego arc length on main trajectory
  const double s_ego = trajectory_ns::closest(traj, current_pose);

  // Create trajectory polygons for collision check
  const auto decimated_traj_polys = polygon_utils::create_one_step_polygons(
    decimated_traj_points, vehicle_info_, params_.lateral_margin_m);

  std::vector<StopObstacle> stop_obstacles;
  for (const auto & object : predicted_objects_->objects) {
    const auto & obj_pose = object.kinematics.initial_pose_with_covariance.pose;

    // 1. rough filtering
    // 1.1. Check if the obstacle is in front of the ego
    const double s_obj = trajectory_ns::closest(traj, obj_pose);
    const double lon_dist = s_obj - s_ego;
    if (lon_dist < 0.0) {
      continue;
    }

    // 1.2. Check if the obstacle is within detection range
    if (lon_dist > params_.detection_range_m) {
      continue;
    }

    // 2. collision check with trajectory polygons
    const auto obj_polygon = autoware_utils_geometry::to_polygon2d(obj_pose, object.shape);
    const auto collision_point = polygon_utils::get_collision_point(
      decimated_traj_points, decimated_traj_polys, decimated_s_values, obj_pose.position,
      obj_polygon, x_offset_to_bumper);

    if (!collision_point) {
      continue;
    }

    // 3. create StopObstacle (dist is now arc length on main trajectory directly)
    const double obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear.x;
    stop_obstacles.emplace_back(
      object.object_id, predicted_objects_stamp, obj_pose, obj_vel, object.shape,
      collision_point->first, collision_point->second);
  }

  prev_stop_obstacles_ = stop_obstacles;

  return stop_obstacles;
}

void ObstacleStopModifier::plan_stop(
  Trajectory & traj, const std::vector<StopObstacle> & stop_obstacles,
  const TrajectoryModifierData & /*data*/)
{
  if (stop_obstacles.empty()) {
    return;
  }

  const auto closest_stop_obstacles = get_closest_stop_obstacles(stop_obstacles);

  std::optional<StopObstacle> determined_stop_obstacle;
  std::optional<double> determined_zero_vel_dist;

  for (const auto & stop_obstacle : closest_stop_obstacles) {
    // dist_to_collide_on_traj is already arc length on main trajectory
    const double dist_to_collide = stop_obstacle.dist_to_collide_on_traj;

    const double desired_stop_margin = params_.stop_margin_m;
    const double candidate_zero_vel_dist = std::max(0.0, dist_to_collide - desired_stop_margin);

    // select obstacle with smallest stop distance
    if (
      !determined_stop_obstacle ||
      stop_obstacle.dist_to_collide_on_traj < determined_stop_obstacle->dist_to_collide_on_traj) {
      determined_zero_vel_dist = candidate_zero_vel_dist;
      determined_stop_obstacle = stop_obstacle;
    }
  }

  if (!determined_zero_vel_dist || !determined_stop_obstacle) {
    return;
  }

  // Set zero velocity from stop point to end of trajectory
  // This replaces insertStopPoint() + manual velocity loop
  traj.longitudinal_velocity_mps().range(*determined_zero_vel_dist, traj.length()).set(0.0);
}

std::vector<StopObstacle> ObstacleStopModifier::get_closest_stop_obstacles(
  const std::vector<StopObstacle> & stop_obstacles)
{
  if (stop_obstacles.empty()) {
    return {};
  }

  // For minimal implementation, return the single closest obstacle.
  // (In the full version, this returns one per classification type.)
  auto closest_it = std::min_element(
    stop_obstacles.begin(), stop_obstacles.end(),
    [](const StopObstacle & a, const StopObstacle & b) {
      return a.dist_to_collide_on_traj < b.dist_to_collide_on_traj;
    });

  return {*closest_it};
}

void ObstacleStopModifier::set_up_params()
{
  auto * node = get_node_ptr();

  rcl_interfaces::msg::ParameterDescriptor stop_margin_desc;
  stop_margin_desc.description = "Stop margin before obstacle [m]";
  params_.stop_margin_m =
    node->declare_parameter<double>("obstacle_stop.stop_margin_m", 5.0, stop_margin_desc);

  rcl_interfaces::msg::ParameterDescriptor detection_range_desc;
  detection_range_desc.description = "Detection range for obstacles [m]";
  params_.detection_range_m =
    node->declare_parameter<double>("obstacle_stop.detection_range_m", 50.0, detection_range_desc);

  rcl_interfaces::msg::ParameterDescriptor lateral_margin_desc;
  lateral_margin_desc.description = "Lateral margin for obstacle detection [m]";
  params_.lateral_margin_m =
    node->declare_parameter<double>("obstacle_stop.lateral_margin_m", 0.0, lateral_margin_desc);

  rcl_interfaces::msg::ParameterDescriptor decimate_step_desc;
  decimate_step_desc.description = "Step length for trajectory decimation [m]";
  params_.decimate_step_length_m = node->declare_parameter<double>(
    "obstacle_stop.decimate_step_length_m", 2.0, decimate_step_desc);
}

rcl_interfaces::msg::SetParametersResult ObstacleStopModifier::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    update_param<double>(parameters, "obstacle_stop.stop_margin_m", params_.stop_margin_m);
    update_param<double>(parameters, "obstacle_stop.detection_range_m", params_.detection_range_m);
    update_param<double>(parameters, "obstacle_stop.lateral_margin_m", params_.lateral_margin_m);
    update_param<double>(
      parameters, "obstacle_stop.decimate_step_length_m", params_.decimate_step_length_m);
  } catch (const std::exception & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

void ObstacleStopModifier::set_predicted_objects(const PredictedObjects::ConstSharedPtr & objects)
{
  predicted_objects_ = objects;
}

}  // namespace autoware::minimum_rule_based_planner::plugin
