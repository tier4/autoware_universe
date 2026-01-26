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

#include "obstacle_stop.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::rule_based_planner
{

void ObstacleStopModule::init(rclcpp::Node & node, const std::string & module_name)
{
  module_name_ = module_name;
  clock_ = node.get_clock();
  logger_ = node.get_logger().get_child("obstacle_stop");

  // Declare and get parameters
  params_.stop_margin =
    node.declare_parameter<double>("obstacle_stop.stop_margin", params_.stop_margin);
  params_.min_behavior_stop_margin = node.declare_parameter<double>(
    "obstacle_stop.min_behavior_stop_margin", params_.min_behavior_stop_margin);
  params_.obstacle_check_distance = node.declare_parameter<double>(
    "obstacle_stop.obstacle_check_distance", params_.obstacle_check_distance);
  params_.obstacle_velocity_threshold = node.declare_parameter<double>(
    "obstacle_stop.obstacle_velocity_threshold", params_.obstacle_velocity_threshold);

  // Create debug publishers
  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacle_stop/debug_markers", 1);
  virtual_wall_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacle_stop/virtual_walls", 1);

  RCLCPP_INFO(
    logger_, "ObstacleStopModule initialized with stop_margin: %.2f m", params_.stop_margin);
}

void ObstacleStopModule::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & param : parameters) {
    const auto & name = param.get_name();

    if (name == "obstacle_stop.stop_margin") {
      params_.stop_margin = param.as_double();
    } else if (name == "obstacle_stop.min_behavior_stop_margin") {
      params_.min_behavior_stop_margin = param.as_double();
    } else if (name == "obstacle_stop.obstacle_check_distance") {
      params_.obstacle_check_distance = param.as_double();
    } else if (name == "obstacle_stop.obstacle_velocity_threshold") {
      params_.obstacle_velocity_threshold = param.as_double();
    }
  }
}

PlanningResult ObstacleStopModule::plan(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  autoware_utils_debug::ScopedTimeTrack time_track(__func__, *time_keeper_);

  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 5000, "plan() object count: %s",
    planner_data->predicted_objects
      ? std::to_string(planner_data->predicted_objects->objects.size()).c_str()
      : "null");

  // Assume forward driving for simplicity
  constexpr bool is_driving_forward = true;
  const double x_offset_to_bumper =
    calc_x_offset_to_bumper(is_driving_forward, planner_data->vehicle_info);

  // Simple implementation: treat the first predicted object as StopObstacle
  std::vector<StopObstacle> stop_obstacles;
  if (planner_data->predicted_objects && !planner_data->predicted_objects->objects.empty()) {
    const auto & object = planner_data->predicted_objects->objects.front();
    const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;

    // Calculate distance from ego to object along trajectory
    const auto ego_pose = planner_data->current_odometry->pose.pose;
    const double dist_to_object = autoware::motion_utils::calcSignedArcLength(
      trajectory_points, ego_pose.position, object_pose.position);

    // Only consider objects within check distance and in front of ego
    if (dist_to_object > 0.0 && dist_to_object < params_.obstacle_check_distance) {
      StopObstacle stop_obstacle;
      stop_obstacle.collision_point = object_pose.position;
      stop_obstacle.dist_to_collision = dist_to_object;
      stop_obstacles.push_back(stop_obstacle);
    }
  }

  // Plan stop
  const auto stop_point =
    plan_stop(planner_data, trajectory_points, stop_obstacles, x_offset_to_bumper);

  PlanningResult result;
  if (stop_point.has_value()) {
    result.stop_points.push_back(stop_point.value());
  }

  return result;
}

std::optional<geometry_msgs::msg::Point> ObstacleStopModule::plan_stop(
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<StopObstacle> & stop_obstacles, const double x_offset_to_bumper)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (stop_obstacles.empty() || traj_points.empty()) {
    return std::nullopt;
  }

  // Simple implementation: use the first obstacle
  const auto & stop_obstacle = stop_obstacles.front();

  // Calculate stop distance: obstacle distance - stop_margin (6m) - bumper offset
  const double stop_dist =
    stop_obstacle.dist_to_collision - params_.stop_margin - std::abs(x_offset_to_bumper);

  if (stop_dist < 0.0) {
    // Already too close to stop safely, return the first trajectory point
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000, "Obstacle is too close (%.2f m), cannot maintain stop margin",
      stop_obstacle.dist_to_collision);
    return traj_points.front().pose.position;
  }

  // Find the stop point on trajectory at stop_dist from ego
  const auto ego_pose = planner_data->current_odometry->pose.pose;
  const auto stop_point_opt =
    autoware::motion_utils::calcLongitudinalOffsetPoint(traj_points, ego_pose.position, stop_dist);

  if (stop_point_opt) {
    RCLCPP_DEBUG(
      logger_, "Stop point set at %.2f m before obstacle (obstacle at %.2f m)", params_.stop_margin,
      stop_obstacle.dist_to_collision);
    return stop_point_opt;
  }

  return std::nullopt;
}

}  // namespace autoware::rule_based_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::rule_based_planner::ObstacleStopModule,
  autoware::rule_based_planner::PluginModuleInterface)
