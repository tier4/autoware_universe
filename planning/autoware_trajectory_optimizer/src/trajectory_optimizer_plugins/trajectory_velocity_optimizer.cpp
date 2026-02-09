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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_velocity_optimizer.hpp"

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_velocity_optimizer_utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

void TrajectoryVelocityOptimizer::set_up_velocity_smoother(
  rclcpp::Node * node_ptr, const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
{
  const auto vehicle_info =
    autoware::vehicle_info_utils::VehicleInfoUtils(*node_ptr).getVehicleInfo();
  double wheelbase = vehicle_info.wheel_base_m;  // vehicle_info.wheel_base_m;
  jerk_filtered_smoother_ = std::make_shared<JerkFilteredSmoother>(*node_ptr, time_keeper);
  jerk_filtered_smoother_->setWheelBase(wheelbase);
}

std::optional<TrajectoryPoint> TrajectoryVelocityOptimizer::calcProjectedTrajectoryPointFromEgo(
  const TrajectoryPoints & trajectory, const geometry_msgs::msg::Pose & ego_pose) const
{
  if (trajectory.size() < 2) {
    return std::nullopt;
  }

  const auto seg_idx = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    trajectory, ego_pose, velocity_params_.nearest_dist_threshold_m,
    autoware_utils_math::deg2rad(velocity_params_.nearest_yaw_threshold_deg));

  const auto & p0 = trajectory.at(seg_idx);
  const auto & p1 = trajectory.at(seg_idx + 1);

  const double dx = p1.pose.position.x - p0.pose.position.x;
  const double dy = p1.pose.position.y - p0.pose.position.y;
  const double seg_len_sq = dx * dx + dy * dy;

  if (seg_len_sq < 1e-10) {
    return p0;
  }

  const double ex = ego_pose.position.x - p0.pose.position.x;
  const double ey = ego_pose.position.y - p0.pose.position.y;
  const double ratio = std::clamp((ex * dx + ey * dy) / seg_len_sq, 0.0, 1.0);

  TrajectoryPoint result = p0;
  result.longitudinal_velocity_mps = static_cast<float>(
    p0.longitudinal_velocity_mps +
    ratio * (p1.longitudinal_velocity_mps - p0.longitudinal_velocity_mps));
  result.acceleration_mps2 = static_cast<float>(
    p0.acceleration_mps2 + ratio * (p1.acceleration_mps2 - p0.acceleration_mps2));

  return result;
}

void TrajectoryVelocityOptimizer::optimize_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
  const TrajectoryOptimizerData & data)
{
  if (!params.use_velocity_optimizer) {
    return;
  }

  const auto & current_odometry = data.current_odometry;
  const auto & current_acceleration = data.current_acceleration;
  const auto & current_speed = current_odometry.twist.twist.linear.x;
  const auto & current_linear_acceleration = current_acceleration.accel.accel.linear.x;
  const double & target_pull_out_speed_mps = velocity_params_.target_pull_out_speed_mps;
  const double & target_pull_out_acc_mps2 = velocity_params_.target_pull_out_acc_mps2;
  const double & max_speed_mps = velocity_params_.max_speed_mps;

  if (velocity_params_.limit_lateral_acceleration) {
    trajectory_velocity_optimizer_utils::limit_lateral_acceleration(
      traj_points, velocity_params_.max_lateral_accel_mps2, data.current_odometry);
  }

  // --- Feature 1: Compute initial motion from prev_output_ for temporal consistency ---
  auto initial_motion_speed = current_speed;
  auto initial_motion_acc = current_linear_acceleration;

  if (current_speed < target_pull_out_speed_mps) {
    // Vehicle is slow (starting/pull-out): use pull-out parameters
    initial_motion_speed = target_pull_out_speed_mps;
    initial_motion_acc = target_pull_out_acc_mps2;
  } else if (!prev_output_.empty()) {
    // Vehicle is moving: use prev_output for temporal consistency when tracking is good
    const auto projected =
      calcProjectedTrajectoryPointFromEgo(prev_output_, current_odometry.pose.pose);
    if (projected) {
      const double desired_vel = std::fabs(projected->longitudinal_velocity_mps);
      const double desired_acc = projected->acceleration_mps2;
      constexpr double replan_vel_deviation = 3.0;  // [m/s]
      if (std::fabs(current_speed - desired_vel) <= replan_vel_deviation) {
        initial_motion_speed = desired_vel;
        initial_motion_acc = desired_acc;
      }
    }
  }

  if (velocity_params_.set_engage_speed && (current_speed < target_pull_out_speed_mps)) {
    // Check distance to stop point to avoid engaging when obstacle is ahead
    // (same logic as autoware_velocity_smoother)
    const auto closest_idx =
      autoware::motion_utils::findNearestIndex(traj_points, current_odometry.pose.pose.position);
    const auto zero_vel_idx =
      autoware::motion_utils::searchZeroVelocityIndex(traj_points, closest_idx, traj_points.size());

    bool should_engage = true;
    if (zero_vel_idx) {
      const double stop_dist =
        autoware::motion_utils::calcSignedArcLength(traj_points, closest_idx, *zero_vel_idx);
      if (stop_dist <= velocity_params_.stop_dist_to_prohibit_engage) {
        // Stop point is too close, do not apply engage speed
        should_engage = false;
      }
    }

    if (should_engage) {
      trajectory_velocity_optimizer_utils::clamp_velocities(
        traj_points, static_cast<float>(initial_motion_speed),
        static_cast<float>(initial_motion_acc));
    }
  }

  if (velocity_params_.limit_speed) {
    trajectory_velocity_optimizer_utils::set_max_velocity(
      traj_points, static_cast<float>(max_speed_mps));
  }

  if (velocity_params_.smooth_velocities) {
    if (!jerk_filtered_smoother_) {
      set_up_velocity_smoother(get_node_ptr(), get_time_keeper());
    }
    InitialMotion initial_motion{initial_motion_speed, initial_motion_acc};
    trajectory_velocity_optimizer_utils::filter_velocity(
      traj_points, initial_motion, velocity_params_.nearest_dist_threshold_m,
      autoware_utils_math::deg2rad(velocity_params_.nearest_yaw_threshold_deg),
      jerk_filtered_smoother_, current_odometry);

    // Save output for temporal consistency in next cycle
    if (!traj_points.empty()) {
      prev_output_ = traj_points;
    }
  }
}

void TrajectoryVelocityOptimizer::set_up_params()
{
  auto node_ptr = get_node_ptr();
  using autoware_utils_rclcpp::get_or_declare_parameter;

  velocity_params_.nearest_dist_threshold_m = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.nearest_dist_threshold_m");
  velocity_params_.nearest_yaw_threshold_deg = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.nearest_yaw_threshold_deg");
  velocity_params_.target_pull_out_speed_mps = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.target_pull_out_speed_mps");
  velocity_params_.target_pull_out_acc_mps2 = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.target_pull_out_acc_mps2");
  velocity_params_.max_speed_mps =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_velocity_optimizer.max_speed_mps");
  velocity_params_.max_lateral_accel_mps2 = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.max_lateral_accel_mps2");
  velocity_params_.stop_dist_to_prohibit_engage = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.stop_dist_to_prohibit_engage");
  velocity_params_.set_engage_speed =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_velocity_optimizer.set_engage_speed");
  velocity_params_.limit_speed =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_velocity_optimizer.limit_speed");
  velocity_params_.limit_lateral_acceleration = get_or_declare_parameter<bool>(
    *node_ptr, "trajectory_velocity_optimizer.limit_lateral_acceleration");
  velocity_params_.smooth_velocities =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_velocity_optimizer.smooth_velocities");
}

rcl_interfaces::msg::SetParametersResult TrajectoryVelocityOptimizer::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  update_param(
    parameters, "trajectory_velocity_optimizer.nearest_dist_threshold_m",
    velocity_params_.nearest_dist_threshold_m);
  update_param(
    parameters, "trajectory_velocity_optimizer.nearest_yaw_threshold_deg",
    velocity_params_.nearest_yaw_threshold_deg);
  update_param(
    parameters, "trajectory_velocity_optimizer.target_pull_out_speed_mps",
    velocity_params_.target_pull_out_speed_mps);
  update_param(
    parameters, "trajectory_velocity_optimizer.target_pull_out_acc_mps2",
    velocity_params_.target_pull_out_acc_mps2);
  update_param(
    parameters, "trajectory_velocity_optimizer.max_speed_mps", velocity_params_.max_speed_mps);
  update_param(
    parameters, "trajectory_velocity_optimizer.max_lateral_accel_mps2",
    velocity_params_.max_lateral_accel_mps2);
  update_param(
    parameters, "trajectory_velocity_optimizer.stop_dist_to_prohibit_engage",
    velocity_params_.stop_dist_to_prohibit_engage);
  update_param(
    parameters, "trajectory_velocity_optimizer.set_engage_speed",
    velocity_params_.set_engage_speed);
  update_param(
    parameters, "trajectory_velocity_optimizer.limit_speed", velocity_params_.limit_speed);
  update_param(
    parameters, "trajectory_velocity_optimizer.limit_lateral_acceleration",
    velocity_params_.limit_lateral_acceleration);
  update_param(
    parameters, "trajectory_velocity_optimizer.smooth_velocities",
    velocity_params_.smooth_velocities);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::trajectory_optimizer::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_optimizer::plugin::TrajectoryVelocityOptimizer,
  autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase)
