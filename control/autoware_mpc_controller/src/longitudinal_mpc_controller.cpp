// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use it except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/longitudinal_mpc_controller/longitudinal_mpc_controller.hpp"

#include "acados_longitudinal_interface.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_control_msgs/msg/longitudinal.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace autoware::motion::control::longitudinal_mpc_controller
{

LongitudinalMpcController::LongitudinalMpcController(
  rclcpp::Node & node,
  std::shared_ptr<diagnostic_updater::Updater> /* diag_updater */)
: logger_(node.get_logger().get_child("longitudinal_mpc")),
  a_state_initialized_(false),
  a_state_filtered_(0.0),
  last_u_cmd_(0.0)
{
  ctrl_period_ = node.declare_parameter<double>("ctrl_period", 0.03);
  tau_equiv_ = std::max(1e-3, node.declare_parameter<double>("tau_equiv", 0.1));
  delay_compensation_time_ =
    std::max(0.0, node.declare_parameter<double>("delay_compensation_time", 0.1));
  accel_state_lpf_alpha_ =
    node.declare_parameter<double>("accel_state_lpf_alpha", 0.8);
  timeout_trajectory_sec_ =
    node.declare_parameter<double>("timeout_trajectory_sec", 1.0);
  max_acc_ = node.declare_parameter<double>("max_acc", 1.5);
  min_acc_ = node.declare_parameter<double>("min_acc", -5.0);
  traj_resample_dist_ = node.declare_parameter<double>("traj_resample_dist", 0.1);

  const int max_iter = node.declare_parameter<int>("mpc_max_iter", 20);
  const double tol = node.declare_parameter<double>("mpc_tol", 1e-4);
  acados_ = std::make_unique<AcadosLongitudinalInterface>(max_iter, tol);
}

LongitudinalMpcController::~LongitudinalMpcController() = default;

bool LongitudinalMpcController::isReady(const trajectory_follower::InputData & input_data)
{
  if (input_data.current_trajectory.points.size() < 2) {
    return false;
  }
  const auto & traj = input_data.current_trajectory;
  const auto & odom = input_data.current_odometry;
  if (traj.points.empty() || odom.header.stamp.sec == 0) {
    return false;
  }
  return true;
}

trajectory_follower::LongitudinalOutput LongitudinalMpcController::run(
  trajectory_follower::InputData const & input_data)
{
  trajectory_follower::LongitudinalOutput out;
  out.control_cmd.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  out.control_cmd.control_time = out.control_cmd.stamp;
  out.control_cmd.velocity = 0.0f;
  out.control_cmd.acceleration = 0.0f;
  out.control_cmd.jerk = 0.0f;
  out.control_cmd.is_defined_acceleration = true;
  out.control_cmd.is_defined_jerk = false;
  out.control_cmd_horizon.time_step_ms = static_cast<float>(ctrl_period_ * 1000.0);
  out.control_cmd_horizon.controls.clear();

  autoware_planning_msgs::msg::Trajectory traj = input_data.current_trajectory;
  if (traj_resample_dist_ > 1e-3 && traj.points.size() >= 2) {
    try {
      traj = autoware::motion_utils::resampleTrajectory(traj, traj_resample_dist_);
    } catch (const std::exception &) {
      // keep original
    }
  }
  if (traj.points.size() < 2) {
    return out;
  }

  const geometry_msgs::msg::Pose ego_pose = input_data.current_odometry.pose.pose;
  const double v = input_data.current_odometry.twist.twist.linear.x;
  const double a_meas = input_data.current_accel.accel.accel.linear.x;
  if (!a_state_initialized_) {
    a_state_filtered_ = a_meas;
    a_state_initialized_ = true;
    last_u_cmd_ = a_meas;
  } else {
    a_state_filtered_ =
      accel_state_lpf_alpha_ * a_state_filtered_ + (1.0 - accel_state_lpf_alpha_) * a_meas;
  }
  const double a = a_state_filtered_;

  double s0;
  try {
    s0 = autoware::motion_utils::calcSignedArcLength(
      traj.points, static_cast<size_t>(0), ego_pose.position);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(RCL_ROS_TIME), 1000,
      "Longitudinal MPC: calcSignedArcLength failed: %s", e.what());
    return out;
  }

  autoware_planning_msgs::msg::TrajectoryPoint ref_point;
  try {
    ref_point = autoware::motion_utils::calcInterpolatedPoint(traj, ego_pose, false);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(RCL_ROS_TIME), 1000,
      "Longitudinal MPC: calcInterpolatedPoint failed: %s", e.what());
    return out;
  }
  const double v_ref = static_cast<double>(ref_point.longitudinal_velocity_mps);

  std::array<double, 3> x_mpc = {s0, v, a};
  if (delay_compensation_time_ > 0.0) {
    x_mpc = acados_->predictStateAfterDelay(
      x_mpc, delay_compensation_time_, tau_equiv_, last_u_cmd_);
  }

  acados_->setParameters(tau_equiv_);
  acados_->setCostReference(x_mpc[0], v_ref);
  AcadosLongitudinalSolution sol = acados_->getControl(x_mpc);

  double u_cmd = sol.u_cmd;
  if (sol.status != 0) {
    RCLCPP_WARN_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(RCL_ROS_TIME), 500,
      "Longitudinal MPC solve failed: status=%d (using last u_cmd)", sol.status);
    u_cmd = last_u_cmd_;
  }
  u_cmd = std::clamp(u_cmd, min_acc_, max_acc_);
  last_u_cmd_ = u_cmd;

  out.control_cmd.velocity = static_cast<float>(v_ref);
  out.control_cmd.acceleration = static_cast<float>(u_cmd);
  out.control_cmd.is_defined_acceleration = true;
  return out;
}

}  // namespace autoware::motion::control::longitudinal_mpc_controller
