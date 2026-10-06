// Copyright 2025 The Autoware Contributors
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

#ifndef COMMON__VEHICLE_STATUS_HPP_
#define COMMON__VEHICLE_STATUS_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace autoware::control_command_gate
{

using autoware_control_msgs::msg::Control;
using autoware_vehicle_msgs::msg::ControlModeReport;
using autoware_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

class VehicleStatus
{
public:
  explicit VehicleStatus(autoware::agnocast_wrapper::Node & node);
  bool is_autoware_control_enabled() const;
  bool is_vehicle_stopped() const;
  double get_current_steering() const;
  double get_current_velocity() const;
  double get_current_acceleration() const;
  Control get_actual_status_as_command() const;

private:
  static constexpr double velocity_buffer_time_sec = 10.0;

  void on_kinematics(const Odometry & msg);

  autoware::agnocast_wrapper::Node & node_;
  AUTOWARE_SUBSCRIPTION_PTR(Odometry) sub_kinematics_;
  AUTOWARE_SUBSCRIPTION_PTR(AccelWithCovarianceStamped) sub_acceleration_;
  AUTOWARE_SUBSCRIPTION_PTR(SteeringReport) sub_steering_;
  AUTOWARE_SUBSCRIPTION_PTR(ControlModeReport) sub_control_mode_;

  // Not VehicleStopChecker: it owns an rclcpp subscription, which an AgnocastOnly executor does
  // not spin. The base holds only the twist buffer, which sub_kinematics_ feeds.
  autoware::motion_utils::VehicleStopCheckerBase vehicle_stop_checker_;

  Odometry current_kinematics_;
  AccelWithCovarianceStamped current_acceleration_;
  SteeringReport current_steering_;
  ControlModeReport current_control_mode_;

  double stop_check_duration_;
};

}  // namespace autoware::control_command_gate

#endif  // COMMON__VEHICLE_STATUS_HPP_
