// Copyright 2020 Tier IV, Inc.
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

#include "autoware_external_cmd_converter/node.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace autoware::external_cmd_converter
{
ExternalCmdConverterNode::ExternalCmdConverterNode(const rclcpp::NodeOptions & node_options)
: agnocast::Node("external_cmd_converter", node_options)
{
  using std::placeholders::_1;

  cmd_pub_ = this->create_publisher<Control>("out/control_cmd", rclcpp::QoS{1});
  pedals_cmd_sub_ = this->create_subscription<PedalsCommand>(
    "in/pedals_cmd", 1, std::bind(&ExternalCmdConverterNode::on_pedals_cmd, this, _1));
  heartbeat_sub_ = this->create_subscription<ManualOperatorHeartbeat>(
    "in/heartbeat", 1, std::bind(&ExternalCmdConverterNode::on_heartbeat, this, _1));

  // Polling Subscribers
  steering_cmd_sub_ = this->create_subscription<SteeringCommand>("in/steering_cmd", 1);
  velocity_sub_ = this->create_subscription<Odometry>("in/odometry", 1);
  gear_cmd_sub_ = this->create_subscription<GearCommand>("in/gear_cmd", 1);
  gate_mode_sub_ = this->create_subscription<GateMode>("in/current_gate_mode", 1);

  // Parameter
  ref_vel_gain_ = declare_parameter<double>("ref_vel_gain");

  // Parameter for Hz check
  const double timer_rate = declare_parameter<double>("timer_rate");
  wait_for_first_topic_ = declare_parameter<bool>("wait_for_first_topic");
  control_command_timeout_ = declare_parameter<double>("control_command_timeout");
  emergency_stop_timeout_ = declare_parameter<double>("emergency_stop_timeout");

  const auto period_ns = rclcpp::Rate(timer_rate).period();
  rate_check_timer_ =
    this->create_wall_timer(period_ns, std::bind(&ExternalCmdConverterNode::on_timer, this));

  // Parameter for accel/brake map
  const std::string csv_path_accel_map = declare_parameter<std::string>("csv_path_accel_map");
  const std::string csv_path_brake_map = declare_parameter<std::string>("csv_path_brake_map");
  acc_map_initialized_ = true;
  if (!accel_map_.readAccelMapFromCSV(csv_path_accel_map)) {
    RCLCPP_ERROR(
      get_logger(), "Cannot read accelmap. csv path = %s. stop calculation.",
      csv_path_accel_map.c_str());
    acc_map_initialized_ = false;
  }
  if (!brake_map_.readBrakeMapFromCSV(csv_path_brake_map)) {
    RCLCPP_ERROR(
      get_logger(), "Cannot read brakemap. csv path = %s. stop calculation.",
      csv_path_brake_map.c_str());
    acc_map_initialized_ = false;
  }

}

void ExternalCmdConverterNode::on_timer()
{
}

void ExternalCmdConverterNode::on_heartbeat(
  const agnocast::ipc_shared_ptr<ManualOperatorHeartbeat> & msg)
{
  if (msg->ready) {
    latest_heartbeat_received_time_ = std::make_shared<rclcpp::Time>(this->now());
  }
}

void ExternalCmdConverterNode::on_pedals_cmd(
  const agnocast::ipc_shared_ptr<PedalsCommand> & pedals)
{
  // Save received time for rate check
  latest_cmd_received_time_ = std::make_shared<rclcpp::Time>(this->now());

  // take data from subscribers
  current_velocity_ptr_ = velocity_sub_->take_data();
  current_gear_cmd_ = gear_cmd_sub_->take_data();
  const auto steering = steering_cmd_sub_->take_data();

  // Wait for input data
  if (!acc_map_initialized_ || !current_velocity_ptr_ || !current_gear_cmd_ || !steering) {
    return;
  }

  // Calculate reference velocity and acceleration
  const double sign = get_gear_velocity_sign(*current_gear_cmd_);
  const double ref_acceleration =
    calculate_acc(*pedals, std::fabs(current_velocity_ptr_->twist.twist.linear.x));

  if (ref_acceleration > 0.0 && sign == 0.0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Target acceleration is positive, but the gear is not appropriate. accel: %f, gear: %d",
      ref_acceleration, current_gear_cmd_->command);
  }

  double ref_velocity =
    current_velocity_ptr_->twist.twist.linear.x + ref_acceleration * ref_vel_gain_ * sign;
  if (current_gear_cmd_->command == GearCommand::REVERSE) {
    ref_velocity = std::min(0.0, ref_velocity);
  } else if (
    current_gear_cmd_->command == GearCommand::DRIVE ||  // NOLINT
    current_gear_cmd_->command == GearCommand::LOW) {
    ref_velocity = std::max(0.0, ref_velocity);
  } else {
    ref_velocity = 0.0;
    // TODO(Hiroki OTA): ref_acceleration also must be correct value for stopping.
  }

  // Publish ControlCommand
  auto loaned_msg = cmd_pub_->borrow_loaned_message();
  loaned_msg->stamp = pedals->stamp;
  loaned_msg->lateral.steering_tire_angle = steering->steering_tire_angle;
  loaned_msg->lateral.steering_tire_rotation_rate = steering->steering_tire_velocity;
  loaned_msg->longitudinal.velocity = static_cast<float>(ref_velocity);
  loaned_msg->longitudinal.acceleration = static_cast<float>(ref_acceleration);

  cmd_pub_->publish(std::move(loaned_msg));
}

double ExternalCmdConverterNode::calculate_acc(const PedalsCommand & cmd, const double vel)
{
  const double desired_throttle = cmd.throttle;
  const double desired_brake = cmd.brake;
  if (
    std::isnan(desired_throttle) || std::isnan(desired_brake) || std::isinf(desired_throttle) ||
    std::isinf(desired_brake)) {
    std::cerr << "Input brake or throttle is out of range. returning 0.0 acceleration."
              << std::endl;
    return 0.0;
  }

  const double desired_pedal = desired_throttle - desired_brake;

  double ref_acceleration = 0.0;
  if (desired_pedal > 0.0) {
    accel_map_.getAcceleration(desired_pedal, vel, ref_acceleration);
  } else {
    brake_map_.getAcceleration(-desired_pedal, vel, ref_acceleration);
  }

  return ref_acceleration;
}

double ExternalCmdConverterNode::get_gear_velocity_sign(const GearCommand & cmd)
{
  if (cmd.command == GearCommand::DRIVE) {
    return 1.0;
  }
  if (cmd.command == GearCommand::LOW) {
    return 1.0;
  }
  if (cmd.command == GearCommand::REVERSE) {
    return -1.0;
  }

  return 0.0;
}

}  // namespace autoware::external_cmd_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::external_cmd_converter::ExternalCmdConverterNode)
