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

#include "autoware/shift_decider/autoware_shift_decider.hpp"

#include <functional>

namespace autoware::shift_decider
{

ShiftDecider::ShiftDecider(const rclcpp::NodeOptions & node_options)
: agnocast::Node("shift_decider", node_options)
{
  park_on_goal_ = declare_parameter<bool>("park_on_goal");

  pub_shift_cmd_ = this->create_publisher<autoware_vehicle_msgs::msg::GearCommand>(
    "output/gear_cmd", rclcpp::QoS{1}.transient_local());

  sub_control_cmd_ =
    this->create_subscription<autoware_control_msgs::msg::Control>("input/control_cmd", 1);
  sub_autoware_state_ =
    this->create_subscription<autoware_system_msgs::msg::AutowareState>("input/state", 1);
  sub_current_gear_ =
    this->create_subscription<autoware_vehicle_msgs::msg::GearReport>("input/current_gear", 1);

  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1));
  timer_ = this->create_wall_timer(period_ns, std::bind(&ShiftDecider::onTimer, this));
}

void ShiftDecider::onTimer()
{
  control_cmd_ = sub_control_cmd_->take_data();
  autoware_state_ = sub_autoware_state_->take_data();
  current_gear_ptr_ = sub_current_gear_->take_data();
  if (!autoware_state_ || !control_cmd_ || !current_gear_ptr_) {
    return;
  }

  updateCurrentShiftCmd();

  auto loaned_msg = pub_shift_cmd_->borrow_loaned_message();
  *loaned_msg = shift_cmd_;
  pub_shift_cmd_->publish(std::move(loaned_msg));
}

void ShiftDecider::updateCurrentShiftCmd()
{
  using autoware_system_msgs::msg::AutowareState;
  using autoware_vehicle_msgs::msg::GearCommand;

  shift_cmd_.stamp = now();
  static constexpr double vel_threshold = 0.01;  // to prevent chattering
  if (autoware_state_->state == AutowareState::DRIVING) {
    if (control_cmd_->longitudinal.velocity > vel_threshold) {
      shift_cmd_.command = GearCommand::DRIVE;
    } else if (control_cmd_->longitudinal.velocity < -vel_threshold) {
      shift_cmd_.command = GearCommand::REVERSE;
    } else {
      shift_cmd_.command = prev_shift_command;
    }
  } else {
    if (
      (autoware_state_->state == AutowareState::ARRIVED_GOAL ||
       autoware_state_->state == AutowareState::WAITING_FOR_ROUTE) &&
      park_on_goal_) {
      shift_cmd_.command = GearCommand::PARK;
    } else {
      shift_cmd_.command = current_gear_ptr_->report;
    }
  }
  prev_shift_command = shift_cmd_.command;
}
}  // namespace autoware::shift_decider

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::shift_decider::ShiftDecider)
