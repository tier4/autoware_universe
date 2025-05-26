//  Copyright 2025 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "flag_node.hpp"

#include <memory>

namespace autoware::operation_mode_transition_manager
{

FlagNode::FlagNode(const rclcpp::NodeOptions & options) : Node("flag_node", options)
{
  declare_parameter<double>("stable_check.duration");
  autonomous_mode_ = std::make_unique<AutonomousMode>(this);

  pub_transition_available_ =
    create_publisher<ModeChangeAvailable>("/system/command_mode/transition/available", 1);
  pub_transition_completed_ =
    create_publisher<ModeChangeAvailable>("/system/command_mode/transition/completed", 1);

  pub_debug_ = create_publisher<ModeChangeBase::DebugInfo>("~/debug_info", 1);

  const auto period = rclcpp::Rate(declare_parameter<double>("frequency_hz")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void FlagNode::on_timer()
{
  const auto publish = [](auto pub, rclcpp::Time stamp, bool value) {
    ModeChangeAvailable msg;
    msg.stamp = stamp;
    msg.available = value;
    pub->publish(msg);
  };

  const auto input = take_data();
  if (!input) {
    return;
  }

  const bool is_available = autonomous_mode_->isModeChangeAvailable(
    input->kinematics, input->trajectory, input->trajectory_follower_control_cmd,
    input->control_cmd);
  const bool is_completed =
    autonomous_mode_->isModeChangeCompleted(input->kinematics, input->trajectory);

  const auto stamp = get_clock()->now();
  publish(pub_transition_available_, stamp, is_available);
  publish(pub_transition_completed_, stamp, is_completed);

  ModeChangeBase::DebugInfo debug = autonomous_mode_->getDebugInfo();
  debug.stamp = stamp;
  pub_debug_->publish(debug);
}

std::optional<FlagNode::InputData> FlagNode::take_data()
{
  const auto kinematics = sub_kinematics_.take_data();
  if (!kinematics) {
    return std::nullopt;
  }

  const auto trajectory = sub_trajectory_.take_data();
  if (!trajectory) {
    return std::nullopt;
  }

  const auto control_cmd = sub_control_cmd_.take_data();
  if (!control_cmd) {
    return std::nullopt;
  }

  const auto trajectory_follower_control_cmd = sub_trajectory_follower_control_cmd_.take_data();
  if (!trajectory_follower_control_cmd) {
    return std::nullopt;
  }

  InputData data;
  data.kinematics = *kinematics;
  data.trajectory = *trajectory;
  data.control_cmd = *control_cmd;
  data.trajectory_follower_control_cmd = *trajectory_follower_control_cmd;
  return data;
}

}  // namespace autoware::operation_mode_transition_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::operation_mode_transition_manager::FlagNode)
