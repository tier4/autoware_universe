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

#ifndef AUTONOMOUS_MODE_TRANSITION_FLAG_NODE_HPP_
#define AUTONOMOUS_MODE_TRANSITION_FLAG_NODE_HPP_

#include "state.hpp"

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/mode_change_available.hpp>

#include <memory>

namespace autoware::operation_mode_transition_manager
{

class AutonomousModeTransitionFlagNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit AutonomousModeTransitionFlagNode(const rclcpp::NodeOptions & options);

private:
  using ModeChangeAvailable = tier4_system_msgs::msg::ModeChangeAvailable;
  void on_timer();
  InputData take_data();

  AUTOWARE_TIMER_PTR timer_;
  AUTOWARE_PUBLISHER_PTR(ModeChangeAvailable) pub_transition_available_;
  AUTOWARE_PUBLISHER_PTR(ModeChangeAvailable) pub_transition_completed_;
  AUTOWARE_PUBLISHER_PTR(ModeChangeBase::DebugInfo) pub_debug_;

  template <class T>
  using PollingSubscriber = autoware::agnocast_wrapper::polling::PollingSubscriber<T>;
  PollingSubscriber<Odometry>::SharedPtr sub_kinematics_;
  PollingSubscriber<Trajectory>::SharedPtr sub_trajectory_;
  PollingSubscriber<Control>::SharedPtr sub_control_cmd_;
  PollingSubscriber<Control>::SharedPtr sub_trajectory_follower_control_cmd_;

  std::unique_ptr<ModeChangeBase> autonomous_mode_;
};

}  // namespace autoware::operation_mode_transition_manager

#endif  // AUTONOMOUS_MODE_TRANSITION_FLAG_NODE_HPP_
