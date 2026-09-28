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

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tier4_system_msgs/msg/driving_mode_flag.hpp>
#include <tier4_system_msgs/msg/driving_mode_info.hpp>
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
  using DrivingModeFlag = tier4_system_msgs::msg::DrivingModeFlag;
  using DrivingModeInfo = tier4_system_msgs::msg::DrivingModeInfo;
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
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

  // Driving mode interface
  AUTOWARE_SUBSCRIPTION_PTR(DrivingModeInfo) sub_driving_mode_info_;
  AUTOWARE_PUBLISHER_PTR(DrivingModeFlag) pub_driving_mode_stable_;
  AUTOWARE_PUBLISHER_PTR(DiagnosticArray) pub_driving_mode_available_;
  void on_driving_mode_info(const DrivingModeInfo & msg);
  void publish_driving_mode_stable(bool flag) const;
  void publish_driving_mode_available(bool flag) const;
  std::optional<uint32_t> driving_mode_id_;  // Refer to the driving_mode_manager for this ID.
};

}  // namespace autoware::operation_mode_transition_manager

#endif  // AUTONOMOUS_MODE_TRANSITION_FLAG_NODE_HPP_
