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

#ifndef FLAG_NODE_HPP_
#define FLAG_NODE_HPP_

#include "state.hpp"

#include <autoware_utils/ros/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/mode_change_available.hpp>

#include <memory>

namespace autoware::operation_mode_transition_manager
{

class FlagNode : public rclcpp::Node
{
public:
  explicit FlagNode(const rclcpp::NodeOptions & options);

  struct InputData
  {
    Odometry kinematics;
    Trajectory trajectory;
    Control control_cmd;
    Control trajectory_follower_control_cmd;
  };

private:
  using ModeChangeAvailable = tier4_system_msgs::msg::ModeChangeAvailable;
  void on_timer();
  std::optional<InputData> take_data();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ModeChangeAvailable>::SharedPtr pub_transition_available_;
  rclcpp::Publisher<ModeChangeAvailable>::SharedPtr pub_transition_completed_;
  rclcpp::Publisher<ModeChangeBase::DebugInfo>::SharedPtr pub_debug_;

  template <class T>
  using PollingSubscriber = autoware_utils::InterProcessPollingSubscriber<T>;
  PollingSubscriber<Odometry> sub_kinematics_{this, "kinematics"};
  PollingSubscriber<Trajectory> sub_trajectory_{this, "trajectory"};
  PollingSubscriber<Control> sub_control_cmd_{this, "control_cmd"};
  PollingSubscriber<Control> sub_trajectory_follower_control_cmd_{
    this, "trajectory_follower_control_cmd"};

  std::unique_ptr<ModeChangeBase> autonomous_mode_;
};

}  // namespace autoware::operation_mode_transition_manager

#endif  // FLAG_NODE_HPP_
