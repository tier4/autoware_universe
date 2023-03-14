// Copyright 2022 TIER IV, Inc.
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

#include "scene_module/bus_stop/state_machine.hpp"

namespace behavior_velocity_planner
{
namespace bus_stop
{
using State = StateMachine::State;

StateMachine::StateMachine(rclcpp::Node & node, const StateParam & state_param)
: node_(node), state_(State::STOP), state_param_(state_param)
{
  using std::placeholders::_1;

  operation_mode_sub_ = node_.create_subscription<OperationModeState>(
    "/system/operation_mode/state", rclcpp::QoS(1).transient_local(),
    std::bind(&StateMachine::onOperationMode, this, _1));
}

void StateMachine::onOperationMode(const OperationModeState::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_operation_mode_ = *msg;

  //! debug
  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("debug"), "operation_mode: " << static_cast<int>(msg->mode));
}

void StateMachine::updateState(const StateInput & state_input, rclcpp::Clock & clock)
{
  mutex_.lock();
  const auto current_operation_mode = current_operation_mode_;
  mutex_.unlock();

  // KEEP stopping
  if (state_ == State::STOP) {
    if (current_operation_mode.mode == OperationModeState::AUTONOMOUS) {
      state_ = State::READY;
    }
    return;
  }

  // put turn signal and wait for 3 seconds
  if (state_ == State::READY) {
    // start to blink turn signal
    if (!put_turn_signal_time_) {
      put_turn_signal_time_ = std::make_shared<const rclcpp::Time>(clock.now());
    }

    if (current_operation_mode.mode != OperationModeState::AUTONOMOUS) {
      state_ = State::STOP;
      put_turn_signal_time_ = {};
      return;
    }

    const auto time_from_turn_signal = clock.now() - *put_turn_signal_time_;
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("debug"), "time from turn signal: " << time_from_turn_signal.seconds());
    if (time_from_turn_signal.seconds() < state_param_.turn_signal_blinking_duration) {
      // keep READY state
      return;
    }

    // keep READY state if velocity of the approaching obstacle is not safe
    if (!state_input.is_safe_velocty) {
      // last_obstacle_detection_time_ = std::make_shared<const rclcpp::Time>(clock.now());
      return;
    }

    // keep READY state if the obstacle is on the side of the ego vehicle
    if (state_input.is_obstacle_on_the_side) {
      return;
    }

    // time_from_turn_signal > threshold && obstacle velocity is safe
    // keep stopping for a certain time after the obstacle is detected
    // if (last_obstacle_detection_time_) {
    //   const auto time_from_last_detection = clock.now() - *last_obstacle_detection_time_;
    //   RCLCPP_DEBUG_STREAM(
    //     rclcpp::get_logger("debug"),
    //     "time from last detection: " << time_from_last_detection.seconds());
    //   if (time_from_last_detection.seconds() < state_param_.keep_stopping_duration) {
    //     return;
    //   }
    // }

    state_ = State::GO;
    return;
  }

  if (state_ == State::GO) {
    return;
  }
}

}  // namespace bus_stop
}  // namespace behavior_velocity_planner
