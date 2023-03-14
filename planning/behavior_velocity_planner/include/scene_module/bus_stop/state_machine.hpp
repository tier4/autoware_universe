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

#ifndef SCENE_MODULE__BUS_STOP__STATE_MACHINE_HPP_
#define SCENE_MODULE__BUS_STOP__STATE_MACHINE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>

#include <memory>
#include <string>

namespace behavior_velocity_planner
{
namespace bus_stop
{
using autoware_adapi_v1_msgs::msg::OperationModeState;
using tier4_autoware_utils::kmph2mps;

class StateMachine
{
public:
  enum class State {
    STOP = 0,
    READY,
    GO,
  };

  struct StateParam
  {
    double turn_signal_blinking_duration;
  };

  struct StateInput
  {
    bool is_safe_velocity;
    bool is_obstacle_on_the_side;
  };

  explicit StateMachine(rclcpp::Node & node, const StateParam & state_param);
  ~StateMachine() {}
  State getCurrentState() const { return state_; }
  void updateState(const StateInput & state_input, rclcpp::Clock & clock);

private:
  // Subscriber
  rclcpp::Subscription<OperationModeState>::SharedPtr operation_mode_sub_;
  void onOperationMode(OperationModeState::ConstSharedPtr msg);

  rclcpp::Node & node_;
  State state_;
  StateParam state_param_;

  OperationModeState current_operation_mode_;
  // Time elapsed since the start of the blinking of turn signal
  std::shared_ptr<const rclcpp::Time> put_turn_signal_time_;
  std::shared_ptr<const rclcpp::Time> last_obstacle_detection_time_;

  // mutex for current_operation_mode_
  std::mutex mutex_;
};

}  // namespace bus_stop
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__BUS_STOP__STATE_MACHINE_HPP_
