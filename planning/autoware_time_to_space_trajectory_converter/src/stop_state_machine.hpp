// Copyright 2025 TIER IV, Inc.
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
#ifndef STOP_STATE_MACHINE_HPP_
#define STOP_STATE_MACHINE_HPP_

#include "data_types.hpp"

#include <rclcpp/time.hpp>

#include <nav_msgs/msg/odometry.hpp>

namespace autoware::time_to_space_trajectory_converter
{

class StopStateMachine
{
public:
  struct State
  {
    int knot_index = -1;
    double accumulated_wait = 0.0;
    bool is_completed = false;
  };

  void update(
    const SplineData & spline, const nav_msgs::msg::Odometry & odom, const rclcpp::Time & now);

  [[nodiscard]] State get_state() const { return state_; }

  // Helpers for testing
  void reset()
  {
    state_ = State{};
    last_update_time_ = rclcpp::Time(0);
  }

private:
  State state_;
  rclcpp::Time last_update_time_{0};

  // Robustness Config
  static constexpr double stop_vel_threshold = 0.1;
  static constexpr double dist_before = 1.0;
  static constexpr double dist_after = 3.0;  // Overrun tolerance
};
}  // namespace autoware::time_to_space_trajectory_converter
#endif  // STOP_STATE_MACHINE_HPP_
