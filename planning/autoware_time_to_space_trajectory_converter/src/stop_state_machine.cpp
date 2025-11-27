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
#include "stop_state_machine.hpp"

#include <cmath>
#include <limits>

namespace autoware::time_to_space_trajectory_converter
{

double calc_s(const SplineData & spline, const geometry_msgs::msg::Point & pos)
{
  if (spline.s.empty()) return 0.0;
  // Simple nearest search (assume standard implementation)
  double min_d2 = std::numeric_limits<double>::max();
  double s = 0.0;
  for (size_t i = 0; i < spline.x.size(); ++i) {
    double dx = spline.x[i] - pos.x;
    double dy = spline.y[i] - pos.y;
    double d2 = dx * dx + dy * dy;
    if (d2 < min_d2) {
      min_d2 = d2;
      s = spline.s[i];
    }
  }
  return s;
}

void StopStateMachine::update(
  const SplineData & spline, const nav_msgs::msg::Odometry & odom, const rclcpp::Time & now)
{
  double dt = (last_update_time_.nanoseconds() > 0) ? (now - last_update_time_).seconds() : 0.0;
  last_update_time_ = now;

  double current_v = std::abs(odom.twist.twist.linear.x);
  double current_s = calc_s(spline, odom.pose.pose.position);
  bool is_stopped = (current_v < stop_vel_threshold);

  int target_idx = -1;
  double target_wait = 0.0;

  for (size_t i = 0; i < spline.s.size(); ++i) {
    if (spline.wait_times[i] > 1e-3) {
      double dist_err = current_s - spline.s[i];
      // Fence Check: -1.0m < err < +3.0m
      if (dist_err > -dist_before && dist_err < dist_after) {
        target_idx = static_cast<int>(i);
        target_wait = spline.wait_times[i];
        break;
      }
    }
  }

  // 1. Departed or Not near stop
  if (target_idx == -1) {
    state_ = State{};  // Reset
    return;
  }

  // 2. New Stop Point
  if (state_.knot_index != target_idx) {
    state_.knot_index = target_idx;
    state_.accumulated_wait = 0.0;
    state_.is_completed = false;
  }

  // 3. Accumulate
  if (!state_.is_completed && is_stopped) {
    state_.accumulated_wait += dt;
    if (state_.accumulated_wait >= target_wait) {
      state_.is_completed = true;
    }
  }
}

}  // namespace autoware::time_to_space_trajectory_converter
