// Copyright 2026 TIER IV, Inc.
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

#include "autoware/diffusion_planner/postprocessing/stop_manager.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace autoware::diffusion_planner::postprocess
{

bool StopManager::update(
  const rclcpp::Time & stamp, const std::vector<float> & planned_velocity_profile,
  const double stopping_threshold, const double dt)
{
  const double current_velocity_mps = previous_plan_first_velocity_mps_;
  previous_plan_first_velocity_mps_ =
    planned_velocity_profile.empty() ? 0.0f : planned_velocity_profile.front();

  const bool is_ego_stopped = std::abs(current_velocity_mps) <= stopping_threshold;
  if (!is_ego_stopped) {
    reset();
    return true;
  }

  // if all velocities are below the threshold, then the vehicle is considered stopped
  if (std::all_of(
        planned_velocity_profile.begin(), planned_velocity_profile.end(),
        [stopping_threshold](const float v) { return std::abs(v) <= stopping_threshold; })) {
    reset();
    return true;
  }

  if (!is_tracking_) {
    is_tracking_ = true;
    stop_start_time_ = stamp;
    max_departure_time_s_ = 0.0;
  }

  const float threshold = static_cast<float>(stopping_threshold);
  for (size_t i = 0; i < planned_velocity_profile.size(); ++i) {
    if (planned_velocity_profile[i] > threshold) {
      const double departure_time_s = static_cast<double>(i) * dt;
      max_departure_time_s_ = std::max(max_departure_time_s_, departure_time_s);
      break;
    }
  }

  const double stopped_duration_s = (stamp - stop_start_time_).seconds();
  return stopped_duration_s <= max_departure_time_s_;
}

void StopManager::reset()
{
  is_tracking_ = false;
  max_departure_time_s_ = 0.0;
}

}  // namespace autoware::diffusion_planner::postprocess
