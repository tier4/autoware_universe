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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__SET_MAX_VELOCITY_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__SET_MAX_VELOCITY_HPP_

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <vector>

namespace autoware::trajectory_modifier::utils
{

using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

/// @brief Clamp points above max_velocity and recompute acceleration on the clamp boundaries.
///
/// Interior points of a clamped run get zero acceleration. The point before the run and the last
/// point of the run get `(v_next - v) / dt`, where `dt` is taken from the pre-clamp velocity change
/// and acceleration. The final point acceleration is zero. This is the in-place cap used when jerk
/// smoothing is off; it does not build a braking ramp.
void set_max_velocity(TrajectoryPoints & input_trajectory_array, float max_velocity);

}  // namespace autoware::trajectory_modifier::utils

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__SET_MAX_VELOCITY_HPP_
