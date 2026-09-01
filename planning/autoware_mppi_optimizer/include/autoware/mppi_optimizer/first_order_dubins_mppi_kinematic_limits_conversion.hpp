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

#ifndef AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_KINEMATIC_LIMITS_CONVERSION_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_KINEMATIC_LIMITS_CONVERSION_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>

namespace autoware::mppi_optimizer
{

/** @brief Converts an Autoware velocity limit into optional MPPI kinematic bounds. */
inline FirstOrderDubinsMppiKinematicLimits makeKinematicLimits(
  const autoware_internal_planning_msgs::msg::VelocityLimit & velocity_limit)
{
  FirstOrderDubinsMppiKinematicLimits limits;
  limits.max_velocity = velocity_limit.max_velocity;

  if (velocity_limit.use_constraints) {
    limits.min_longitudinal_acceleration = velocity_limit.constraints.min_acceleration;
    limits.max_longitudinal_acceleration = velocity_limit.constraints.max_acceleration;
    limits.min_longitudinal_jerk = velocity_limit.constraints.min_jerk;
    limits.max_longitudinal_jerk = velocity_limit.constraints.max_jerk;
  }

  return limits;
}

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_KINEMATIC_LIMITS_CONVERSION_HPP_
