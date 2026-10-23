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

#ifndef AUTOWARE__MPPI_OPTIMIZER__INPUT_COLLISION_GATE_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__INPUT_COLLISION_GATE_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params.hpp"

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <vector>

namespace autoware::mppi_optimizer
{

struct InputCollisionConditions
{
  bool road_border{false};
  bool static_object{false};
};

struct InputCollisionResult
{
  bool road_border{false};
  bool static_object{false};

  [[nodiscard]] bool triggered() const { return road_border || static_object; }
};

/**
 * Check the input prefix that MPPI can replace, using the optimizer's collision geometry
 * inflated by the supplied activation margins.
 * The object argument contains stationary avoidance targets selected by the modifier plugin.
 */
[[nodiscard]] InputCollisionResult checkInputTrajectoryCollisions(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const autoware_perception_msgs::msg::TrackedObjects & static_objects,
  const std::vector<Segment> & road_borders, const FirstOrderDubinsMppiVehicleParams & vehicle,
  float obstacle_margin, float road_border_margin, InputCollisionConditions conditions);

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__INPUT_COLLISION_GATE_HPP_
