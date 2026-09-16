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

#ifndef CONSTRAINT_GENERATOR__LANELET_SPEED_LIMIT_HPP_
#define CONSTRAINT_GENERATOR__LANELET_SPEED_LIMIT_HPP_

#include "constraint_generator_interface.hpp"

#include <string>

namespace autoware::safety_planner::experiment
{

//! Turns the speed limit of the map into one SpeedLimitZone per lanelet of the lane sequence on the
//! route: the region is the lanelet itself, the bound the limit the traffic rules of lanelet2
//! report for it (the speed_limit tag of the lanelet, or the default of the rule set).
//!
//! The zone of the lanelet the ego is on is never put below the current speed, the way
//! external_velocity_limit holds its global bound: a bound the ego already exceeds is violated by
//! the first point of every candidate, and the node stops emitting a trajectory altogether. What
//! brings the ego down to the limit are the zones of the lanelets ahead, which the planner reaches
//! by braking.
class LaneletSpeedLimitConstraintGenerator : public ConstraintGeneratorInterface
{
public:
  std::string get_name() const override { return "lanelet_speed_limit"; }
  ConstraintGeneratorOutput generate_constraints(const PlannerContext & context) override;
};

}  // namespace autoware::safety_planner::experiment

#endif  // CONSTRAINT_GENERATOR__LANELET_SPEED_LIMIT_HPP_
