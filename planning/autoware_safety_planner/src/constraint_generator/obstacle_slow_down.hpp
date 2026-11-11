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

#ifndef CONSTRAINT_GENERATOR__OBSTACLE_SLOW_DOWN_HPP_
#define CONSTRAINT_GENERATOR__OBSTACLE_SLOW_DOWN_HPP_

#include "constraint_generator_interface.hpp"

#include <string>

namespace autoware::safety_planner::experimental
{

//! The obstacle_slow_down of motion_velocity_planner (predicted objects only), emitting one
//! SpeedLimitZone per obstacle the ego passes close by
class ObstacleSlowDownConstraintGenerator : public ConstraintGeneratorInterface
{
public:
  std::string get_name() const override { return "obstacle_slow_down"; }

  ConstraintGeneratorOutput generate_constraints(const PlannerContext & context) override;
};

}  // namespace autoware::safety_planner::experimental

#endif  // CONSTRAINT_GENERATOR__OBSTACLE_SLOW_DOWN_HPP_
