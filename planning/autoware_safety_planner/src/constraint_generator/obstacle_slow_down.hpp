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

#include <autoware/trajectory_modifier/trajectory_modifier_utils/obstacle_slow_down_utils.hpp>

#include <optional>
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

private:
  using SlowDownParameters = trajectory_modifier::utils::obstacle_slow_down::SlowDownParameters;
  using SlowDownPlanner = trajectory_modifier::utils::obstacle_slow_down::SlowDownPlanner;

  //! Converted from params_ and validated on the first call, since params_ is only set after
  //! construction
  const SlowDownParameters & slow_down_parameters();

  std::optional<SlowDownParameters> slow_down_parameters_;
  //! Keeps the per-UUID entry/exit counters and the previous outputs across cycles
  SlowDownPlanner slow_down_planner_;
};

}  // namespace autoware::safety_planner::experimental

#endif  // CONSTRAINT_GENERATOR__OBSTACLE_SLOW_DOWN_HPP_
