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

#ifndef TRAJECTORY_PLANNER__PLANNING_FACTORS_HPP_
#define TRAJECTORY_PLANNER__PLANNING_FACTORS_HPP_

#include "../context.hpp"
#include "frenet_sampling_based_planner/constraints_compiler.hpp"
#include "trajectory_planner_interface.hpp"

namespace autoware::safety_planner
{

//! Records the stop bar, or the path end when there is none.
void add_planning_factors(
  PlanningFactorInterface * planning_factor_interface, const PlannerContext & context,
  const CompiledConstraints & compiled_constraints, double horizon_s);

//! The cautious side publishes the same factors when it reuses the normal trajectory
void copy_planning_factors(const PlanningFactorInterface * from, PlanningFactorInterface * to);

}  // namespace autoware::safety_planner

#endif  // TRAJECTORY_PLANNER__PLANNING_FACTORS_HPP_
