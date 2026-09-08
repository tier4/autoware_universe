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

#ifndef AUTOWARE__SAFETY_PLANNER__UTILS__TRAJECTORY_CONVERSION_HPP_
#define AUTOWARE__SAFETY_PLANNER__UTILS__TRAJECTORY_CONVERSION_HPP_

// Conversion from the internal representations (RoughPlanPoint / OptimizedTrajectoryPoint) to the
// output message, plus the post-processing of the output trajectory. Both the node (debug
// publishing) and the trajectory_planner plugins call it, so it lives here.

#include "../trajectory_planner/nlp_planner/rough_planner.hpp"
#include "../trajectory_planner/nlp_planner/trajectory_optimizer_interface.hpp"
#include "../type_alias.hpp"

namespace autoware::safety_planner
{

//! Converts one point of a RoughPlan. The plan is 2D (Pose2d + kappa), so z is taken from the
//! reference pose (ego).
TrajectoryPoint to_trajectory_point(
  const RoughPlanPoint & rough_point, const double z, const double wheel_base_m);

//! Converts one point of an optimized trajectory. It is 2D, so z is taken from the reference pose
//! (ego).
TrajectoryPoint to_trajectory_point(
  const OptimizedTrajectoryPoint & optimized_point, const double z, const double wheel_base_m);

//! Puts a lower bound (the engage velocity) on the velocity of the launch section. In the cycle
//! that starts from standstill the leading points have v near 0, which the longitudinal controller
//! reads as a stop point at distance 0 and never leaves STOPPED. Only the points up to the first
//! one already at or above the bound are rewritten, so the deceleration towards the goal is left
//! alone, and a cycle that barely advances (just before the goal, a stop plan) is not touched.
Trajectory set_engage_speed(const Trajectory & trajectory, const double engage_velocity_mps);

}  // namespace autoware::safety_planner

#endif  // AUTOWARE__SAFETY_PLANNER__UTILS__TRAJECTORY_CONVERSION_HPP_
