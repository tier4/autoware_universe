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

#ifndef UTILS__TRAJECTORY_CONVERSION_HPP_
#define UTILS__TRAJECTORY_CONVERSION_HPP_

// The internal trajectory point shared by the trajectory_planner plugins, its conversion to the
// output message, plus the post-processing of the output trajectory.

#include "../constraint.hpp"
#include "../type_alias.hpp"

namespace autoware::safety_planner
{

//! One point of a planned trajectory: the state (px, py, theta, k, v, a) and the input (w, j).
//! The input is the one held from this point to the next, and is 0 at the last point (k = N).
struct OptimizedTrajectoryPoint
{
  double t{0.0};      //!< [s] relative to the planning reference time
  Pose2d pose{};      //!< world coordinates (px, py, theta)
  double kappa{0.0};  //!< [1/m] path curvature
  double v{0.0};      //!< [m/s]
  double a{0.0};      //!< [m/s^2]
  double w{0.0};      //!< [1/(m·s)] dκ/dt
  double j{0.0};      //!< [m/s³]  da/dt
};

//! Converts one point. It is 2D, so z is taken from the reference pose (ego).
TrajectoryPoint to_trajectory_point(
  const OptimizedTrajectoryPoint & optimized_point, const double z, const double wheel_base_m);

//! Puts a lower bound (the engage velocity) on the velocity of the launch section. In the cycle
//! that starts from standstill the leading points have v near 0, which the longitudinal controller
//! reads as a stop point at distance 0 and never leaves STOPPED. Only the points up to the first
//! one already at or above the bound are rewritten, so the deceleration towards the goal is left
//! alone, and a cycle that barely advances (just before the goal, a stop plan) is not touched.
Trajectory set_engage_speed(const Trajectory & trajectory, const double engage_velocity_mps);

}  // namespace autoware::safety_planner

#endif  // UTILS__TRAJECTORY_CONVERSION_HPP_
