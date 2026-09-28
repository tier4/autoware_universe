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

#ifndef UTILS__VELOCITY_OPTIMIZER_HPP_
#define UTILS__VELOCITY_OPTIMIZER_HPP_

#include <optional>
#include <vector>

namespace autoware::safety_planner
{

struct VelocityOptimizerParams
{
  double a_min{-1.0};  //!< [m/s^2] < 0
  double a_max{1.0};   //!< [m/s^2] > 0
  double j_min{-1.0};  //!< [m/s^3] < 0
  double j_max{1.0};   //!< [m/s^3] > 0
  double jerk_weight{10.0};
  double over_v_weight{1.0e5};
  double over_a_weight{5.0e3};
  double over_j_weight{2.0e3};
};

//! v and a along the arc length, on the grid of v_max
struct VelocityOptimizerResult
{
  std::vector<double> v;  //!< [m/s]
  std::vector<double> a;  //!< [m/s^2]
};

//! The velocity profile along a path, as the JerkFilteredSmoother of autoware_velocity_smoother
//! formulates it: jerk limited forward / backward filters give the reference, then a QP in
//! b = v^2 and a minimizes the time and the pseudo jerk under soft bounds on v, a and j. v_max is
//! sampled every ds [m] from the ego (v0, a0); the first point with v_max == 0 is a stop point and
//! every point after it stays stopped. Without a0 the first acceleration is left to the QP, within
//! [a_min, a_max]. Returns nullopt when the QP does not solve
std::optional<VelocityOptimizerResult> optimize_velocity(
  const std::vector<double> & v_max, double ds, double v0, std::optional<double> a0,
  const VelocityOptimizerParams & params);

}  // namespace autoware::safety_planner

#endif  // UTILS__VELOCITY_OPTIMIZER_HPP_
