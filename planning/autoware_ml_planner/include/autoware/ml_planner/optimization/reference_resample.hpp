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

#ifndef AUTOWARE__ML_PLANNER__OPTIMIZATION__REFERENCE_RESAMPLE_HPP_
#define AUTOWARE__ML_PLANNER__OPTIMIZATION__REFERENCE_RESAMPLE_HPP_

#include "autoware/ml_planner/optimization/acados_solver_wrapper.hpp"

#include <array>

namespace autoware::ml_planner::optimization
{

/// One pose on the polyline. Index 0 is the ego sample; indices 1..N are the horizon.
struct PoseSample
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/**
 * @brief A polyline reparameterized in time under a lateral-acceleration cap.
 *
 * ``poses[k]`` is where stage ``k`` should sit. ``speed_mps``, ``steering_rad`` and
 * ``acceleration_mps2`` are the feasible profile at that stage and are meant as the
 * initial guess of the following constrained solve, not as a tracked reference.
 * ``curvature_inv_m`` is the curvature of the input polyline, one value per input sample.
 */
struct ResampledSequence
{
  std::array<PoseSample, opt_horizon + 1> poses{};
  std::array<double, opt_horizon + 1> speed_mps{};
  std::array<double, opt_horizon + 1> steering_rad{};
  std::array<double, opt_horizon + 1> acceleration_mps2{};
  std::array<double, opt_horizon + 1> curvature_inv_m{};
};

/**
 * @brief Retime a pose sequence without leaving its polyline.
 *
 * Curvature is a smoothed ``d(yaw) / ds`` of ``poses``. The speed implied by the
 * spacing, and any steering stored elsewhere, are not inputs. The speed cap at a
 * station is ``sqrt(a_lat_max / |kappa|)``. The profile brakes or accelerates toward
 * ``min(cap, ego_speed)`` inside ``[min_acceleration, max_acceleration]`` and never
 * goes faster than ``ego_speed_mps``.
 *
 * @param poses ``opt_horizon + 1`` poses. Index 0 is the ego pose.
 * @param ego_speed_mps Measured speed at index 0 [m/s].
 * @param lateral_accel_limit_mps2 Lateral-acceleration cap used for the speed limit [m/s^2].
 * @param wheelbase_m Wheelbase used to turn curvature into a steering seed [m].
 * @param min_acceleration_mps2 Most negative acceleration of one step [m/s^2].
 * @param max_acceleration_mps2 Most positive acceleration of one step [m/s^2].
 */
ResampledSequence resample_pose_sequence(
  const std::array<PoseSample, opt_horizon + 1> & poses, double ego_speed_mps,
  double lateral_accel_limit_mps2, double wheelbase_m, double min_acceleration_mps2,
  double max_acceleration_mps2);

}  // namespace autoware::ml_planner::optimization

#endif  // AUTOWARE__ML_PLANNER__OPTIMIZATION__REFERENCE_RESAMPLE_HPP_
