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

/// One sample of the smoothed solution. Index 0 is the ego sample; indices 1..N are the horizon.
/// ``steering_rad`` is the solved steering angle, the source of the curvature.
struct PoseSample
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double steering_rad{0.0};
};

/**
 * @brief A polyline reparameterized in time under a lateral-acceleration cap.
 *
 * ``poses[k]`` is where stage ``k`` should sit. ``speed_mps``, ``steering_rad`` and
 * ``acceleration_mps2`` are the feasible profile at that stage and are meant as the
 * initial guess of the following constrained solve, not as a tracked reference.
 * ``curvature_inv_m`` is ``tan(steering) / wheelbase`` of the smoothed solution, one value per
 * sample.
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
 * @brief Retime the smoothed solution without leaving its polyline.
 *
 * Curvature at each sample is the bicycle curvature of the solved steering angle,
 * ``tan(steering_rad) / wheelbase``. Heading differences are not used: the smoothing
 * solve already produced a steering profile, and the raw model output's steering is
 * not what this function sees. The speed cap at a station is
 * ``sqrt(a_lat_max / |kappa|)``. The profile brakes or accelerates toward
 * ``min(cap, ego_speed)`` inside ``[min_acceleration, max_acceleration]`` and never
 * goes faster than ``ego_speed_mps``.
 *
 * @param poses ``opt_horizon + 1`` samples of the smoothed solution. Index 0 is ego.
 * @param ego_speed_mps Measured speed at index 0 [m/s].
 * @param lateral_accel_limit_mps2 Lateral-acceleration cap used for the speed limit [m/s^2].
 * @param wheelbase_m Wheelbase used to turn steering into curvature [m].
 * @param min_acceleration_mps2 Most negative acceleration of one step [m/s^2].
 * @param max_acceleration_mps2 Most positive acceleration of one step [m/s^2].
 */
ResampledSequence resample_pose_sequence(
  const std::array<PoseSample, opt_horizon + 1> & poses, double ego_speed_mps,
  double lateral_accel_limit_mps2, double wheelbase_m, double min_acceleration_mps2,
  double max_acceleration_mps2);

}  // namespace autoware::ml_planner::optimization

#endif  // AUTOWARE__ML_PLANNER__OPTIMIZATION__REFERENCE_RESAMPLE_HPP_
