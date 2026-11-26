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

#ifndef AUTOWARE__ML_PLANNER__OPTIMIZATION__LATERAL_ACCELERATION_GUARD_HPP_
#define AUTOWARE__ML_PLANNER__OPTIMIZATION__LATERAL_ACCELERATION_GUARD_HPP_

#include "autoware/ml_planner/optimization/acados_solver_wrapper.hpp"

#include <array>

namespace autoware::ml_planner::optimization
{

/// One sample of the geometry-pass solution. Index 0 is the ego sample; indices 1..N are
/// the horizon. ``steering_rad`` is the solved steering, the source of curvature.
/// ``speed_mps`` is the speed the model wanted at this station.
struct PoseSample
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double steering_rad{0.0};
  double speed_mps{0.0};
};

/**
 * @brief A polyline placed in time under a lateral-acceleration cap.
 *
 * ``poses[k]`` is where stage ``k`` should sit. The following constrained solve tracks those
 * poses. ``curvature_inv_m`` is ``tan(steering) / wheelbase`` of the geometry-pass solution.
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
 * @brief Place new target times on the geometry-pass polyline under ``a_lat``.
 *
 * Curvature is ``tan(steering_rad) / wheelbase``. The speed cap at a station is
 * ``sqrt(a_lat_max / |kappa|)``. The profile starts at ``ego_speed_mps`` and brakes or
 * accelerates toward ``min(cap, speed_mps)`` inside the longitudinal acceleration bounds.
 * Stations slower than ``min_filter_speed_mps`` are not capped.
 *
 * @param poses ``opt_horizon + 1`` samples of the geometry-pass solution. Index 0 is ego.
 *        ``speed_mps`` on each sample is the desired speed at that station [m/s].
 * @param ego_speed_mps Measured speed at index 0 [m/s].
 * @param lateral_accel_limit_mps2 Lateral-acceleration cap used for the speed limit [m/s^2].
 * @param wheelbase_m Wheelbase used to turn steering into curvature [m].
 * @param min_acceleration_mps2 Most negative acceleration of one step [m/s^2].
 * @param max_acceleration_mps2 Most positive acceleration of one step [m/s^2].
 * @param min_filter_speed_mps Do not apply the curvature cap below this speed [m/s].
 */
ResampledSequence resample_pose_sequence(
  const std::array<PoseSample, opt_horizon + 1> & poses, double ego_speed_mps,
  double lateral_accel_limit_mps2, double wheelbase_m, double min_acceleration_mps2,
  double max_acceleration_mps2, double min_filter_speed_mps = 0.0);

}  // namespace autoware::ml_planner::optimization

#endif  // AUTOWARE__ML_PLANNER__OPTIMIZATION__LATERAL_ACCELERATION_GUARD_HPP_
