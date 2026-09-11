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

#ifndef TEMPORAL_MPT__BICYCLE_MODEL_PARAMS_HPP_
#define TEMPORAL_MPT__BICYCLE_MODEL_PARAMS_HPP_

#include <autoware/vehicle_info_utils/vehicle_info.hpp>

#include <algorithm>
#include <cmath>

namespace temporal_mpt
{

/** lf/lr for the temporal acados bicycle: L = lf + lr = wheel_base. */
struct BicycleLfLr
{
  double lf{1.0};
  double lr{1.0};
};

inline BicycleLfLr bicycleLfLrFromWheelBaseAndRearAxleToCg(
  const double wheel_base_m, const double rear_axle_to_cg_m)
{
  const double wheel_base = std::max(1.0e-3, wheel_base_m);
  double lr = rear_axle_to_cg_m;
  lr = std::clamp(lr, 1.0e-3, wheel_base - 1.0e-3);
  return {wheel_base - lr, lr};
}

/**
 * Rear-axle bicycle with yaw rate (v/L)*tan(delta), L = wheel_base.
 * lr is rear-axle to geometric box center (MPPI ego_axle_to_box_center); lf = wheel_base - lr.
 */
inline BicycleLfLr bicycleLfLrFromVehicleInfo(
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  return bicycleLfLrFromWheelBaseAndRearAxleToCg(
    vehicle_info.wheel_base_m,
    0.5 * vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m);
}

}  // namespace temporal_mpt

#endif  // TEMPORAL_MPT__BICYCLE_MODEL_PARAMS_HPP_
