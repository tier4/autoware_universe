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

#pragma once

#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params.hpp"

#include <fstream>
#include <iomanip>
#include <string>

namespace autoware::mppi_optimizer::open_loop_sim
{

/** MPPI oriented ego footprint (rear-axle state + offset to box center). */
inline bool writeEgoFootprintCsv(
  const std::string & path, const FirstOrderDubinsMppiVehicleParams & vehicle)
{
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << "length,width,axle_to_box_center\n";
  out << std::setprecision(9) << std::fixed << vehicle.ego_length << "," << vehicle.ego_width
      << "," << vehicle.ego_axle_to_box_center << "\n";
  return true;
}

}  // namespace autoware::mppi_optimizer::open_loop_sim
