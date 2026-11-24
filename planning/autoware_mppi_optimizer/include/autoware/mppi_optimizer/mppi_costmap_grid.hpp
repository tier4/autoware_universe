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

#ifndef AUTOWARE__MPPI_OPTIMIZER__MPPI_COSTMAP_GRID_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__MPPI_COSTMAP_GRID_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace autoware::mppi_optimizer
{

/**
 * Heat value in [0, 100] for one cell, or -1 (unknown) for non-finite clearance.
 *
 * The rollout obstacle barrier is quadratic in how far a cell intrudes into the safety margin, so
 * its outer halo is nearly invisible when mapped linearly. sqrt(cost / contact_cost) is the
 * (linear) fraction of the margin that is intruded: 0 at the edge of influence, 1 at exact
 * contact, and saturated at 100 inside an obstacle. The raw clearance stays in the snapshot.
 */
[[nodiscard]] inline std::int8_t obstacleCostmapHeat(
  const FirstOrderDubinsMppiCostmapSnapshot & snapshot, const float clearance)
{
  if (!std::isfinite(clearance)) {
    return -1;
  }
  const float reference_cost = snapshot.referenceCost();
  const float cost = snapshot.barrierCost(clearance);
  if (!(reference_cost > 0.0F) || !(cost > 0.0F)) {
    return 0;
  }
  const float fraction = std::min(1.0F, std::sqrt(cost / reference_cost));
  return static_cast<std::int8_t>(std::lround(100.0F * fraction));
}

/**
 * Convert the obstacle distance slice into an OccupancyGrid for RViz's Map display (use the
 * "costmap" color scheme for a heatmap). The grid is axis-aligned in the odometry frame (identity
 * orientation) and covers the obstacle texture, which is centered on the reference path rather
 * than on base_link, so it is positioned with the origin carried by the snapshot.
 *
 * @param z Height [m] of the grid plane in the odometry frame.
 */
[[nodiscard]] inline nav_msgs::msg::OccupancyGrid makeObstacleCostmapGrid(
  const FirstOrderDubinsMppiCostmapSnapshot & snapshot, const double z)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = snapshot.frame_id;
  grid.header.stamp = snapshot.stamp;
  grid.info.map_load_time = snapshot.stamp;
  grid.info.resolution = snapshot.resolution_m;
  grid.info.width = static_cast<std::uint32_t>(snapshot.width);
  grid.info.height = static_cast<std::uint32_t>(snapshot.height);
  grid.info.origin.position.x = snapshot.origin_x;
  grid.info.origin.position.y = snapshot.origin_y;
  grid.info.origin.position.z = z;
  grid.info.origin.orientation.w = 1.0;
  const std::size_t cell_count =
    static_cast<std::size_t>(snapshot.width) * static_cast<std::size_t>(snapshot.height);
  grid.data.assign(cell_count, static_cast<std::int8_t>(-1));
  const std::size_t available = std::min(cell_count, snapshot.clearance_m.size());
  for (std::size_t index = 0; index < available; ++index) {
    grid.data[index] = obstacleCostmapHeat(snapshot, snapshot.clearance_m[index]);
  }
  return grid;
}

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__MPPI_COSTMAP_GRID_HPP_
