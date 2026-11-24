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

#include "autoware/mppi_optimizer/mppi_costmap_grid.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace autoware::mppi_optimizer
{
namespace
{

FirstOrderDubinsMppiCostmapSnapshot makeSnapshot()
{
  FirstOrderDubinsMppiCostmapSnapshot snapshot;
  snapshot.valid = true;
  snapshot.frame_id = "map";
  snapshot.width = 4;
  snapshot.height = 2;
  snapshot.resolution_m = 0.5F;
  snapshot.origin_x = 100.0;
  snapshot.origin_y = -20.0;
  snapshot.footprint_circle_radius_m = 1.0F;
  snapshot.safety_margin_m = 2.0F;
  snapshot.barrier_weight = 25.0F;  // contact cost = 25 * 2^2 = 100
  snapshot.clearance_m.assign(8U, 50.0F);
  return snapshot;
}

TEST(MppiCostmapGrid, BarrierCostMatchesTheRolloutBarrier)
{
  const auto snapshot = makeSnapshot();
  EXPECT_FLOAT_EQ(snapshot.referenceCost(), 100.0F);
  // Footprint circle exactly at the edge of influence.
  EXPECT_FLOAT_EQ(snapshot.barrierCost(3.0F), 0.0F);
  EXPECT_FLOAT_EQ(snapshot.barrierCost(100.0F), 0.0F);
  // Half of the margin intruded.
  EXPECT_FLOAT_EQ(snapshot.barrierCost(2.0F), 25.0F);
  // Circle touching the obstacle.
  EXPECT_FLOAT_EQ(snapshot.barrierCost(1.0F), 100.0F);
}

TEST(MppiCostmapGrid, GridIsAxisAlignedAtTheSnapshotOrigin)
{
  const auto message = makeObstacleCostmapGrid(makeSnapshot(), 1.5);
  EXPECT_DOUBLE_EQ(message.info.origin.position.x, 100.0);
  EXPECT_DOUBLE_EQ(message.info.origin.position.y, -20.0);
  EXPECT_DOUBLE_EQ(message.info.origin.position.z, 1.5);
  EXPECT_DOUBLE_EQ(message.info.origin.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(message.info.origin.orientation.w, 1.0);
  EXPECT_EQ(message.info.width, 4U);
  EXPECT_EQ(message.info.height, 2U);
  EXPECT_FLOAT_EQ(message.info.resolution, 0.5F);
  EXPECT_EQ(message.header.frame_id, "map");
}

TEST(MppiCostmapGrid, HeatSaturatesAtContactAndMarksNonFiniteCellsUnknown)
{
  auto snapshot = makeSnapshot();
  snapshot.clearance_m[0] = 50.0F;  // far away
  snapshot.clearance_m[1] = 2.0F;   // sqrt(25 / 100): half of the margin intruded
  snapshot.clearance_m[2] = 1.0F;   // circle touching the obstacle
  snapshot.clearance_m[3] = -4.0F;  // deep inside an obstacle
  snapshot.clearance_m[4] = std::numeric_limits<float>::quiet_NaN();
  const auto message = makeObstacleCostmapGrid(snapshot, 0.0);
  ASSERT_EQ(message.data.size(), 8U);
  EXPECT_EQ(message.data[0], 0);
  EXPECT_EQ(message.data[1], 50);
  EXPECT_EQ(message.data[2], 100);
  EXPECT_EQ(message.data[3], 100);
  EXPECT_EQ(message.data[4], -1);
}

TEST(MppiCostmapGrid, DisabledBarrierProducesAFlatMap)
{
  auto snapshot = makeSnapshot();
  snapshot.barrier_weight = 0.0F;
  snapshot.clearance_m.assign(8U, -1.0F);
  const auto message = makeObstacleCostmapGrid(snapshot, 0.0);
  for (const auto value : message.data) {
    EXPECT_EQ(value, 0);
  }
}

}  // namespace
}  // namespace autoware::mppi_optimizer
