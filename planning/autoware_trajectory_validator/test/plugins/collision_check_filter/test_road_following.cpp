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

#include "../../../src/filters/safety/collision_check_filter/parameter.hpp"

#include <gtest/gtest.h>

namespace autoware::trajectory_validator::plugin::safety
{
namespace
{
TEST(RoadFollowingParams, CopiesEveryNodeParameter)
{
  // The values differ from the defaults, so a constructor that ignores its input fails.
  validator::Params::CollisionCheck::RoadFollowing node_params;
  node_params.ego_footprint_margin.lateral = -0.2;
  node_params.ego_footprint_margin.front = 0.3;
  node_params.ego_footprint_margin.rear = 0.4;
  node_params.max_lateral_deviation = 0.6;
  node_params.max_heading_deviation = 0.15;
  node_params.window_distance = 40.0;
  node_params.window_time = 4.0;

  const RoadFollowingParams params(node_params);

  EXPECT_DOUBLE_EQ(params.ego_footprint_margin.lateral, -0.2);
  EXPECT_DOUBLE_EQ(params.ego_footprint_margin.front, 0.3);
  EXPECT_DOUBLE_EQ(params.ego_footprint_margin.rear, 0.4);
  EXPECT_DOUBLE_EQ(params.max_lateral_deviation, 0.6);
  EXPECT_DOUBLE_EQ(params.max_heading_deviation, 0.15);
  EXPECT_DOUBLE_EQ(params.window_distance, 40.0);
  EXPECT_DOUBLE_EQ(params.window_time, 4.0);
}
}  // namespace
}  // namespace autoware::trajectory_validator::plugin::safety
