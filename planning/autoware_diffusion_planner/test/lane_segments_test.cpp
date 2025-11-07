// Copyright 2025 TIER IV, Inc.
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

#include "lane_segments_test.hpp"

#include "autoware/diffusion_planner/conversion/lanelet.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/preprocessing/lane_segments.hpp"

#include <Eigen/Dense>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner::test
{

TEST_F(LaneSegmentsTest, LaneSegmentContextFunctionality)
{
}

}  // namespace autoware::diffusion_planner::test
