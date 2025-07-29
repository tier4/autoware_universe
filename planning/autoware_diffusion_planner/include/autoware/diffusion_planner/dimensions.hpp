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

#ifndef AUTOWARE__DIFFUSION_PLANNER__DIMENSIONS_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__DIMENSIONS_HPP_

#include <array>
#include <cstdint>
#include <vector>

namespace autoware::diffusion_planner
{
inline constexpr int64_t SEGMENT_POINT_DIM = 12;   // Dimension of a lane segment point
inline constexpr int64_t POINTS_PER_SEGMENT = 20;  //!< Number of points in each lane segment.
// Number of columns in a segment matrix
// (X,Y,dX,dY,LeftBoundX,LeftBoundY,RightBoundX,RightBoundX,TrafficLightEncoding(Dim4),Speed Limit)
inline constexpr int64_t FULL_MATRIX_ROWS = 14;
inline constexpr int64_t TRAFFIC_LIGHT_ONE_HOT_DIM = 4;

// Index for each field
inline constexpr int64_t X = 0;
inline constexpr int64_t Y = 1;
inline constexpr int64_t dX = 2;
inline constexpr int64_t dY = 3;
inline constexpr int64_t LB_X = 4;
inline constexpr int64_t LB_Y = 5;
inline constexpr int64_t RB_X = 6;
inline constexpr int64_t RB_Y = 7;
inline constexpr int64_t TRAFFIC_LIGHT = 8;
inline constexpr int64_t TRAFFIC_LIGHT_GREEN = 8;
inline constexpr int64_t TRAFFIC_LIGHT_YELLOW = 9;
inline constexpr int64_t TRAFFIC_LIGHT_RED = 10;
inline constexpr int64_t TRAFFIC_LIGHT_WHITE = 11;
inline constexpr int64_t SPEED_LIMIT = 12;
inline constexpr int64_t LANE_ID = 13;

inline constexpr int64_t OUTPUT_T = 80;  // Output timestamp number
inline constexpr std::array<int64_t, 4> OUTPUT_SHAPE = {1, 33, 80, 4};

inline constexpr std::array<int64_t, 2> EGO_CURRENT_STATE_SHAPE = {1, 10};
inline constexpr std::array<int64_t, 4> NEIGHBOR_SHAPE = {1, 32, 21, 11};
inline constexpr std::array<int64_t, 3> LANE_HAS_SPEED_LIMIT_SHAPE = {1, 70, 1};
inline constexpr std::array<int64_t, 3> STATIC_OBJECTS_SHAPE = {1, 5, 10};
inline constexpr std::array<int64_t, 4> LANES_SHAPE = {1, 70, 20, 12};
inline constexpr std::array<int64_t, 3> LANES_SPEED_LIMIT_SHAPE = {1, 70, 1};
inline constexpr std::array<int64_t, 4> ROUTE_LANES_SHAPE = {1, 25, 20, 12};
}  // namespace autoware::diffusion_planner
#endif  // AUTOWARE__DIFFUSION_PLANNER__DIMENSIONS_HPP_
