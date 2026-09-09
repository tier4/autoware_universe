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

#ifndef AUTOWARE__ML_PLANNER__DIMENSIONS_HPP_
#define AUTOWARE__ML_PLANNER__DIMENSIONS_HPP_

#include "autoware/ml_planner/preprocessing/items/map.hpp"

#include <array>
#include <cstdint>

namespace autoware::ml_planner
{
inline constexpr int64_t NUM_SEGMENTS_IN_LANE = 140;
inline constexpr int64_t NUM_SEGMENTS_IN_ROUTE = 25;
inline constexpr int64_t NUM_INTERSECTION_AREAS = 10;
inline constexpr int64_t NUM_STOP_LINES = 30;
inline constexpr int64_t NUM_ROAD_BORDERS = 30;
inline constexpr int64_t MAX_NUM_NEIGHBORS = 320;
inline constexpr int64_t MAX_NUM_AGENTS = MAX_NUM_NEIGHBORS + 1;  // Including ego
inline constexpr int64_t HIDDEN_DIM = 256;
inline constexpr int64_t ENCODING_TOKEN_NUM =
  MAX_NUM_NEIGHBORS + NUM_SEGMENTS_IN_LANE + NUM_SEGMENTS_IN_ROUTE + NUM_INTERSECTION_AREAS +
  NUM_STOP_LINES + NUM_ROAD_BORDERS + 3;  // goal_pose, ego_shape, ego_history
inline constexpr int64_t POINTS_PER_SEGMENT = 20;
inline constexpr int64_t POINTS_PER_INTERSECTION_AREA = 40;
inline constexpr int64_t POINTS_PER_STOP_LINE = 2;
inline constexpr int64_t POINTS_PER_ROAD_BORDER = 20;
// Traffic light state encoding:
// [GREEN, AMBER, RED, UNKNOWN/no-data, WHITE or no light, is-arrow flag]
// The first five slots are a one-hot color encoding; the last slot is set
// when the decisive element is an arrow (e.g. green arrow vs. green circle).
inline constexpr int64_t TRAFFIC_LIGHT_ONE_HOT_DIM = 6;

inline constexpr int64_t EGO_AGENT_PAST_IDX_X = 0;
inline constexpr int64_t EGO_AGENT_PAST_IDX_Y = 1;
inline constexpr int64_t EGO_AGENT_PAST_IDX_COS = 2;
inline constexpr int64_t EGO_AGENT_PAST_IDX_SIN = 3;
inline constexpr int64_t EGO_AGENT_PAST_IDX_VELOCITY = 4;
inline constexpr int64_t EGO_AGENT_PAST_IDX_YAW_RATE = 5;
inline constexpr int64_t EGO_HISTORY_DIM = 6;

// Index for each field
inline constexpr int64_t X = 0;
inline constexpr int64_t Y = 1;
inline constexpr int64_t LB_X = 2;
inline constexpr int64_t LB_Y = 3;
inline constexpr int64_t RB_X = 4;
inline constexpr int64_t RB_Y = 5;
inline constexpr int64_t SEGMENT_POINT_DIM = 6;
inline constexpr int64_t LINE_TYPE_LEFT_START = 0;
inline constexpr int64_t LINE_TYPE_RIGHT_START = LINE_TYPE_NUM;
inline constexpr int64_t LANE_TYPE_DIM = 2 * LINE_TYPE_NUM;

inline constexpr int64_t INPUT_T = 30;
inline constexpr int64_t INPUT_T_WITH_CURRENT = INPUT_T + 1;  // Including current time step
// Time window of raw message history buffers: the 31-step 0.1 s input grid
// span plus a margin for the bracketing message and stamp jitter.
inline constexpr double HISTORY_WINDOW_S = 0.1 * static_cast<double>(INPUT_T_WITH_CURRENT) + 0.5;
inline constexpr int64_t OUTPUT_T = 80;  // Output timestamp number
inline constexpr int64_t POSE_DIM = 4;   // x, y, cos(yaw), sin(yaw)
inline constexpr std::array<int64_t, 4> OUTPUT_SHAPE = {1, MAX_NUM_AGENTS, OUTPUT_T, POSE_DIM};

inline constexpr int64_t TURN_INDICATOR_OUTPUT_DISABLE = 0;
inline constexpr int64_t TURN_INDICATOR_OUTPUT_ENABLE_LEFT = 1;
inline constexpr int64_t TURN_INDICATOR_OUTPUT_ENABLE_RIGHT = 2;
inline constexpr int64_t TURN_INDICATOR_OUTPUT_DIM = 3;
inline constexpr std::array<int64_t, 2> TURN_INDICATOR_LOGIT_SHAPE = {1, TURN_INDICATOR_OUTPUT_DIM};

inline constexpr std::array<int64_t, 4> INITIAL_NOISE_SHAPE = {
  1, MAX_NUM_AGENTS, OUTPUT_T, POSE_DIM};
inline constexpr std::array<int64_t, 3> EGO_HISTORY_SHAPE = {1, INPUT_T + 1, EGO_HISTORY_DIM};
inline constexpr std::array<int64_t, 4> NEIGHBOR_SHAPE = {
  1, MAX_NUM_NEIGHBORS, INPUT_T + 1, POSE_DIM};
inline constexpr std::array<int64_t, 3> AGENT_SHAPE_SHAPE = {1, MAX_NUM_NEIGHBORS, 2};
inline constexpr std::array<int64_t, 3> AGENT_LABEL_SHAPE = {1, MAX_NUM_NEIGHBORS, 3};
inline constexpr std::array<int64_t, 4> LANES_SHAPE = {
  1, NUM_SEGMENTS_IN_LANE, POINTS_PER_SEGMENT, SEGMENT_POINT_DIM};
inline constexpr std::array<int64_t, 3> LANE_TYPES_SHAPE = {1, NUM_SEGMENTS_IN_LANE, LANE_TYPE_DIM};
inline constexpr std::array<int64_t, 3> LANES_HAS_SPEED_LIMIT_SHAPE = {1, NUM_SEGMENTS_IN_LANE, 1};
inline constexpr std::array<int64_t, 3> LANES_SPEED_LIMIT_SHAPE = {1, NUM_SEGMENTS_IN_LANE, 1};
inline constexpr std::array<int64_t, 4> ROUTE_LANES_SHAPE = {
  1, NUM_SEGMENTS_IN_ROUTE, POINTS_PER_SEGMENT, SEGMENT_POINT_DIM};
inline constexpr std::array<int64_t, 3> ROUTE_LANE_TYPES_SHAPE = {
  1, NUM_SEGMENTS_IN_ROUTE, LANE_TYPE_DIM};
inline constexpr std::array<int64_t, 3> ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE = {
  1, NUM_SEGMENTS_IN_ROUTE, 1};
inline constexpr std::array<int64_t, 3> ROUTE_LANES_SPEED_LIMIT_SHAPE = {
  1, NUM_SEGMENTS_IN_ROUTE, 1};
inline constexpr std::array<int64_t, 4> INTERSECTION_AREA_SHAPE = {
  1, NUM_INTERSECTION_AREAS, POINTS_PER_INTERSECTION_AREA, 2};
inline constexpr std::array<int64_t, 4> STOP_LINES_SHAPE = {
  1, NUM_STOP_LINES, POINTS_PER_STOP_LINE, 2};
inline constexpr std::array<int64_t, 4> ROAD_BORDERS_SHAPE = {
  1, NUM_ROAD_BORDERS, POINTS_PER_ROAD_BORDER, 2};
inline constexpr std::array<int64_t, 2> GOAL_POSE_SHAPE = {1, POSE_DIM};
inline constexpr std::array<int64_t, 2> EGO_SHAPE_SHAPE = {1, 3};
inline constexpr std::array<int64_t, 2> TURN_INDICATORS_SHAPE = {1, INPUT_T + 1};
// Per-segment history of the associated traffic light state
inline constexpr std::array<int64_t, 4> LANE_TRAFFIC_LIGHT_PAST_SHAPE = {
  1, NUM_SEGMENTS_IN_LANE, INPUT_T + 1, TRAFFIC_LIGHT_ONE_HOT_DIM};
inline constexpr std::array<int64_t, 4> ROUTE_TRAFFIC_LIGHT_PAST_SHAPE = {
  1, NUM_SEGMENTS_IN_ROUTE, INPUT_T + 1, TRAFFIC_LIGHT_ONE_HOT_DIM};
inline constexpr std::array<int64_t, 4> LANE_TRAFFIC_LIGHT_FUTURE_SHAPE = {
  1, NUM_SEGMENTS_IN_LANE, OUTPUT_T, TRAFFIC_LIGHT_ONE_HOT_DIM};
inline constexpr std::array<int64_t, 4> ROUTE_TRAFFIC_LIGHT_FUTURE_SHAPE = {
  1, NUM_SEGMENTS_IN_ROUTE, OUTPUT_T, TRAFFIC_LIGHT_ONE_HOT_DIM};

inline constexpr float POSITION_SCALE = 50.0F;
inline constexpr float SPEED_SCALE = 15.0F;
inline constexpr float VEHICLE_SHAPE_SCALE = 10.0F;
}  // namespace autoware::ml_planner
#endif  // AUTOWARE__ML_PLANNER__DIMENSIONS_HPP_
