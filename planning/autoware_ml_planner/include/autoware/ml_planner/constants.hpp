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

#ifndef AUTOWARE__ML_PLANNER__CONSTANTS_HPP_
#define AUTOWARE__ML_PLANNER__CONSTANTS_HPP_

#include <cmath>

namespace autoware::ml_planner::constants
{

// Major version of the ONNX weights this node is built against.
constexpr int WEIGHT_MAJOR_VERSION = 5;

// Velocity thresholds
constexpr float MOVING_VELOCITY_THRESHOLD_MPS = 0.2f;

// Time constants
constexpr double PREDICTION_TIME_STEP_S = 0.1;
constexpr int LOG_THROTTLE_INTERVAL_MS = 5000;

// Geometric constants
constexpr double LANE_MASK_RANGE_M = 100.0;
constexpr double MAX_ROUTE_SEGMENT_YAW_DIFF_RAD = M_PI / 3.0;
// Lanelets whose centerline is longer than this are split into consecutive lane segments of
// equal length, so a long lanelet keeps a fine point spacing in the fixed-size lane tensors.
constexpr double LANE_SEGMENT_MAX_LENGTH_M = 40.0;
// Maximum distance between consecutive road border points; longer gaps are filled by linear
// interpolation. Original points are always kept.
constexpr double ROAD_BORDER_MAX_STEP_M = 5.0;

}  // namespace autoware::ml_planner::constants

#endif  // AUTOWARE__ML_PLANNER__CONSTANTS_HPP_
