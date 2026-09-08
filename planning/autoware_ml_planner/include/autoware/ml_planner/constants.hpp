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

namespace autoware::ml_planner::constants
{

// WEIGHT_MAJOR_VERSION should match the major version in ml_planner.param.json
constexpr int WEIGHT_MAJOR_VERSION = 5;

// Velocity thresholds
constexpr float MOVING_VELOCITY_THRESHOLD_MPS = 0.2f;

// Time constants
constexpr double PREDICTION_TIME_STEP_S = 0.1;
constexpr int LOG_THROTTLE_INTERVAL_MS = 5000;

// Geometric constants
constexpr double LANE_MASK_RANGE_M = 100.0;

}  // namespace autoware::ml_planner::constants

#endif  // AUTOWARE__ML_PLANNER__CONSTANTS_HPP_
