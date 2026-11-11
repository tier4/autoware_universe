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

// Velocity thresholds
constexpr float MOVING_VELOCITY_THRESHOLD_MPS = 0.2f;

// Time constants
constexpr double PREDICTION_TIME_STEP_S = 0.1;
constexpr int LOG_THROTTLE_INTERVAL_MS = 5000;

// Geometric constants
constexpr double LANE_MASK_RANGE_M = 100.0;
constexpr double MAX_ROUTE_SEGMENT_YAW_DIFF_RAD = M_PI / 3.0;
// A map element is on another level than the ego (an overpass or an underpass) and is not an
// input when its point closest to the ego, in the ego frame, is higher or lower than the ego by
// more than MAX_SAME_LEVEL_HEIGHT_DIFF_M plus SAME_LEVEL_HEIGHT_DIFF_PER_DISTANCE times its
// horizontal distance. Levels of a grade separation differ by 5.5 m or more (4.5 m clearance), and
// the distance term allows for a 5% change of grade on the ego's own road. On 2,699 frames of 16
// sites this leaves out 0.1-0.6% of the lanelets within 150 m, all 6.4 m or more above or below.
constexpr double MAX_SAME_LEVEL_HEIGHT_DIFF_M = 4.0;
constexpr double SAME_LEVEL_HEIGHT_DIFF_PER_DISTANCE = 0.05;
// Lane and route segments, intersection areas, stop lines and road borders are kept when a
// point lies within this box in the ego frame: LANE_MASK_BACKWARD_M behind to
// LANE_MASK_FORWARD_M ahead, and LANE_MASK_RANGE_M to either side.
constexpr double LANE_MASK_BACKWARD_M = 50.0;
constexpr double LANE_MASK_FORWARD_M = 150.0;

}  // namespace autoware::ml_planner::constants

#endif  // AUTOWARE__ML_PLANNER__CONSTANTS_HPP_
