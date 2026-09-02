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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TIME_SEQUENCE_RAW__OPTIMIZER_PARAMS_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TIME_SEQUENCE_RAW__OPTIMIZER_PARAMS_HPP_

namespace autoware::trajectory_processor::time_sequence_raw
{

/// @brief Runtime parameters for the pose-only acados trajectory optimization.
struct TrajectoryOptimizationParams
{
  double weight_longitudinal{0.5};
  double weight_lateral{0.5};
  double weight_yaw{0.05};
  double weight_acceleration{0.1};
  double weight_steering_rate{10.0};
  double terminal_weight_scale{2.5};

  double min_velocity_mps{0.0};
  double max_velocity_mps{30.0};

  double min_acceleration_mps2{-4.0};
  double max_acceleration_mps2{3.0};
  double max_steering_rate_rps{1.0};

  double max_lateral_acceleration_mps2{3.0};

  int max_sqp_iterations{50};
};

}  // namespace autoware::trajectory_processor::time_sequence_raw

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TIME_SEQUENCE_RAW__OPTIMIZER_PARAMS_HPP_
