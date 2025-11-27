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

#ifndef WAIT_TIME_ACCUMULATOR_HPP_
#define WAIT_TIME_ACCUMULATOR_HPP_

#include "data_types.hpp"

namespace autoware::time_to_space_trajectory_converter
{
/**
 * @brief Helper class to statefully accumulate wait times as we traverse the spline.
 * Encapsulates the cursor index so the main loop doesn't have to manage it.
 */
class WaitTimeAccumulator
{
public:
  explicit WaitTimeAccumulator(const SplineData & data) : data_(data) {}

  /**
   * @brief Collects all wait times encountered up to the new s position.
   * @param s_limit The current position in the resampling loop.
   * @return The sum of wait times found in this step (0.0 if none).
   */
  double collect(double s_limit);

private:
  const SplineData & data_;
  int current_idx_{0};
};
}  // namespace autoware::time_to_space_trajectory_converter

#endif  // WAIT_TIME_ACCUMULATOR_HPP_
