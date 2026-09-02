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

#include "autoware/mppi_optimizer/curvature_adaptive_steering_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autoware::mppi_optimizer
{

namespace
{

void validate(const CurvatureAdaptiveSteeringFilterParams & params)
{
  if (
    !std::isfinite(params.alpha_straight) || !std::isfinite(params.alpha_turn) ||
    !std::isfinite(params.turn_angle_rad) || params.alpha_straight < 0.0F ||
    params.alpha_straight > params.alpha_turn || params.alpha_turn > 1.0F ||
    params.turn_angle_rad <= 0.0F) {
    throw std::invalid_argument(
      "Curvature-adaptive steering filter requires 0 <= alpha_straight <= alpha_turn <= 1 "
      "and a positive finite turn angle");
  }
}

bool sameParams(
  const CurvatureAdaptiveSteeringFilterParams & lhs,
  const CurvatureAdaptiveSteeringFilterParams & rhs)
{
  return lhs.alpha_straight == rhs.alpha_straight && lhs.alpha_turn == rhs.alpha_turn &&
         lhs.turn_angle_rad == rhs.turn_angle_rad;
}

}  // namespace

CurvatureAdaptiveSteeringFilter::CurvatureAdaptiveSteeringFilter(
  const CurvatureAdaptiveSteeringFilterParams & params)
{
  setParams(params);
}

void CurvatureAdaptiveSteeringFilter::setParams(
  const CurvatureAdaptiveSteeringFilterParams & params)
{
  validate(params);
  if (!sameParams(params_, params)) {
    reset();
  }
  params_ = params;
}

void CurvatureAdaptiveSteeringFilter::reset()
{
  previous_filtered_command_ = 0.0F;
  initialized_ = false;
}

void CurvatureAdaptiveSteeringFilter::filter(
  std::vector<float> & steering_commands, const float measured_steering)
{
  if (steering_commands.empty()) {
    return;
  }

  float previous = initialized_ ? previous_filtered_command_
                                : (std::isfinite(measured_steering) ? measured_steering : 0.0F);
  for (auto & command : steering_commands) {
    const float target = std::isfinite(command) ? command : previous;
    // Use both sides of the transition so entering and leaving a turn receive the fast response.
    const float steering_magnitude = std::max(std::abs(target), std::abs(previous));
    const float turn_ratio = std::clamp(steering_magnitude / params_.turn_angle_rad, 0.0F, 1.0F);
    const float alpha =
      params_.alpha_straight + turn_ratio * (params_.alpha_turn - params_.alpha_straight);
    previous += alpha * (target - previous);
    command = previous;
  }

  previous_filtered_command_ = steering_commands.front();
  initialized_ = true;
}

}  // namespace autoware::mppi_optimizer
