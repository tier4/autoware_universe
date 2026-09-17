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

#ifndef MPPI__COST_FUNCTIONS__DUBINS__FIRST_ORDER_DUBINS_BICYCLE_KINEMATIC_LIMITS_CUH_
#define MPPI__COST_FUNCTIONS__DUBINS__FIRST_ORDER_DUBINS_BICYCLE_KINEMATIC_LIMITS_CUH_

#include <cuda_runtime_api.h>

#include <cmath>
#include <cstdint>

/** Fixed unit conversions applied before squaring optional kinematic-limit violations. */
inline constexpr float kAccelerationViolationNormalization = 0.5F;
inline constexpr float kJerkViolationNormalization = 0.2F;

inline constexpr std::uint32_t kVelocityLimitActive = 1U << 0U;
inline constexpr std::uint32_t kAccelerationLimitActive = 1U << 1U;
inline constexpr std::uint32_t kJerkLimitActive = 1U << 2U;

/** Compact POD copied to the CUDA cost object for the current optimization cycle. */
struct FirstOrderDubinsBicycleKinematicLimitData
{
  std::uint32_t active_mask = 0U;
  float min_velocity = 0.0F;
  float max_velocity = 0.0F;
  float min_longitudinal_acceleration = 0.0F;
  float max_longitudinal_acceleration = 0.0F;
  float min_longitudinal_jerk = 0.0F;
  float max_longitudinal_jerk = 0.0F;
};

struct FirstOrderDubinsBicycleKinematicCost
{
  float velocity = 0.0F;
  float acceleration = 0.0F;
  float jerk = 0.0F;
  float total = 0.0F;
};

/** Zero inside [lower, upper], normalized and quadratic outside it. */
__host__ __device__ inline float kinematicIntervalViolationSquared(
  const float value, const float lower, const float upper, const float normalization)
{
  const float below = fmaxf(0.0F, lower - value) * normalization;
  const float above = fmaxf(0.0F, value - upper) * normalization;
  return below * below + above * above;
}

/** Normalized optional-limit cost with one shared weight and an explicit numerical safety cap. */
__host__ __device__ inline FirstOrderDubinsBicycleKinematicCost computeCappedKinematicIntervalCost(
  const FirstOrderDubinsBicycleKinematicLimitData & limits, const float coefficient,
  const float maximum_cost, const float velocity, const float longitudinal_acceleration,
  const float longitudinal_jerk)
{
  FirstOrderDubinsBicycleKinematicCost result;
  const float nonnegative_maximum_cost = fmaxf(maximum_cost, 0.0F);
  if (limits.active_mask == 0U || coefficient <= 0.0F || nonnegative_maximum_cost <= 0.0F) {
    return result;
  }

  // Cap in normalized squared-violation space before applying the coefficient. This prevents
  // overflow in extreme rollouts and guarantees that this complete per-step cost cannot exceed
  // crash_contact_penalty, avoiding all-zero MPPI importance weights.
  const float maximum_squared_sum = nonnegative_maximum_cost / coefficient;
  float velocity_squared = 0.0F;
  float acceleration_squared = 0.0F;
  float jerk_squared = 0.0F;
  if ((limits.active_mask & kVelocityLimitActive) != 0U) {
    velocity_squared = fminf(
      kinematicIntervalViolationSquared(velocity, limits.min_velocity, limits.max_velocity, 1.0F),
      maximum_squared_sum);
  }
  if ((limits.active_mask & kAccelerationLimitActive) != 0U) {
    acceleration_squared = fminf(
      kinematicIntervalViolationSquared(
        longitudinal_acceleration, limits.min_longitudinal_acceleration,
        limits.max_longitudinal_acceleration, kAccelerationViolationNormalization),
      maximum_squared_sum);
  }
  if ((limits.active_mask & kJerkLimitActive) != 0U) {
    jerk_squared = fminf(
      kinematicIntervalViolationSquared(
        longitudinal_jerk, limits.min_longitudinal_jerk, limits.max_longitudinal_jerk,
        kJerkViolationNormalization),
      maximum_squared_sum);
  }

  const float squared_sum = velocity_squared + acceleration_squared + jerk_squared;
  if (squared_sum <= 0.0F) {
    return result;
  }
  const float capped_squared_sum = fminf(squared_sum, maximum_squared_sum);
  const float target_cost = fminf(coefficient * capped_squared_sum, nonnegative_maximum_cost);
  const float component_scale = target_cost / squared_sum;
  result.velocity = fminf(component_scale * velocity_squared, target_cost);
  const float after_velocity = fmaxf(target_cost - result.velocity, 0.0F);
  result.acceleration = fminf(component_scale * acceleration_squared, after_velocity);
  result.jerk = fmaxf(after_velocity - result.acceleration, 0.0F);
  result.total = target_cost;
  return result;
}

#endif  // MPPI__COST_FUNCTIONS__DUBINS__FIRST_ORDER_DUBINS_BICYCLE_KINEMATIC_LIMITS_CUH_
