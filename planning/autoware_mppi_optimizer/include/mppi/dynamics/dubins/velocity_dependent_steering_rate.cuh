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

#pragma once

#ifndef MPPIGENERIC_VELOCITY_DEPENDENT_STEERING_RATE_CUH
#define MPPIGENERIC_VELOCITY_DEPENDENT_STEERING_RATE_CUH

#include <cmath>

#if defined(__CUDACC__)
#define MPPI_STEERING_RATE_HOST_DEVICE __host__ __device__
#else
#define MPPI_STEERING_RATE_HOST_DEVICE
#endif

/**
 * Steering actuator-rate bound shared by host prediction and CUDA rollout propagation.
 * The standstill limit blends smoothly into the normal-speed bound. The hardware limit is
 * retained at normal speeds unless j_lat = v^2 * steer_rate / L is more restrictive.
 */
MPPI_STEERING_RATE_HOST_DEVICE inline float velocityDependentSteeringRateLimit(
  const float velocity, const float wheel_base, const float hardware_limit,
  const float max_lateral_jerk, const float standstill_limit,
  const float restart_velocity_threshold)
{
  const float nonnegative_hardware_limit = fmaxf(hardware_limit, 0.0F);
  const float speed = fabsf(velocity);
  constexpr float kMinimumSquaredSpeed = 1.0E-8F;
  const float jerk_limited_rate = fmaxf(max_lateral_jerk, 0.0F) * fmaxf(wheel_base, 0.0F) /
                                  fmaxf(speed * speed, kMinimumSquaredSpeed);
  const float moving_limit = fminf(nonnegative_hardware_limit, jerk_limited_rate);
  const float stopped_limit = fminf(fmaxf(standstill_limit, 0.0F), nonnegative_hardware_limit);
  const float release_speed = fmaxf(restart_velocity_threshold, 0.0F);
  if (release_speed <= 1.0E-6F || speed >= release_speed) {
    return moving_limit;
  }
  const float ratio = fminf(fmaxf(speed / release_speed, 0.0F), 1.0F);
  const float blend = ratio * ratio * (3.0F - 2.0F * ratio);
  return stopped_limit + blend * (moving_limit - stopped_limit);
}

#undef MPPI_STEERING_RATE_HOST_DEVICE

#endif  // MPPIGENERIC_VELOCITY_DEPENDENT_STEERING_RATE_CUH
