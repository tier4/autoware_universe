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

// Frozen derivative arithmetic from Universe df828f42a6. Keep independent of the
// production scalar helper: this is the numerical oracle, not a second call to it.
#pragma once

#ifndef AUTOWARE_MPPI_OPTIMIZER_TEST_DYNAMICS_DERIVATIVE_REFERENCE_CUH_
#define AUTOWARE_MPPI_OPTIMIZER_TEST_DYNAMICS_DERIVATIVE_REFERENCE_CUH_

#include <mppi/dynamics/dubins/first_order_dubins_bicycle.cuh>

namespace autoware::mppi_optimizer::derivative_test
{
using S = FirstOrderDubinsBicycleParams::StateIndex;
using C = FirstOrderDubinsBicycleParams::ControlIndex;

__host__ __device__ inline int originalAccelDelayTapIndex(const int i)
{
  return static_cast<int>(S::ACCEL_CMD_D0) + i;
}

__host__ __device__ inline int originalSteerDelayTapIndex(const int i)
{
  return static_cast<int>(S::STEER_CMD_D0) + i;
}

/** Resolve plant-facing commands: front of each delay pipe, or raw u when N=0. */
__host__ __device__ inline void originalResolveDelayedControl(
  const FirstOrderDubinsBicycleParams & p, const float * state, const float * control,
  float & accel_cmd, float & steer_cmd)
{
  accel_cmd = control[static_cast<int>(C::ACCELERATION_CMD)];
  steer_cmd = control[static_cast<int>(C::STEER_CMD)];
  const int n_acc = clampInputDelaySteps(p.acc_delay_steps);
  const int n_steer = clampInputDelaySteps(p.steer_delay_steps);
  if (n_acc > 0) {
    accel_cmd = state[originalAccelDelayTapIndex(0)];
  }
  if (n_steer > 0) {
    steer_cmd = state[originalSteerDelayTapIndex(0)];
  }
}

__host__ __device__ inline void originalDerivative(
  const FirstOrderDubinsBicycleParams & p, const float * state, const float * control,
  float * state_der)
{
  const float v = state[static_cast<int>(S::VEL_X)];
  const float yaw = state[static_cast<int>(S::YAW)];
  const float steer = state[static_cast<int>(S::STEER_ANGLE)];
  const float accel = state[static_cast<int>(S::ACCELERATION)];
  float accel_cmd = 0.0F;
  float steer_cmd = 0.0F;
  originalResolveDelayedControl(p, state, control, accel_cmd, steer_cmd);

  const float accel_tau = fmaxf(p.accel_time_constant, 1.0E-4F);
  const float steer_tau = fmaxf(p.steer_time_constant, 1.0E-4F);

  state_der[static_cast<int>(S::ACCELERATION)] = (accel_cmd - accel) / accel_tau;
  state_der[static_cast<int>(S::VEL_X)] = accel;
  state_der[static_cast<int>(S::YAW)] = (v / p.wheel_base) * tanf(steer);

  float sin_yaw = 0.0F;
  float cos_yaw = 0.0F;
  sincosf(yaw, &sin_yaw, &cos_yaw);
  state_der[static_cast<int>(S::POS_X)] = v * cos_yaw;
  state_der[static_cast<int>(S::POS_Y)] = v * sin_yaw;

  // Sampled controls are target steering angles and may jump because of rollout noise. Clamp the
  // realized actuator derivative here so every host/device integration step remains achievable.
  const float steer_dot = clampSteerRate(p, v, (steer_cmd - steer) / steer_tau);
  state_der[static_cast<int>(S::STEER_ANGLE)] = steer_dot;

  state_der[static_cast<int>(S::PREVIOUS_ACCEL_CMD)] = 0.0F;
  state_der[static_cast<int>(S::PREVIOUS_STEER_CMD)] = 0.0F;
  state_der[static_cast<int>(S::PREVIOUS_STEER_CMD_RATE)] = 0.0F;
  state_der[static_cast<int>(S::STEERING_COMMAND_HOLD_ACTIVE)] = 0.0F;
  state_der[static_cast<int>(S::SHORT_REFERENCE_STEERING_HOLD_ACTIVE)] = 0.0F;

  // Delay taps are discrete; keep continuous ders at zero then overwrite in step().
#ifdef __CUDA_ARCH__
#pragma unroll
#endif
  for (int i = 0; i < FirstOrderDubinsBicycleParams::kMaxInputDelaySteps; ++i) {
    state_der[originalAccelDelayTapIndex(i)] = 0.0F;
    state_der[originalSteerDelayTapIndex(i)] = 0.0F;
  }
}

class OriginalDerivativeModel : public FirstOrderDubinsBicycleImpl<OriginalDerivativeModel>
{
public:
  using Base = FirstOrderDubinsBicycleImpl<OriginalDerivativeModel>;
  using Base::computeDynamics;

  explicit OriginalDerivativeModel(FirstOrderDubinsBicycleParams & params) : Base(params) {}

  __device__ void computeDynamics(float * state, float * control, float * state_der, float *)
  {
    // Preserve the original temporary and runtime-indexed gather in the reference.
    float local_der[Base::STATE_DIM];
    originalDerivative(this->params_, state, control, local_der);
    for (int i = threadIdx.y; i < Base::STATE_DIM; i += blockDim.y) {
      state_der[i] = local_der[i];
    }
  }
};
}  // namespace autoware::mppi_optimizer::derivative_test

#endif  // AUTOWARE_MPPI_OPTIMIZER_TEST_DYNAMICS_DERIVATIVE_REFERENCE_CUH_
