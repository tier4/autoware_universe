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

#ifndef AUTOWARE_MPPI_OPTIMIZER_TEST_DYNAMICS_OUTPUT_REFERENCE_CUH_
#define AUTOWARE_MPPI_OPTIMIZER_TEST_DYNAMICS_OUTPUT_REFERENCE_CUH_

#include <mppi/core/mppi_common.cuh>

namespace autoware::mppi_optimizer::output_test
{
// Frozen split dynamics loop from vendor 56e4b48. Use the production model and
// sampler, but retain the original per-rollout output stores independently of
// storeRolloutOutputs. Do not replace this oracle with a second production call.
template <class Model, class Sampler>
__global__ void legacyRolloutDynamics(
  Model * dynamics, Sampler * sampling, const float dt, const int steps, const int rollouts,
  const float * initial, float * outputs)
{
  const int rollout = blockDim.x * blockIdx.x + threadIdx.x;
  const int local = blockDim.x * threadIdx.z + threadIdx.x;
  const int tile_rollouts = blockDim.x * blockDim.z;
  extern __shared__ float storage[];
  float * state = storage;
  float * next = state + mppi::math::nearest_multiple_4(tile_rollouts * Model::STATE_DIM);
  float * output = next + mppi::math::nearest_multiple_4(tile_rollouts * Model::STATE_DIM);
  float * derivative = output + mppi::math::nearest_multiple_4(tile_rollouts * Model::OUTPUT_DIM);
  float * control = derivative + mppi::math::nearest_multiple_4(tile_rollouts * Model::STATE_DIM);
  float * theta_s = control + mppi::math::nearest_multiple_4(tile_rollouts * Model::CONTROL_DIM);
  float * theta_d =
    theta_s + mppi::kernels::calcClassSharedMemSize(dynamics, blockDim) / sizeof(float);
  float * x = state + local * Model::STATE_DIM;
  float * xn = next + local * Model::STATE_DIM;
  float * y = output + local * Model::OUTPUT_DIM;
  float * dx = derivative + local * Model::STATE_DIM;
  float * u = control + local * Model::CONTROL_DIM;
  mppi::kernels::loadGlobalToShared<Model::STATE_DIM, Model::CONTROL_DIM>(
    rollouts, blockDim.y, rollout, threadIdx.y, threadIdx.z, initial, x, dx, u);
  __syncthreads();
  dynamics->initializeDynamics(x, u, y, theta_s, 0.0F, dt);
  sampling->initializeDistributions(y, 0.0F, dt, theta_d);
  __syncthreads();
  for (int t = 0; t < steps; ++t) {
    sampling->readControlSample(rollout, t, threadIdx.z, u, theta_d, blockDim.y, threadIdx.y, y);
    __syncthreads();
    dynamics->enforceConstraints(x, u);
    __syncthreads();
    sampling->writeControlSample(rollout, t, threadIdx.z, u, theta_d, blockDim.y, threadIdx.y, y);
    dynamics->step(x, xn, dx, u, y, theta_s, t, dt);
    __syncthreads();
    float * temp = x;
    x = xn;
    xn = temp;
    const int offset = ((rollouts * threadIdx.z + rollout) * steps + t) * Model::OUTPUT_DIM;
    mppi::p1::loadArrayParallel<Model::OUTPUT_DIM>(outputs, offset, y, 0);
    __syncthreads();
  }
}
}  // namespace autoware::mppi_optimizer::output_test

#endif  // AUTOWARE_MPPI_OPTIMIZER_TEST_DYNAMICS_OUTPUT_REFERENCE_CUH_
