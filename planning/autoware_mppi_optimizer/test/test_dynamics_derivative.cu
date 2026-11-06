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

#include "dynamics_derivative_reference.cuh"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace autoware::mppi_optimizer::derivative_test
{
namespace
{
using Model = FirstOrderDubinsBicycle;
using Params = FirstOrderDubinsBicycleParams;
constexpr int kRollouts = 64;
constexpr int kSteps = 80;
constexpr int kMaxBlockRollouts = 32;
constexpr float kDt = 0.1F;
constexpr float kSentinel = -1234567.0F;
constexpr int kDerivativeStride = Model::STATE_DIM + 2;
// One guard at each end; derivatives, next state, outputs and applied controls in between.
constexpr int kSnapshotStride = 2 * Model::STATE_DIM + Model::OUTPUT_DIM + Model::CONTROL_DIM + 2;

std::uint32_t floatBits(const float value)
{
  std::uint32_t bits;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

void expectSameBits(const std::vector<float> & actual, const std::vector<float> & expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < actual.size(); ++i) {
    ASSERT_TRUE(std::isfinite(actual[i])) << "element=" << i;
    ASSERT_EQ(floatBits(actual[i]), floatBits(expected[i]))
      << "element=" << i << " actual=" << actual[i] << " expected=" << expected[i];
  }
}

template <class T>
struct DeviceBuffer
{
  T * data = nullptr;
  explicit DeviceBuffer(const std::size_t count)
  {
    HANDLE_ERROR(cudaMalloc(reinterpret_cast<void **>(&data), count * sizeof(T)));
  }
  ~DeviceBuffer() { cudaFreeNoThrow(data); }
  DeviceBuffer(const DeviceBuffer &) = delete;
  DeviceBuffer & operator=(const DeviceBuffer &) = delete;
};

void upload(DeviceBuffer<float> & device, const std::vector<float> & values)
{
  HANDLE_ERROR(
    cudaMemcpy(device.data, values.data(), values.size() * sizeof(float), cudaMemcpyHostToDevice));
}

std::vector<float> download(const DeviceBuffer<float> & device, const std::size_t count)
{
  std::vector<float> result(count);
  HANDLE_ERROR(
    cudaMemcpy(result.data(), device.data, result.size() * sizeof(float), cudaMemcpyDeviceToHost));
  return result;
}

std::vector<Params> parameterCases()
{
  Params base;
  base.wheel_base = 2.8F;
  std::vector<Params> cases{base};
  auto delayed = base;
  delayed.acc_delay_steps = 2;
  delayed.steer_delay_steps = 3;
  cases.push_back(delayed);
  delayed.acc_delay_steps = delayed.steer_delay_steps = Params::kMaxInputDelaySteps;
  cases.push_back(delayed);
  delayed.acc_delay_steps = -3;
  delayed.steer_delay_steps = Params::kMaxInputDelaySteps + 3;
  cases.push_back(delayed);
  delayed.acc_delay_steps = Params::kMaxInputDelaySteps + 1;
  delayed.steer_delay_steps = -1;
  delayed.prevent_reverse_velocity = false;
  cases.push_back(delayed);
  auto small_tau = base;
  small_tau.accel_time_constant = 0.0F;
  small_tau.steer_time_constant = -0.001F;
  cases.push_back(small_tau);
  auto zero_limit = base;
  zero_limit.max_lateral_jerk_mps3 = 0.0F;
  zero_limit.standstill_steer_rate_lim = 0.0F;
  zero_limit.restart_velocity_threshold_mps = 0.0F;
  cases.push_back(zero_limit);
  return cases;
}

std::array<dim3, 6> launchShapes()
{
  // X/Z select independent rollouts. Y=3 exercises a non-power-of-two stride;
  // Y=32 includes workers owning no derivative. Every block contains complete rollouts.
  return {dim3(32, 1, 1), dim3(32, 2, 1), dim3(16, 3, 2),
          dim3(16, 4, 2), dim3(8, 8, 2),  dim3(2, 32, 2)};
}

struct Inputs
{
  std::vector<float> state = std::vector<float>(kRollouts * Model::STATE_DIM);
  std::vector<float> control = std::vector<float>(kRollouts * Model::CONTROL_DIM);
  std::vector<float> commands = std::vector<float>(kRollouts * kSteps * Model::CONTROL_DIM);
};

Inputs makeInputs(const Params & p)
{
  Inputs result;
  const float below_release = std::nextafter(p.restart_velocity_threshold_mps, 0.0F);
  const float above_release =
    std::nextafter(p.restart_velocity_threshold_mps, std::numeric_limits<float>::infinity());
  const std::array<float, 16> velocities{
    0.0F,          -0.0F,          0.0001F, -0.0001F,      0.079F,
    0.08F,         0.081F,         -0.08F,  below_release, p.restart_velocity_threshold_mps,
    above_release, -above_release, 2.0F,    -2.0F,         25.0F,
    -25.0F};
  const std::array<float, 8> yaw{0.0F, -0.0F, 0.1F, -0.1F, 3.1415927F, -3.1415927F, 7.0F, -7.0F};
  const std::array<float, 8> accelerations{0.0F, -0.0F, -7.0F, -6.0F, -1.0F, 1.0F, 4.0F, 5.0F};
  for (int rollout = 0; rollout < kRollouts; ++rollout) {
    float * x = result.state.data() + rollout * Model::STATE_DIM;
    float * u = result.control.data() + rollout * Model::CONTROL_DIM;
    const float velocity = velocities[rollout % velocities.size()];
    const float steering = (rollout % 3 - 1) * p.max_steer_angle;
    x[static_cast<int>(S::VEL_X)] = velocity;
    x[static_cast<int>(S::YAW)] = yaw[rollout % yaw.size()];
    x[static_cast<int>(S::POS_X)] = 0.125F * rollout;
    x[static_cast<int>(S::POS_Y)] = -0.25F * rollout;
    x[static_cast<int>(S::STEER_ANGLE)] = steering;
    x[static_cast<int>(S::ACCELERATION)] = accelerations[rollout % accelerations.size()];
    x[static_cast<int>(S::PREVIOUS_ACCEL_CMD)] = 0.125F * (rollout % 5 - 2);
    x[static_cast<int>(S::PREVIOUS_STEER_CMD)] = 0.1F * (rollout % 3 - 1);
    x[static_cast<int>(S::PREVIOUS_STEER_CMD_RATE)] = 0.05F * (rollout % 5 - 2);
    x[static_cast<int>(S::STEERING_COMMAND_HOLD_ACTIVE)] = (rollout % 4 == 0) ? 1.0F : 0.0F;
    x[static_cast<int>(S::SHORT_REFERENCE_STEERING_HOLD_ACTIVE)] = (rollout % 7 == 0) ? 1.0F : 0.0F;
    for (int tap = 0; tap < Params::kMaxInputDelaySteps; ++tap) {
      x[static_cast<int>(S::ACCEL_CMD_D0) + tap] = 0.125F * (tap + 1) * (rollout % 3 - 1);
      x[static_cast<int>(S::STEER_CMD_D0) + tap] = -0.2F + 0.025F * tap;
    }
    u[static_cast<int>(C::ACCELERATION_CMD)] = accelerations[(rollout + 3) % accelerations.size()];
    // Inputs on either side of the physical steering-rate clamp, including delayed commands.
    const float requested_rate = (rollout % 2 ? -1.0F : 1.0F) * steeringRateLimit(p, velocity);
    float command = steering + requested_rate * std::max(p.steer_time_constant, 1.0E-4F);
    if (rollout % 3 != 0) {
      command = std::nextafter(
        command, rollout % 3 == 1 ? -std::numeric_limits<float>::infinity()
                                  : std::numeric_limits<float>::infinity());
    }
    u[static_cast<int>(C::STEER_CMD)] = command;
    // Keep some front taps distinct from raw commands, so delay selection is observable too.
    if (rollout % 4 != 0) x[static_cast<int>(S::STEER_CMD_D0)] = command;
    for (int t = 0; t < kSteps; ++t) {
      const int offset = (rollout * kSteps + t) * Model::CONTROL_DIM;
      result.commands[offset] = t < 10 ? u[0] : (t / 10 % 2 ? 4.5F : -6.5F);
      result.commands[offset + 1] = t < 10 ? u[1] : (t / 10 % 2 ? 0.6F : -0.6F);
    }
  }
  return result;
}

template <class ModelT>
__global__ void derivativeOwnership(
  ModelT * model, float * states, float * controls, float * results)
{
  const int rollout = (blockIdx.x * blockDim.z + threadIdx.z) * blockDim.x + threadIdx.x;
  // Give each Y worker a separate sentinel-filled destination. That lets the host verify
  // that it writes precisely its owned entries, including no writes from inactive workers.
  float * destination = results + (rollout * blockDim.y + threadIdx.y) * kDerivativeStride + 1;
  model->computeDynamics(
    states + rollout * Model::STATE_DIM, controls + rollout * Model::CONTROL_DIM, destination,
    nullptr);
}

template <class ModelT>
__global__ void replayDerivatives(
  ModelT * model, const float * initial, const float * commands, float * snapshots)
{
  __shared__ float state[kMaxBlockRollouts * Model::STATE_DIM];
  __shared__ float next[kMaxBlockRollouts * Model::STATE_DIM];
  __shared__ float derivative[kMaxBlockRollouts * Model::STATE_DIM];
  __shared__ float output[kMaxBlockRollouts * Model::OUTPUT_DIM];
  __shared__ float control[kMaxBlockRollouts * Model::CONTROL_DIM];
  const int local = threadIdx.z * blockDim.x + threadIdx.x;
  const int rollout = blockIdx.x * blockDim.x * blockDim.z + local;
  float * x = state + local * Model::STATE_DIM;
  float * xn = next + local * Model::STATE_DIM;
  float * dx = derivative + local * Model::STATE_DIM;
  float * y = output + local * Model::OUTPUT_DIM;
  float * u = control + local * Model::CONTROL_DIM;
  for (int i = threadIdx.y; i < Model::STATE_DIM; i += blockDim.y) {
    x[i] = initial[rollout * Model::STATE_DIM + i];
  }
  __syncthreads();
  for (int t = 0; t < kSteps; ++t) {
    for (int i = threadIdx.y; i < Model::STATE_DIM; i += blockDim.y) {
      dx[i] = xn[i] = kSentinel;
    }
    for (int i = threadIdx.y; i < Model::OUTPUT_DIM; i += blockDim.y) y[i] = kSentinel;
    for (int i = threadIdx.y; i < Model::CONTROL_DIM; i += blockDim.y) {
      u[i] = commands[(rollout * kSteps + t) * Model::CONTROL_DIM + i];
    }
    __syncthreads();
    model->enforceConstraints(x, u);
    __syncthreads();
    model->step(x, xn, dx, u, y, nullptr, t * kDt, kDt);
    __syncthreads();
    float * record = snapshots + (rollout * kSteps + t) * kSnapshotStride + 1;
    for (int i = threadIdx.y; i < Model::STATE_DIM; i += blockDim.y) {
      record[i] = dx[i];
      record[Model::STATE_DIM + i] = xn[i];
    }
    for (int i = threadIdx.y; i < Model::OUTPUT_DIM; i += blockDim.y) {
      record[2 * Model::STATE_DIM + i] = y[i];
    }
    for (int i = threadIdx.y; i < Model::CONTROL_DIM; i += blockDim.y) {
      record[2 * Model::STATE_DIM + Model::OUTPUT_DIM + i] = u[i];
    }
    __syncthreads();
    for (int i = threadIdx.y; i < Model::STATE_DIM; i += blockDim.y) x[i] = xn[i];
    __syncthreads();
  }
}

TEST(DynamicsDerivative, ScalarHostMatchesOriginalBits)
{
  for (auto p : parameterCases()) {
    const auto inputs = makeInputs(p);
    Model model(p);
    for (int rollout = 0; rollout < kRollouts; ++rollout) {
      SCOPED_TRACE(
        ::testing::Message() << "delay=" << p.acc_delay_steps << ',' << p.steer_delay_steps
                             << " rollout=" << rollout);
      Model::state_array state;
      Model::control_array control;
      std::copy_n(inputs.state.data() + rollout * Model::STATE_DIM, Model::STATE_DIM, state.data());
      std::copy_n(
        inputs.control.data() + rollout * Model::CONTROL_DIM, Model::CONTROL_DIM, control.data());
      auto actual = Model::state_array::Constant(kSentinel).eval();
      std::vector<float> expected(Model::STATE_DIM, kSentinel);
      originalDerivative(p, state.data(), control.data(), expected.data());
      model.computeDynamics(state, control, actual);
      ASSERT_NO_FATAL_FAILURE(expectSameBits(
        std::vector<float>(actual.data(), actual.data() + Model::STATE_DIM), expected));
    }
  }
}

class GpuDynamicsDerivative : public ::testing::Test
{
protected:
  void SetUp() override
  {
    int count = 0;
    const auto error = cudaGetDeviceCount(&count);
    if (error == cudaErrorNoDevice || (error == cudaSuccess && count == 0)) {
      GTEST_SKIP() << "CUDA device required for derivative parity";
    }
    ASSERT_EQ(error, cudaSuccess) << cudaGetErrorString(error);
  }
};

TEST_F(GpuDynamicsDerivative, ScalarDeviceMatchesOriginalBitsAndWriteOwnership)
{
  for (auto p : parameterCases()) {
    const auto inputs = makeInputs(p);
    Model model(p);
    OriginalDerivativeModel original(p);
    model.GPUSetup();
    original.GPUSetup();
    DeviceBuffer<float> states(inputs.state.size()), controls(inputs.control.size());
    upload(states, inputs.state);
    upload(controls, inputs.control);
    for (const auto shape : launchShapes()) {
      SCOPED_TRACE(
        ::testing::Message() << "delay=" << p.acc_delay_steps << ',' << p.steer_delay_steps
                             << " block=" << shape.x << ',' << shape.y << ',' << shape.z);
      ASSERT_EQ(kRollouts % (shape.x * shape.z), 0U);
      const int blocks = kRollouts / (shape.x * shape.z);
      const std::vector<float> sentinel(kRollouts * shape.y * kDerivativeStride, kSentinel);
      DeviceBuffer<float> actual(sentinel.size()), expected(sentinel.size());
      upload(actual, sentinel);
      upload(expected, sentinel);
      derivativeOwnership<<<blocks, shape>>>(
        original.model_d_, states.data, controls.data, expected.data);
      HANDLE_ERROR(cudaGetLastError());
      derivativeOwnership<<<blocks, shape>>>(
        model.model_d_, states.data, controls.data, actual.data);
      HANDLE_ERROR(cudaGetLastError());
      HANDLE_ERROR(cudaDeviceSynchronize());
      const auto values = download(actual, sentinel.size());
      ASSERT_NO_FATAL_FAILURE(expectSameBits(values, download(expected, sentinel.size())));
      for (int rollout = 0; rollout < kRollouts; ++rollout) {
        for (unsigned int worker = 0; worker < shape.y; ++worker) {
          const int offset = (rollout * shape.y + worker) * kDerivativeStride;
          ASSERT_EQ(floatBits(values[offset]), floatBits(kSentinel));
          ASSERT_EQ(floatBits(values[offset + kDerivativeStride - 1]), floatBits(kSentinel));
          for (int i = 0; i < Model::STATE_DIM; ++i) {
            const bool owned = static_cast<unsigned int>(i) % shape.y == worker;
            ASSERT_EQ(values[offset + 1 + i] != kSentinel, owned)
              << "rollout=" << rollout << " worker=" << worker << " component=" << i;
          }
        }
      }
      ASSERT_NO_FATAL_FAILURE(expectSameBits(download(states, inputs.state.size()), inputs.state));
      ASSERT_NO_FATAL_FAILURE(
        expectSameBits(download(controls, inputs.control.size()), inputs.control));
    }
  }
}

TEST_F(GpuDynamicsDerivative, ScalarReplayMatchesOriginalBits)
{
  for (auto p : parameterCases()) {
    const auto inputs = makeInputs(p);
    Model model(p);
    OriginalDerivativeModel original(p);
    model.GPUSetup();
    original.GPUSetup();
    DeviceBuffer<float> states(inputs.state.size()), commands(inputs.commands.size());
    upload(states, inputs.state);
    upload(commands, inputs.commands);
    const std::vector<float> sentinel(kRollouts * kSteps * kSnapshotStride, kSentinel);
    DeviceBuffer<float> actual(sentinel.size()), expected(sentinel.size());
    for (const auto shape : launchShapes()) {
      SCOPED_TRACE(
        ::testing::Message() << "delay=" << p.acc_delay_steps << ',' << p.steer_delay_steps
                             << " block=" << shape.x << ',' << shape.y << ',' << shape.z);
      ASSERT_LE(shape.x * shape.z, static_cast<unsigned int>(kMaxBlockRollouts));
      ASSERT_EQ(kRollouts % (shape.x * shape.z), 0U);
      const int blocks = kRollouts / (shape.x * shape.z);
      upload(actual, sentinel);
      upload(expected, sentinel);
      replayDerivatives<<<blocks, shape>>>(
        original.model_d_, states.data, commands.data, expected.data);
      HANDLE_ERROR(cudaGetLastError());
      replayDerivatives<<<blocks, shape>>>(model.model_d_, states.data, commands.data, actual.data);
      HANDLE_ERROR(cudaGetLastError());
      HANDLE_ERROR(cudaDeviceSynchronize());
      const auto values = download(actual, sentinel.size());
      ASSERT_NO_FATAL_FAILURE(expectSameBits(values, download(expected, sentinel.size())));
      for (std::size_t i = 0; i < values.size(); ++i) {
        const auto component = i % kSnapshotStride;
        const bool guard = component == 0 || component == kSnapshotStride - 1;
        ASSERT_EQ(values[i] == kSentinel, guard) << "snapshot element=" << i;
      }
      ASSERT_NO_FATAL_FAILURE(expectSameBits(download(states, inputs.state.size()), inputs.state));
      ASSERT_NO_FATAL_FAILURE(
        expectSameBits(download(commands, inputs.commands.size()), inputs.commands));
    }
  }
}
}  // namespace
}  // namespace autoware::mppi_optimizer::derivative_test
