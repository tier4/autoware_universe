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

#include "dynamics_output_reference.cuh"

#include <mppi/dynamics/dubins/first_order_dubins_bicycle.cuh>
#include <mppi/sampling_distributions/colored_noise/colored_noise.cuh>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace autoware::mppi_optimizer::output_test
{
namespace
{
constexpr float kSentinel = -1234567.0F;
constexpr int kBlocks = 3;
constexpr int kSteps = 80;
constexpr float kDt = 0.1F;

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
    ASSERT_EQ(floatBits(actual[i]), floatBits(expected[i])) << "element=" << i;
  }
}

struct DeviceBuffer
{
  float * data = nullptr;
  explicit DeviceBuffer(const std::size_t count)
  {
    HANDLE_ERROR(cudaMalloc(reinterpret_cast<void **>(&data), count * sizeof(float)));
  }
  ~DeviceBuffer() { cudaFreeNoThrow(data); }
  DeviceBuffer(const DeviceBuffer &) = delete;
  DeviceBuffer & operator=(const DeviceBuffer &) = delete;
};

void upload(float * device, const std::vector<float> & values)
{
  HANDLE_ERROR(
    cudaMemcpy(device, values.data(), values.size() * sizeof(float), cudaMemcpyHostToDevice));
}

std::vector<float> download(const float * device, const std::size_t count)
{
  std::vector<float> result(count);
  HANDLE_ERROR(cudaMemcpy(result.data(), device, count * sizeof(float), cudaMemcpyDeviceToHost));
  return result;
}

std::array<dim3, 7> launchShapes()
{
  // Full X tiles, including partial warps, distinct Z distributions, and Y workers
  // that own no components in the old copy. Include the deployed (32,2,1) shape.
  return {dim3(1, 1, 1),  dim3(32, 1, 1), dim3(32, 2, 1), dim3(31, 3, 2),
          dim3(16, 4, 2), dim3(8, 8, 2),  dim3(2, 32, 2)};
}

template <int OutputDim>
__global__ void copyOutputTiles(
  const float * input, float * output, const int rollouts, const int steps, const int step_stride)
{
  extern __shared__ float tile[];
  const int rollout = blockIdx.x * blockDim.x + threadIdx.x;
  const int local = threadIdx.z * blockDim.x + threadIdx.x;
  for (int t = 0; t < steps; t += step_stride) {
    // Populate by the old per-rollout ownership, independently of the new flat copy.
    for (int c = threadIdx.y; c < OutputDim; c += blockDim.y) {
      tile[local * OutputDim + c] =
        input[((threadIdx.z * rollouts + rollout) * steps + t) * OutputDim + c];
    }
    __syncthreads();
    mppi::kernels::detail::storeRolloutOutputs<OutputDim>(output, tile, rollouts, steps, t);
    __syncthreads();
  }
}

template <int OutputDim>
void checkStoreCases()
{
  // Copies must preserve nonfinite payloads, subnormals and signed zeros as well
  // as ordinary values. Remaining elements have index-dependent finite bits.
  constexpr std::array<std::uint32_t, 8> special{0x00000000U, 0x80000000U, 0x7f800000U,
                                                 0xff800000U, 0x7fc01234U, 0xffc05678U,
                                                 0x00000001U, 0x80000001U};
  // Deliberately offset by one float: the helper must not require vector alignment.
  constexpr int guard = 1;
  for (const auto shape : launchShapes()) {
    for (const int steps : {1, 3, kSteps}) {
      for (const int step_stride : {1, 2}) {
        SCOPED_TRACE(
          ::testing::Message() << "outputs=" << OutputDim << " block=" << shape.x << ',' << shape.y
                               << ',' << shape.z << " steps=" << steps
                               << " stride=" << step_stride);
        const int rollouts = kBlocks * shape.x;
        const std::size_t count = shape.z * rollouts * steps * OutputDim;
        std::vector<float> input(count + 2 * guard, kSentinel);
        std::vector<float> expected(input.size(), kSentinel);
        for (std::size_t i = 0; i < count; ++i) {
          const std::uint32_t bits =
            i % 17 < special.size() ? special[i % 17] : 0x3f000000U + static_cast<std::uint32_t>(i);
          std::memcpy(&input[guard + i], &bits, sizeof(bits));
          const int timestep = (i / OutputDim) % steps;
          if (timestep % step_stride == 0) expected[guard + i] = input[guard + i];
        }
        DeviceBuffer source(input.size()), destination(input.size());
        upload(source.data, input);
        upload(destination.data, std::vector<float>(input.size(), kSentinel));
        const auto shared_bytes = shape.x * shape.z * OutputDim * sizeof(float);
        copyOutputTiles<OutputDim><<<kBlocks, shape, shared_bytes>>>(
          source.data + guard, destination.data + guard, rollouts, steps, step_stride);
        HANDLE_ERROR(cudaGetLastError());
        HANDLE_ERROR(cudaDeviceSynchronize());
        ASSERT_NO_FATAL_FAILURE(expectSameBits(download(destination.data, input.size()), expected));
        ASSERT_NO_FATAL_FAILURE(expectSameBits(download(source.data, input.size()), input));
      }
    }
  }
}

class GpuDynamicsOutput : public ::testing::Test
{
protected:
  void SetUp() override
  {
    int count = 0;
    const auto error = cudaGetDeviceCount(&count);
    if (error == cudaErrorNoDevice || (error == cudaSuccess && count == 0)) {
      GTEST_SKIP() << "CUDA device required for output-store and constraint parity";
    }
    ASSERT_EQ(error, cudaSuccess) << cudaGetErrorString(error);
  }
};

TEST_F(GpuDynamicsOutput, CooperativeStorePreservesBitsAndLayout)
{
  ASSERT_NO_FATAL_FAILURE(checkStoreCases<1>());
  ASSERT_NO_FATAL_FAILURE(checkStoreCases<2>());
  ASSERT_NO_FATAL_FAILURE(checkStoreCases<3>());
  ASSERT_NO_FATAL_FAILURE(checkStoreCases<4>());
  ASSERT_NO_FATAL_FAILURE(checkStoreCases<7>());
  ASSERT_NO_FATAL_FAILURE(checkStoreCases<13>());
  ASSERT_NO_FATAL_FAILURE(checkStoreCases<16>());
  ASSERT_NO_FATAL_FAILURE(checkStoreCases<33>());
}

__global__ void evaluateSharedConstraints(
  FirstOrderDubinsBicycle * model, const float * states, const float * commands,
  const int num_cases, float * snapshots)
{
  using Model = FirstOrderDubinsBicycle;
  constexpr int stride = Model::STATE_DIM + Model::CONTROL_DIM + 2;
  const int local = threadIdx.z * blockDim.x + threadIdx.x;
  const int group = blockIdx.x * blockDim.x * blockDim.z + local;
  extern __shared__ float storage[];
  float * state = storage + local * Model::STATE_DIM;
  float * control =
    storage + blockDim.x * blockDim.z * Model::STATE_DIM + local * Model::CONTROL_DIM;
  // Every X/Z group visits every case, offset so neighbors have different inputs.
  // Reuse shared storage with the same publication/consumption barriers as rollout.
  for (int step = 0; step < num_cases; ++step) {
    const int c = (group + step) % num_cases;
    for (int i = threadIdx.y; i < Model::STATE_DIM; i += blockDim.y) {
      state[i] = states[c * Model::STATE_DIM + i];
    }
    for (int i = threadIdx.y; i < Model::CONTROL_DIM; i += blockDim.y) {
      control[i] = commands[c * Model::CONTROL_DIM + i];
    }
    __syncthreads();
    model->enforceConstraints(state, control);
    __syncthreads();
    float * record = snapshots + (group * num_cases + step) * stride + 1;
    for (int i = threadIdx.y; i < Model::STATE_DIM; i += blockDim.y) record[i] = state[i];
    for (int i = threadIdx.y; i < Model::CONTROL_DIM; i += blockDim.y) {
      record[Model::STATE_DIM + i] = control[i];
    }
    __syncthreads();
  }
}

TEST_F(GpuDynamicsOutput, ControlConstraintsMatchHostAcrossWorkers)
{
  using Model = FirstOrderDubinsBicycle;
  using Params = FirstOrderDubinsBicycleParams;
  using S = Params::StateIndex;
  using C = Params::ControlIndex;
  constexpr int accel = static_cast<int>(C::ACCELERATION_CMD);
  constexpr int steer = static_cast<int>(C::STEER_CMD);
  constexpr int stride = Model::STATE_DIM + Model::CONTROL_DIM + 2;
  struct Case
  {
    const char * name;
    float velocity;
    float previous_steer;
    float previous_rate;
    bool hold;
    bool short_hold;
    float acceleration;
    float steering;
  };
  const std::vector<Case> cases{
    {"positive range", 2.0F, 0.0F, 0.0F, false, false, 8.0F, 0.75F},
    {"negative range", -2.0F, 0.0F, 0.0F, false, false, -8.0F, -0.75F},
    {"held command outside parent range", 0.0625F, 0.375F, -0.125F, true, false, -8.0F, -0.75F},
    {"short reference while moving", 2.0F, -0.375F, 0.125F, false, true, 8.0F, 0.75F},
    {"held command angle limit", 0.0F, 0.75F, 0.0F, true, false, 0.0F, -0.75F},
    {"hold release boundary", 0.125F, 0.375F, 0.0F, true, false, 8.0F, -0.75F},
    {"crossing into reverse", 0.125F, 0.0F, 0.0F, false, false, -8.0F, 0.125F},
    {"already reversing", -0.125F, 0.0F, 0.0F, false, false, -8.0F, 0.125F},
    {"braking without reversal", 2.0F, 0.125F, -0.125F, false, false, -1.0F, -0.125F},
    {"reverse from rest", 0.0F, 0.0F, 0.0F, false, false, -8.0F, 0.75F},
    {"restart rate positive", 0.0F, 0.125F, 0.125F, false, false, 8.0F, 0.75F},
    {"restart rate reversal", 0.0F, 0.125F, 0.125F, false, false, 8.0F, -0.75F},
    {"restart blend", 0.25F, -0.125F, -0.125F, false, false, 1.0F, 0.75F},
    {"restart released", 0.5F, 0.125F, 0.125F, false, false, 1.0F, -0.75F},
    {"reverse hold released", -0.5F, 0.125F, 0.125F, true, false, -1.0F, -0.75F},
    {"short hold in reverse", -2.0F, -0.75F, 0.0F, false, true, -8.0F, 0.75F},
    {"inside positive deadbands", 2.0F, 0.0F, 0.0F, false, false, 0.0625F, 0.015625F},
    {"inside negative deadbands", 2.0F, 0.0F, 0.0F, false, false, -0.0625F, -0.015625F},
    {"positive deadband boundary", 2.0F, 0.0F, 0.0F, false, false, 0.125F, 0.03125F},
    {"negative deadband boundary", 2.0F, 0.0F, 0.0F, false, false, -0.125F, -0.03125F},
    {"outside deadbands", 2.0F, 0.0F, 0.0F, false, false, 0.25F, -0.0625F},
    {"signed zeros", -0.0F, -0.0F, -0.0F, false, false, -0.0F, 0.0F}};
  const int num_cases = static_cast<int>(cases.size());
  for (const bool prevent_reverse : {false, true}) {
    for (const bool deadbands_enabled : {false, true}) {
      for (const float restart_threshold : {0.0F, 0.5F}) {
        SCOPED_TRACE(
          ::testing::Message() << "prevent_reverse=" << prevent_reverse << " deadbands="
                               << deadbands_enabled << " restart_threshold=" << restart_threshold);
        Params params;
        params.prevent_reverse_velocity = prevent_reverse;
        params.max_steer_angle = 0.5F;
        params.standstill_steer_hold_exit_velocity_mps = 0.125F;
        params.restart_velocity_threshold_mps = restart_threshold;
        Model model(params);
        // The narrower parent range makes clamp-before-continuity order observable.
        std::array<float2, Model::CONTROL_DIM> ranges{
          make_float2(-6.0F, 4.0F), make_float2(-0.25F, 0.25F)};
        std::array<float, Model::CONTROL_DIM> deadbands{
          deadbands_enabled ? 0.125F : 0.0F, deadbands_enabled ? 0.03125F : 0.0F};
        model.setControlRanges(ranges);
        model.setControlDeadbands(deadbands);
        Model::control_array zero;
        zero << 0.0625F, -0.015625F;
        model.setZeroControl(zero);
        model.GPUSetup();
        std::vector<float> states, commands, expected;
        for (const auto & c : cases) {
          auto state = Model::state_array::Zero().eval();
          state[static_cast<int>(S::VEL_X)] = c.velocity;
          state[static_cast<int>(S::PREVIOUS_STEER_CMD)] = c.previous_steer;
          state[static_cast<int>(S::PREVIOUS_STEER_CMD_RATE)] = c.previous_rate;
          state[static_cast<int>(S::STEERING_COMMAND_HOLD_ACTIVE)] = c.hold ? 1.0F : 0.0F;
          state[static_cast<int>(S::SHORT_REFERENCE_STEERING_HOLD_ACTIVE)] =
            c.short_hold ? 1.0F : 0.0F;
          states.insert(states.end(), state.data(), state.data() + Model::STATE_DIM);
          Model::control_array control;
          control << c.acceleration, c.steering;
          commands.insert(commands.end(), control.data(), control.data() + Model::CONTROL_DIM);
          // Independent serial execution, not a second parallel rollout through the same race.
          model.enforceConstraints(state, control);
          expected.insert(expected.end(), control.data(), control.data() + Model::CONTROL_DIM);
        }
        // Anchor the fixture to known outcomes, including a hold that must override the range
        // clamp.
        constexpr std::array<float, 5> anchor_steering{0.25F, -0.25F, 0.375F, -0.375F, 0.5F};
        for (std::size_t c = 0; c < anchor_steering.size(); ++c) {
          ASSERT_NEAR(expected[c * Model::CONTROL_DIM + steer], anchor_steering[c], 2.0E-6F);
        }
        ASSERT_LT(expected[5 * Model::CONTROL_DIM + steer], 0.375F);
        ASSERT_FLOAT_EQ(expected[6 * Model::CONTROL_DIM + accel], prevent_reverse ? -1.25F : -6.0F);
        ASSERT_FLOAT_EQ(expected[7 * Model::CONTROL_DIM + accel], -6.0F);
        DeviceBuffer source_states(states.size()), source_commands(commands.size());
        upload(source_states.data, states);
        upload(source_commands.data, commands);
        std::vector<float> serial_device_controls;
        for (const auto shape : launchShapes()) {
          SCOPED_TRACE(
            ::testing::Message() << "block=" << shape.x << ',' << shape.y << ',' << shape.z);
          const int groups = kBlocks * shape.x * shape.z;
          const std::vector<float> sentinel(groups * num_cases * stride, kSentinel);
          DeviceBuffer result(sentinel.size());
          upload(result.data, sentinel);
          const auto shared_bytes =
            shape.x * shape.z * (Model::STATE_DIM + Model::CONTROL_DIM) * sizeof(float);
          evaluateSharedConstraints<<<kBlocks, shape, shared_bytes>>>(
            model.model_d_, source_states.data, source_commands.data, num_cases, result.data);
          HANDLE_ERROR(cudaGetLastError());
          HANDLE_ERROR(cudaDeviceSynchronize());
          const auto values = download(result.data, sentinel.size());
          // The first shape has Y=1. Also require exact device bits across all later shapes.
          if (serial_device_controls.empty()) {
            for (int c = 0; c < num_cases; ++c) {
              const auto offset = c * stride + 1 + Model::STATE_DIM;
              serial_device_controls.insert(
                serial_device_controls.end(), values.begin() + offset,
                values.begin() + offset + Model::CONTROL_DIM);
            }
          }
          for (int group = 0; group < groups; ++group) {
            for (int step = 0; step < num_cases; ++step) {
              const int c = (group + step) % num_cases;
              SCOPED_TRACE(::testing::Message() << cases[c].name << " group=" << group);
              const int offset = (group * num_cases + step) * stride;
              ASSERT_EQ(floatBits(values[offset]), floatBits(kSentinel));
              ASSERT_EQ(floatBits(values[offset + stride - 1]), floatBits(kSentinel));
              for (int i = 0; i < Model::STATE_DIM; ++i) {
                ASSERT_EQ(
                  floatBits(values[offset + 1 + i]), floatBits(states[c * Model::STATE_DIM + i]));
              }
              for (int i = 0; i < Model::CONTROL_DIM; ++i) {
                const float actual = values[offset + 1 + Model::STATE_DIM + i];
                const int index = c * Model::CONTROL_DIM + i;
                // Host/device contraction can differ; ownership changes must retain device bits.
                ASSERT_NEAR(actual, expected[index], 2.0E-6F) << "component=" << i;
                ASSERT_EQ(floatBits(actual), floatBits(serial_device_controls[index]));
              }
            }
          }
          ASSERT_NO_FATAL_FAILURE(
            expectSameBits(download(source_states.data, states.size()), states));
          ASSERT_NO_FATAL_FAILURE(
            expectSameBits(download(source_commands.data, commands.size()), commands));
        }
      }
    }
  }
}

TEST_F(GpuDynamicsOutput, ProductionRolloutMatchesLegacyStoresBits)
{
  using Model = FirstOrderDubinsBicycle;
  using Params = FirstOrderDubinsBicycleParams;
  using S = Params::StateIndex;
  using Sampler = mppi::sampling_distributions::ColoredNoiseDistribution<Params>;
  // Preserve the alignment used by the legacy scalar/vector copy helpers.
  constexpr int guard = 4;
  for (const int delay : {0, 3, Params::kMaxInputDelaySteps}) {
    Params params;
    params.wheel_base = 2.8F;
    params.acc_delay_steps = delay;
    params.steer_delay_steps = delay == 3 ? 2 : delay;
    params.prevent_reverse_velocity = delay != Params::kMaxInputDelaySteps;
    Model model(params);
    std::array<float2, Model::CONTROL_DIM> limits{
      make_float2(-6.0F, 4.0F), make_float2(-0.45F, 0.45F)};
    model.setControlRanges(limits);
    model.GPUSetup();
    for (auto shape : launchShapes()) {
      SCOPED_TRACE(
        ::testing::Message() << "delay=" << delay << " block=" << shape.x << ',' << shape.y << ','
                             << shape.z);
      const int rollouts = kBlocks * shape.x;
      Sampler::SAMPLING_PARAMS_T sampling_params(rollouts, kSteps, shape.z);
      sampling_params.use_same_noise_for_all_distributions = false;
      Sampler sampler(sampling_params);
      sampler.GPUSetup();
      std::vector<float> initial(shape.z * Model::STATE_DIM + 2 * guard, kSentinel);
      for (unsigned int z = 0; z < shape.z; ++z) {
        float * x = initial.data() + guard + z * Model::STATE_DIM;
        std::fill_n(x, Model::STATE_DIM, 0.0F);
        x[static_cast<int>(S::VEL_X)] = z == 0 ? 0.0F : -2.0F;
        x[static_cast<int>(S::POS_X)] = 5.0F + z;
        x[static_cast<int>(S::POS_Y)] = -3.0F * z;
        x[static_cast<int>(S::YAW)] = z == 0 ? -0.0F : 0.25F;
        x[static_cast<int>(S::STEER_ANGLE)] = 0.1F;
        x[static_cast<int>(S::PREVIOUS_STEER_CMD)] = 0.1F;
        x[static_cast<int>(S::STEERING_COMMAND_HOLD_ACTIVE)] = z == 0 ? 1.0F : 0.0F;
        for (int tap = 0; tap < Params::kMaxInputDelaySteps; ++tap) {
          x[static_cast<int>(S::ACCEL_CMD_D0) + tap] = 0.125F * (tap + 1);
          x[static_cast<int>(S::STEER_CMD_D0) + tap] = 0.025F * (tap - 3);
        }
      }
      std::vector<float> commands(shape.z * rollouts * kSteps * Model::CONTROL_DIM);
      for (unsigned int z = 0; z < shape.z; ++z) {
        for (int rollout = 0; rollout < rollouts; ++rollout) {
          for (int t = 0; t < kSteps; ++t) {
            const int offset = ((z * rollouts + rollout) * kSteps + t) * Model::CONTROL_DIM;
            commands[offset] = (t / 10 % 2 ? -7.0F : 5.0F) + 0.125F * (rollout % 7);
            commands[offset + 1] = ((t / 8 + rollout + z) % 2 ? -0.6F : 0.6F);
          }
        }
      }
      const std::size_t output_count = shape.z * rollouts * kSteps * Model::OUTPUT_DIM;
      const std::vector<float> sentinel(output_count + 2 * guard, kSentinel);
      DeviceBuffer input(initial.size()), actual(sentinel.size()), expected(sentinel.size());
      upload(input.data, initial);
      upload(actual.data, sentinel);
      upload(expected.data, sentinel);
      float * controls = sampler.getControlSample(0, 0, 0);
      upload(controls, commands);
      const auto shared_bytes =
        mppi::kernels::calcRolloutDynamicsKernelSharedMemSize(&model, &sampler, shape);
      legacyRolloutDynamics<<<kBlocks, shape, shared_bytes>>>(
        model.model_d_, sampler.sampling_d_, kDt, kSteps, rollouts, input.data + guard,
        expected.data + guard);
      HANDLE_ERROR(cudaGetLastError());
      HANDLE_ERROR(cudaDeviceSynchronize());
      const auto expected_controls = download(controls, commands.size());
      ASSERT_FALSE(std::equal(expected_controls.begin(), expected_controls.end(), commands.begin()))
        << "Fixture must exercise constrained-control writes";
      for (const bool cooperative : {false, true}) {
        SCOPED_TRACE(::testing::Message() << "cooperative=" << cooperative);
        // Both specializations start from the original samples and sentinel outputs.
        upload(controls, commands);
        upload(actual.data, sentinel);
        if (cooperative) {
          mppi::kernels::rolloutDynamicsKernel<Model, Sampler, true>
            <<<kBlocks, shape, shared_bytes>>>(
              model.model_d_, sampler.sampling_d_, kDt, kSteps, rollouts, input.data + guard,
              actual.data + guard);
        } else {
          mppi::kernels::rolloutDynamicsKernel<Model, Sampler, false>
            <<<kBlocks, shape, shared_bytes>>>(
              model.model_d_, sampler.sampling_d_, kDt, kSteps, rollouts, input.data + guard,
              actual.data + guard);
        }
        HANDLE_ERROR(cudaGetLastError());
        HANDLE_ERROR(cudaDeviceSynchronize());
        const auto values = download(actual.data, sentinel.size());
        ASSERT_NO_FATAL_FAILURE(expectSameBits(values, download(expected.data, sentinel.size())));
        ASSERT_NO_FATAL_FAILURE(
          expectSameBits(download(controls, commands.size()), expected_controls));
        ASSERT_NO_FATAL_FAILURE(expectSameBits(download(input.data, initial.size()), initial));
        for (std::size_t i = 0; i < values.size(); ++i) {
          ASSERT_TRUE(std::isfinite(values[i])) << "element=" << i;
          const bool is_guard = i < guard || i >= guard + output_count;
          ASSERT_EQ(values[i] == kSentinel, is_guard) << "element=" << i;
        }
      }
    }
  }
}
}  // namespace
}  // namespace autoware::mppi_optimizer::output_test
