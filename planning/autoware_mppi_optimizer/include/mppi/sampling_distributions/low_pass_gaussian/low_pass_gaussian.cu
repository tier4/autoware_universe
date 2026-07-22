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

#include <mppi/sampling_distributions/low_pass_gaussian/low_pass_gaussian.cuh>

#include <mppi/core/mppi_common.cuh>
#include <mppi/utils/cuda_math_utils.cuh>
#include <mppi/utils/math_utils.h>

#include <cmath>

namespace mppi::sampling_distributions
{
namespace
{

template <int FILTER_ORDER>
__global__ void filterGaussianNoiseAlongHorizon(
  float * noise, const int control_dim, const int num_timesteps, const int num_rollouts,
  const int num_distributions, const float b0, const float b1, const float b2, const float a1,
  const float a2)
{
  const int channel_index = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int num_channels = num_distributions * num_rollouts * control_dim;
  if (channel_index >= num_channels) {
    return;
  }

  const int control_index = channel_index % control_dim;
  const int rollout_and_distribution_index = channel_index / control_dim;
  const int rollout_index = rollout_and_distribution_index % num_rollouts;
  const int distribution_index = rollout_and_distribution_index / num_rollouts;
  const int sequence_start =
    ((distribution_index * num_rollouts + rollout_index) * num_timesteps) * control_dim +
    control_index;

  float x1 = 0.0F;
  float x2 = 0.0F;
  float y1 = 0.0F;
  float y2 = 0.0F;
  for (int time_index = 0; time_index < num_timesteps; ++time_index) {
    const int sample_index = sequence_start + time_index * control_dim;
    const float x0 = noise[sample_index];
    float y0 = b0 * x0 + b1 * x1 - a1 * y1;
    if constexpr (FILTER_ORDER == 2) {
      y0 += b2 * x2 - a2 * y2;
      x2 = x1;
      y2 = y1;
    }
    noise[sample_index] = y0;
    x1 = x0;
    y1 = y0;
  }
}

struct ButterworthCoefficients
{
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
};

ButterworthCoefficients makeButterworthCoefficients(
  const float cutoff_frequency_hz, const float sampling_period_s, const int filter_order)
{
  constexpr float pi = 3.14159265358979323846F;
  const float warped_cutoff = std::tan(pi * cutoff_frequency_hz * sampling_period_s);

  if (filter_order == 1) {
    const float normalization = 1.0F / (1.0F + warped_cutoff);
    return {
      warped_cutoff * normalization, warped_cutoff * normalization, 0.0F,
      (warped_cutoff - 1.0F) * normalization, 0.0F};
  }

  constexpr float sqrt_two = 1.41421356237309504880F;
  const float warped_cutoff_squared = warped_cutoff * warped_cutoff;
  const float normalization =
    1.0F / (1.0F + sqrt_two * warped_cutoff + warped_cutoff_squared);
  return {
    warped_cutoff_squared * normalization, 2.0F * warped_cutoff_squared * normalization,
    warped_cutoff_squared * normalization,
    2.0F * (warped_cutoff_squared - 1.0F) * normalization,
    (1.0F - sqrt_two * warped_cutoff + warped_cutoff_squared) * normalization};
}

}  // namespace

template <class DYN_PARAMS_T>
__host__ void LowPassGaussianDistribution<DYN_PARAMS_T>::generateSamples(
  const int & optimization_stride, const int & iteration_num, curandGenerator_t & gen,
  const bool synchronize)
{
  const int samples_per_distribution =
    this->getNumTimesteps() * this->getNumRollouts() * PARENT_CLASS::CONTROL_DIM;
  const int generated_distributions = this->params_.use_same_noise_for_all_distributions
                                        ? 1
                                        : this->getNumDistributions();
  HANDLE_CURAND_ERROR(curandGenerateNormal(
    gen, this->control_samples_d_, samples_per_distribution * generated_distributions, 0.0F,
    1.0F));

  const ButterworthCoefficients coefficients = makeButterworthCoefficients(
    this->params_.cutoff_frequency_hz, this->params_.sampling_period_s,
    this->params_.filter_order);
  constexpr int threads_per_block = 256;
  const int num_filter_channels =
    generated_distributions * this->getNumRollouts() * PARENT_CLASS::CONTROL_DIM;
  const int filter_blocks = mppi::math::int_ceil(num_filter_channels, threads_per_block);
  if (this->params_.filter_order == 1) {
    filterGaussianNoiseAlongHorizon<1><<<filter_blocks, threads_per_block, 0, this->stream_>>>(
      this->control_samples_d_, PARENT_CLASS::CONTROL_DIM, this->getNumTimesteps(),
      this->getNumRollouts(), generated_distributions, coefficients.b0, coefficients.b1,
      coefficients.b2, coefficients.a1, coefficients.a2);
  } else {
    filterGaussianNoiseAlongHorizon<2><<<filter_blocks, threads_per_block, 0, this->stream_>>>(
      this->control_samples_d_, PARENT_CLASS::CONTROL_DIM, this->getNumTimesteps(),
      this->getNumRollouts(), generated_distributions, coefficients.b0, coefficients.b1,
      coefficients.b2, coefficients.a1, coefficients.a2);
  }
  HANDLE_ERROR(cudaGetLastError());

  if (this->params_.use_same_noise_for_all_distributions) {
    for (int distribution_index = 1; distribution_index < this->getNumDistributions();
         ++distribution_index) {
      HANDLE_ERROR(cudaMemcpyAsync(
        &this->control_samples_d_[samples_per_distribution * distribution_index],
        this->control_samples_d_, sizeof(float) * samples_per_distribution,
        cudaMemcpyDeviceToDevice, this->stream_));
    }
  }

  const int block_size_x = this->params_.rewrite_controls_block_dim.x;
  const int block_size_y = this->params_.rewrite_controls_block_dim.y;
  const int block_size_z = this->params_.rewrite_controls_block_dim.z;
  const dim3 control_writing_grid(
    mppi::math::int_ceil(this->getNumRollouts(), block_size_x),
    mppi::math::int_ceil(this->getNumTimesteps(), block_size_y),
    mppi::math::int_ceil(this->getNumDistributions(), block_size_z));

  unsigned int std_dev_mem_size = this->getNumDistributions() * PARENT_CLASS::CONTROL_DIM;
  std_dev_mem_size = mppi::math::nearest_multiple_4(
    this->params_.time_specific_std_dev
      ? std_dev_mem_size * this->getNumTimesteps()
      : std_dev_mem_size);
  unsigned int shared_mem_size =
    std_dev_mem_size +
    mppi::math::nearest_multiple_4(
      this->getNumDistributions() * this->getNumTimesteps() * PARENT_CLASS::CONTROL_DIM) +
    mppi::math::nearest_multiple_4(
      block_size_x * block_size_y * block_size_z * PARENT_CLASS::CONTROL_DIM);
  shared_mem_size *= sizeof(float);

  setGaussianControls<<<
    control_writing_grid, this->params_.rewrite_controls_block_dim, shared_mem_size,
    this->stream_>>>(
    this->control_means_d_, this->std_dev_d_, this->control_samples_d_,
    PARENT_CLASS::CONTROL_DIM, this->getNumTimesteps(), this->getNumRollouts(),
    this->getNumDistributions(), optimization_stride,
    powf(this->params_.std_dev_decay, iteration_num),
    this->params_.pure_noise_trajectories_percentage, this->params_.time_specific_std_dev);
  HANDLE_ERROR(cudaGetLastError());

  if (synchronize) {
    HANDLE_ERROR(cudaStreamSynchronize(this->stream_));
  }
}

}  // namespace mppi::sampling_distributions
