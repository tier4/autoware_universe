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

#include <mppi/sampling_distributions/gaussian/gaussian.cuh>

#include <string>

namespace mppi::sampling_distributions
{

/** Gaussian sampler parameters augmented with the LP-MPPI filter configuration. */
template <int C_DIM>
struct LowPassGaussianParams : public GaussianParams<C_DIM>
{
  /** Butterworth -3 dB cutoff frequency. Must be below the sampling Nyquist frequency. */
  float cutoff_frequency_hz = 1.0F;
  /** Supported Butterworth filter orders are 1 and 2. */
  int filter_order = 2;
  /** Internal discretization period used to convert cutoff_frequency_hz to IIR coefficients. */
  float sampling_period_s = 0.1F;

  LowPassGaussianParams(
    const int num_rollouts = 1, const int num_timesteps = 1, const int num_distributions = 1)
  : GaussianParams<C_DIM>(num_rollouts, num_timesteps, num_distributions)
  {
  }
};

template <class DYN_PARAMS_T>
class LowPassGaussianDistribution
: public GaussianDistributionImpl<
    LowPassGaussianDistribution<DYN_PARAMS_T>, LowPassGaussianParams, DYN_PARAMS_T>
{
public:
  using PARENT_CLASS = GaussianDistributionImpl<
    LowPassGaussianDistribution<DYN_PARAMS_T>, LowPassGaussianParams, DYN_PARAMS_T>;
  using SAMPLING_PARAMS_T = typename PARENT_CLASS::SAMPLING_PARAMS_T;

  explicit LowPassGaussianDistribution(cudaStream_t stream = 0) : PARENT_CLASS(stream) {}

  explicit LowPassGaussianDistribution(
    const SAMPLING_PARAMS_T & params, cudaStream_t stream = 0)
  : PARENT_CLASS(params, stream)
  {
  }

  __host__ std::string getSamplingDistributionName() const override
  {
    return "Low-pass Gaussian";
  }

  /** Generate white Gaussian noise, low-pass filter it along time, then add the nominal control. */
  __host__ void generateSamples(
    const int & optimization_stride, const int & iteration_num, curandGenerator_t & gen,
    bool synchronize = true);
};

}  // namespace mppi::sampling_distributions

#if __CUDACC__
#include "low_pass_gaussian.cu"
#endif
