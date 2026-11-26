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

#include "autoware/ml_planner/inference/initial_noise.hpp"

#include "autoware/ml_planner/dimensions.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <random>

namespace autoware::ml_planner
{
xt::xarray<float> create_initial_noise(const std::vector<double> & noise_scales)
{
  const std::array<std::size_t, 4> shape{
    noise_scales.size(), static_cast<std::size_t>(INITIAL_NOISE_SHAPE[1]),
    static_cast<std::size_t>(INITIAL_NOISE_SHAPE[2]),
    static_cast<std::size_t>(INITIAL_NOISE_SHAPE[3])};
  auto noise = xt::xarray<float>::from_shape(shape);
  const auto sample_size = shape[1] * shape[2] * shape[3];
  for (std::size_t batch = 0; batch < noise_scales.size(); ++batch) {
    auto * sample = noise.data() + batch * sample_size;
    const auto scale = noise_scales[batch];
    if (scale == 0.0) {
      std::fill_n(sample, sample_size, 0.0F);
      continue;
    }

    std::random_device random_device;
    std::mt19937 generator(random_device());
    std::normal_distribution<float> distribution(0.0F, 1.0F);
    std::generate_n(
      sample, sample_size, [&] { return distribution(generator) * static_cast<float>(scale); });
  }
  return noise;
}
}  // namespace autoware::ml_planner
