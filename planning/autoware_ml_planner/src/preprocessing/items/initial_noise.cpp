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

#include "autoware/ml_planner/preprocessing/items/initial_noise.hpp"

#include "autoware/ml_planner/dimensions.hpp"

#include <random>

namespace autoware::ml_planner::preprocess
{
xt::xarray<float> create_initial_noise(const double noise_scale)
{
  xt::xarray<float> noise = xt::xarray<float>::from_shape(
    {static_cast<size_t>(MAX_NUM_AGENTS), static_cast<size_t>(OUTPUT_T),
     static_cast<size_t>(POSE_DIM)});
  std::random_device random_device;
  std::mt19937 generator(random_device());
  std::normal_distribution<float> distribution(0.0F, 1.0F);
  for (float & value : noise) {
    value = distribution(generator) * static_cast<float>(noise_scale);
  }
  return noise;
}
}  // namespace autoware::ml_planner::preprocess
