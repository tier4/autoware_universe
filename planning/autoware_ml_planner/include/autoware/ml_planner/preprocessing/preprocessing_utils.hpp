// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__ML_PLANNER__PREPROCESSING__PREPROCESSING_UTILS_HPP_
#define AUTOWARE__ML_PLANNER__PREPROCESSING__PREPROCESSING_UTILS_HPP_

#include <tl/expected.hpp>
#include <xtensor/xarray.hpp>

#include <cassert>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::ml_planner::preprocess
{

using TensorMap = std::unordered_map<std::string, xt::xarray<float>>;
using TensorMapResult = tl::expected<TensorMap, std::string>;

/// Apply the fixed scales used by the Python training dataset.
void normalize_input_data(TensorMap & input_data_map);

}  // namespace autoware::ml_planner::preprocess
#endif  // AUTOWARE__ML_PLANNER__PREPROCESSING__PREPROCESSING_UTILS_HPP_
