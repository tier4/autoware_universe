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

#include "autoware/tensorrt_e2e/types.hpp"

#include <algorithm>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::tensorrt_e2e
{

int64_t shape_num_elements(const std::vector<int64_t> & shape)
{
  if (shape.empty()) {
    return 0;
  }
  return std::accumulate(shape.begin(), shape.end(), int64_t{1}, std::multiplies<>());
}

std::string shape_to_string(const std::vector<int64_t> & shape)
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < shape.size(); ++i) {
    oss << shape[i] << (i + 1 < shape.size() ? ", " : "");
  }
  oss << "]";
  return oss.str();
}

int64_t TensorSpec::num_elements() const
{
  return shape_num_elements(shape);
}

int64_t Tensor::num_elements() const
{
  return shape_num_elements(shape);
}

Tensor Tensor::from_host(std::vector<int64_t> tensor_shape, std::vector<float> data)
{
  Tensor tensor;
  tensor.shape = std::move(tensor_shape);
  tensor.host_data = std::move(data);
  return tensor;
}

Tensor Tensor::from_device(std::vector<int64_t> tensor_shape, const float * data)
{
  Tensor tensor;
  tensor.shape = std::move(tensor_shape);
  tensor.device_data = data;
  return tensor;
}

const TensorSpec * find_spec(const std::vector<TensorSpec> & specs, const std::string & name)
{
  const auto it = std::find_if(
    specs.begin(), specs.end(), [&name](const TensorSpec & spec) { return spec.name == name; });
  return it == specs.end() ? nullptr : &(*it);
}

}  // namespace autoware::tensorrt_e2e
