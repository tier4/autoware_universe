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

#include "autoware/tensorrt_e2e/inference_engine.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/tensorrt_common/utils.hpp>

#include <NvInfer.h>

#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::tensorrt_e2e
{
using autoware::tensorrt_common::Profiler;
using autoware::tensorrt_common::TrtCommon;
using autoware::tensorrt_common::TrtCommonConfig;

namespace
{

size_t dtype_size(const TensorDataType dtype)
{
  switch (dtype) {
    case TensorDataType::kFLOAT32:
      return sizeof(float);
    case TensorDataType::kBOOL:
      return sizeof(uint8_t);
    case TensorDataType::kINT32:
      return sizeof(int32_t);
  }
  return sizeof(float);
}

TensorDataType to_tensor_dtype(const nvinfer1::DataType dtype, const std::string & name)
{
  switch (dtype) {
    case nvinfer1::DataType::kFLOAT:
      return TensorDataType::kFLOAT32;
    case nvinfer1::DataType::kBOOL:
      return TensorDataType::kBOOL;
    case nvinfer1::DataType::kINT32:
      return TensorDataType::kINT32;
    default:
      throw std::runtime_error(
        "Engine tensor '" + name +
        "' has an unsupported data type. Supported types: float32, bool, int32.");
  }
}

}  // namespace

InferenceEngine::InferenceEngine(const Config & config)
{
  load_engine(config);
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
}

InferenceEngine::~InferenceEngine()
{
  if (stream_) {
    cudaStreamDestroy(stream_);
  }
}

void InferenceEngine::load_engine(const Config & config)
{
  const auto trt_config = TrtCommonConfig(
    config.model_path, config.precision, "", config.max_workspace_size, -1, false);

  std::vector<std::string> plugin_paths;
  if (!config.plugins_path.empty()) {
    plugin_paths.push_back(config.plugins_path);
  }
  trt_common_ = std::make_unique<TrtCommon>(trt_config, std::make_shared<Profiler>(), plugin_paths);

  // Force single-stream execution to reduce scratch memory (same rationale as the diffusion
  // planner: large transformer models allocate hundreds of MB of auxiliary stream scratch).
  auto builder_config = trt_common_->getBuilderConfig();
  if (builder_config) {
    builder_config->setMaxAuxStreams(0);
  }

  if (!trt_common_->setup()) {
    throw std::runtime_error("Failed to setup TensorRT engine from " + config.model_path);
  }

  introspect_and_bind();
}

void InferenceEngine::introspect_and_bind()
{
  const int32_t num_io = trt_common_->getNbIOTensors();
  for (int32_t i = 0; i < num_io; ++i) {
    const char * name = trt_common_->getIOTensorName(i);
    const nvinfer1::Dims dims = trt_common_->getTensorShape(name);
    const auto nv_dtype = trt_common_->getTensorDataType(name);
    if (!nv_dtype) {
      throw std::runtime_error(std::string("Failed to query dtype of engine tensor '") + name + "'");
    }
    const bool is_input = trt_common_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT;

    TensorSpec spec;
    spec.name = name;
    spec.dtype = to_tensor_dtype(*nv_dtype, spec.name);
    spec.shape.reserve(dims.nbDims);
    for (int32_t d = 0; d < dims.nbDims; ++d) {
      int64_t dim = dims.d[d];
      if (dim < 0) {
        if (d == 0) {
          dim = 1;  // Dynamic batch dimension: this node always runs with batch 1.
        } else {
          throw std::runtime_error(
            "Engine tensor '" + spec.name + "' has a dynamic non-batch dimension (dim " +
            std::to_string(d) + "). Export the model with static shapes.");
        }
      }
      spec.shape.push_back(dim);
    }

    if (is_input && dims.nbDims > 0 && dims.d[0] < 0) {
      nvinfer1::Dims resolved = dims;
      resolved.d[0] = 1;
      if (!trt_common_->setInputShape(spec.name.c_str(), resolved)) {
        throw std::runtime_error("Failed to set batch-1 shape for engine tensor '" + spec.name + "'");
      }
    }

    Binding binding;
    binding.spec = spec;
    binding.byte_size = static_cast<size_t>(spec.num_elements()) * dtype_size(spec.dtype);
    binding.device = autoware::cuda_utils::make_unique<uint8_t[]>(binding.byte_size);

    if (!trt_common_->setTensorAddress(spec.name.c_str(), binding.device.get())) {
      throw std::runtime_error("Failed to bind device buffer for engine tensor '" + spec.name + "'");
    }

    if (is_input) {
      input_specs_.push_back(spec);
      input_bindings_.push_back(std::move(binding));
    } else {
      if (spec.dtype != TensorDataType::kFLOAT32) {
        throw std::runtime_error(
          "Engine output tensor '" + spec.name + "' is not float32; only float32 outputs are "
          "supported.");
      }
      binding.pinned = autoware::cuda_utils::make_unique_host<float[]>(
        static_cast<size_t>(spec.num_elements()), cudaHostAllocDefault);
      output_specs_.push_back(spec);
      output_bindings_.push_back(std::move(binding));
    }
  }

  if (output_specs_.empty()) {
    throw std::runtime_error("The engine has no output tensors.");
  }
}

std::string InferenceEngine::transfer_input(Binding & binding, const Tensor & tensor)
{
  const auto & spec = binding.spec;
  if (tensor.num_elements() != spec.num_elements()) {
    return "Input tensor '" + spec.name + "' has " + std::to_string(tensor.num_elements()) +
           " elements (shape " + shape_to_string(tensor.shape) + "), but the engine expects " +
           std::to_string(spec.num_elements()) + " (shape " + shape_to_string(spec.shape) + ")";
  }

  if (tensor.is_device()) {
    if (spec.dtype != TensorDataType::kFLOAT32) {
      return "Device-resident input '" + spec.name + "' requires a float32 engine tensor";
    }
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      binding.device.get(), tensor.device_data, binding.byte_size, cudaMemcpyDeviceToDevice,
      stream_));
    return "";
  }

  switch (spec.dtype) {
    case TensorDataType::kFLOAT32: {
      CHECK_CUDA_ERROR(cudaMemcpyAsync(
        binding.device.get(), tensor.host_data.data(), binding.byte_size, cudaMemcpyHostToDevice,
        stream_));
      break;
    }
    case TensorDataType::kBOOL: {
      binding.staging.resize(tensor.host_data.size());
      for (size_t i = 0; i < tensor.host_data.size(); ++i) {
        binding.staging[i] = tensor.host_data[i] > std::numeric_limits<float>::epsilon() ? 1 : 0;
      }
      CHECK_CUDA_ERROR(cudaMemcpyAsync(
        binding.device.get(), binding.staging.data(), binding.byte_size, cudaMemcpyHostToDevice,
        stream_));
      break;
    }
    case TensorDataType::kINT32: {
      binding.staging.resize(tensor.host_data.size() * sizeof(int32_t));
      auto * staging_i32 = reinterpret_cast<int32_t *>(binding.staging.data());
      for (size_t i = 0; i < tensor.host_data.size(); ++i) {
        staging_i32[i] = static_cast<int32_t>(tensor.host_data[i]);
      }
      CHECK_CUDA_ERROR(cudaMemcpyAsync(
        binding.device.get(), binding.staging.data(), binding.byte_size, cudaMemcpyHostToDevice,
        stream_));
      break;
    }
  }
  return "";
}

InferenceEngine::Result InferenceEngine::infer(const TensorMap & inputs)
{
  Result result;

  for (auto & binding : input_bindings_) {
    const auto it = inputs.find(binding.spec.name);
    if (it == inputs.end()) {
      result.error_msg = "Missing input tensor '" + binding.spec.name + "'";
      return result;
    }
    const std::string error = transfer_input(binding, it->second);
    if (!error.empty()) {
      result.error_msg = error;
      return result;
    }
  }

  const bool status = trt_common_->enqueueV3(stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  if (!status) {
    result.error_msg = "Failed to enqueue inference";
    return result;
  }

  for (auto & binding : output_bindings_) {
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      binding.pinned.get(), binding.device.get(), binding.byte_size, cudaMemcpyDeviceToHost,
      stream_));
  }
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  TensorMap outputs;
  for (const auto & binding : output_bindings_) {
    const size_t count = static_cast<size_t>(binding.spec.num_elements());
    outputs.emplace(
      binding.spec.name,
      Tensor::from_host(
        binding.spec.shape, std::vector<float>(binding.pinned.get(), binding.pinned.get() + count)));
  }

  result.outputs = std::move(outputs);
  return result;
}

}  // namespace autoware::tensorrt_e2e
