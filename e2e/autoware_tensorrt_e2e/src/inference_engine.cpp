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

#include <cstdint>
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

namespace
{
//! cudaMalloc's alignment guarantee, and what a tensor address handed to TensorRT has to
//! satisfy; every slot in a block starts on it so any tensor can be read from its slot.
constexpr size_t kTensorAlignment = 256;

size_t align_up(const size_t bytes)
{
  return (bytes + kTensorAlignment - 1) / kTensorAlignment * kTensorAlignment;
}

bool is_aligned(const void * address)
{
  return reinterpret_cast<uintptr_t>(address) % kTensorAlignment == 0;
}
}  // namespace

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
    if (is_input) {
      binding.offset = input_block_bytes_;
      input_block_bytes_ += align_up(binding.byte_size);
      input_specs_.push_back(spec);
      input_bindings_.push_back(std::move(binding));
    } else {
      if (spec.dtype != TensorDataType::kFLOAT32) {
        throw std::runtime_error(
          "Engine output tensor '" + spec.name + "' is not float32; only float32 outputs are "
          "supported.");
      }
      binding.offset = output_block_bytes_;
      output_block_bytes_ += align_up(binding.byte_size);
      output_specs_.push_back(spec);
      output_bindings_.push_back(std::move(binding));
    }
  }
  if (output_specs_.empty()) {
    throw std::runtime_error("The engine has no output tensors.");
  }

  // One device block and one pinned block per direction. A tick's host tensors are staged
  // side by side in the pinned block and go up the bus in one copy per contiguous run,
  // instead of one pageable-memory copy each -- a dozen of those, each of which the
  // runtime has to stage and wait on, cost more than the bytes they move. The outputs come
  // back the same way, in one copy.
  device_inputs_ = autoware::cuda_utils::make_unique<uint8_t[]>(input_block_bytes_);
  pinned_inputs_ = autoware::cuda_utils::make_unique_host<uint8_t[]>(
    input_block_bytes_, cudaHostAllocDefault);
  device_outputs_ = autoware::cuda_utils::make_unique<uint8_t[]>(output_block_bytes_);
  pinned_outputs_ = autoware::cuda_utils::make_unique_host<uint8_t[]>(
    output_block_bytes_, cudaHostAllocDefault);
  for (auto & binding : input_bindings_) {
    binding.device = device_inputs_.get() + binding.offset;
    bind_address(binding, binding.device);
  }
  for (auto & binding : output_bindings_) {
    binding.device = device_outputs_.get() + binding.offset;
    binding.pinned = reinterpret_cast<const float *>(pinned_outputs_.get() + binding.offset);
    bind_address(binding, binding.device);
  }
}

void InferenceEngine::bind_address(Binding & binding, const void * address)
{
  if (binding.bound_address == address) {
    return;
  }
  if (!trt_common_->setTensorAddress(binding.spec.name.c_str(), const_cast<void *>(address))) {
    throw std::runtime_error(
      "Failed to bind device buffer for engine tensor '" + binding.spec.name + "'");
  }
  binding.bound_address = address;
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
    binding.host_fed = false;
    // The tensor was produced on this stream (InputProviderInterface::bind_stream), so the
    // network can read it where it is: no copy, and stream order guarantees it is complete.
    if (is_aligned(tensor.device_data)) {
      bind_address(binding, tensor.device_data);
      return "";
    }
    // A provider buffer that is not aligned for TensorRT goes through this tensor's slot.
    bind_address(binding, binding.device);
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      binding.device, tensor.device_data, binding.byte_size, cudaMemcpyDeviceToDevice, stream_));
    return "";
  }

  binding.host_fed = true;
  bind_address(binding, binding.device);
  uint8_t * staging = pinned_inputs_.get() + binding.offset;
  switch (spec.dtype) {
    case TensorDataType::kFLOAT32: {
      std::memcpy(staging, tensor.host_data.data(), binding.byte_size);
      break;
    }
    case TensorDataType::kBOOL: {
      for (size_t i = 0; i < tensor.host_data.size(); ++i) {
        staging[i] = tensor.host_data[i] > std::numeric_limits<float>::epsilon() ? 1 : 0;
      }
      break;
    }
    case TensorDataType::kINT32: {
      auto * staging_i32 = reinterpret_cast<int32_t *>(staging);
      for (size_t i = 0; i < tensor.host_data.size(); ++i) {
        staging_i32[i] = static_cast<int32_t>(tensor.host_data[i]);
      }
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

  // Bindings are laid out in manifest order, so consecutive host-fed tensors are one
  // contiguous span of both blocks: copy each such run at once. Device-fed tensors sit
  // between runs and are skipped -- a BEV history is tens of megabytes that must never
  // ride along.
  const auto flush = [this](const size_t begin, const size_t end) {
    if (end > begin) {
      CHECK_CUDA_ERROR(cudaMemcpyAsync(
        device_inputs_.get() + begin, pinned_inputs_.get() + begin, end - begin,
        cudaMemcpyHostToDevice, stream_));
    }
  };
  size_t run_begin = 0;
  size_t run_end = 0;
  for (const auto & binding : input_bindings_) {
    if (binding.host_fed) {
      if (run_end == run_begin) {
        run_begin = binding.offset;
      }
      run_end = binding.offset + binding.byte_size;
    } else {
      flush(run_begin, run_end);
      run_begin = run_end = 0;
    }
  }
  flush(run_begin, run_end);

  if (!trt_common_->enqueueV3(stream_)) {
    result.error_msg = "Failed to enqueue inference";
    return result;
  }
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    pinned_outputs_.get(), device_outputs_.get(), output_block_bytes_, cudaMemcpyDeviceToHost,
    stream_));
  // The one host synchronization of the tick: everything a provider queued on this stream,
  // the network, and the output copy have completed when it returns.
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  TensorMap outputs;
  for (const auto & binding : output_bindings_) {
    const size_t count = static_cast<size_t>(binding.spec.num_elements());
    outputs.emplace(
      binding.spec.name,
      Tensor::from_host(binding.spec.shape, std::vector<float>(binding.pinned, binding.pinned + count)));
  }
  result.outputs = std::move(outputs);
  return result;
}

}  // namespace autoware::tensorrt_e2e
