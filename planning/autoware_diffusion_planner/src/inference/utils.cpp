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

#include "autoware/diffusion_planner/inference/utils.hpp"

#include <autoware/tensorrt_common/tensorrt_common.hpp>

#include <NvInfer.h>

#include <cstddef>
#include <filesystem>
#include <functional>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{

using autoware::tensorrt_common::NetworkIO;
using autoware::tensorrt_common::ProfileDims;
using autoware::tensorrt_common::Profiler;
using autoware::tensorrt_common::TrtCommon;

CudaGraphExecutor::~CudaGraphExecutor()
{
  reset();
}

bool CudaGraphExecutor::is_captured() const
{
  return graph_exec_ != nullptr;
}

bool CudaGraphExecutor::capture_failed() const
{
  return capture_failed_;
}

void CudaGraphExecutor::reset_graphs()
{
  if (graph_exec_) {
    cudaGraphExecDestroy(graph_exec_);
    graph_exec_ = nullptr;
  }
  if (graph_) {
    cudaGraphDestroy(graph_);
    graph_ = nullptr;
  }
}

void CudaGraphExecutor::reset()
{
  reset_graphs();
  capture_failed_ = false;
}

bool CudaGraphExecutor::capture(
  cudaStream_t stream, const std::function<bool(cudaStream_t)> & enqueue)
{
  if (capture_failed_) {
    return false;
  }
  reset_graphs();
  if (cudaStreamBeginCapture(stream, cudaStreamCaptureModeGlobal) != cudaSuccess) {
    capture_failed_ = true;
    return false;
  }

  const bool status = enqueue(stream);
  if (cudaStreamEndCapture(stream, &graph_) != cudaSuccess || !status || !graph_) {
    reset_graphs();
    capture_failed_ = true;
    return false;
  }

  if (cudaGraphInstantiate(&graph_exec_, graph_, 0) != cudaSuccess) {
    reset_graphs();
    capture_failed_ = true;
    return false;
  }
  return true;
}

bool CudaGraphExecutor::launch(cudaStream_t stream)
{
  if (!graph_exec_) {
    return false;
  }
  if (cudaGraphLaunch(graph_exec_, stream) != cudaSuccess) {
    capture_failed_ = true;
    reset_graphs();
    return false;
  }
  return true;
}

size_t num_elements(const std::vector<int64_t> & shape)
{
  return std::accumulate(shape.begin(), shape.end(), size_t{1}, std::multiplies<>());
}

ProfileDims make_profile_dims(
  const std::string & name, const nvinfer1::Dims & dims, const int batch_size)
{
  nvinfer1::Dims min_dims = dims;
  nvinfer1::Dims opt_dims = dims;
  nvinfer1::Dims max_dims = dims;
  min_dims.d[0] = 1;
  opt_dims.d[0] = batch_size;
  max_dims.d[0] = batch_size;
  return ProfileDims{name, min_dims, opt_dims, max_dims};
}

std::string engine_file_path(
  const std::string & model_path, const int batch_size, const std::string & precision)
{
  const std::filesystem::path engine_path(model_path);
  return (engine_path.parent_path() / (engine_path.stem().string() + "_batch" +
                                       std::to_string(batch_size) + "_" + precision + ".engine"))
    .string();
}

std::unique_ptr<TrtCommon> setup_engine(
  const std::string & model_path, const std::string & plugins_path, const int batch_size,
  const std::string & precision, const std::vector<NetworkIO> & network_io,
  const std::vector<ProfileDims> & profile_dims)
{
  const auto trt_config = autoware::tensorrt_common::TrtCommonConfig(
    model_path, precision, engine_file_path(model_path, batch_size, precision), 1ULL << 30, -1,
    false);
  auto trt_ptr = std::make_unique<TrtCommon>(
    trt_config, std::make_shared<Profiler>(), std::vector<std::string>{plugins_path});

  auto builder_config = trt_ptr->getBuilderConfig();
  if (builder_config) {
    builder_config->setMaxAuxStreams(0);
  }

  auto network_io_ptr = std::make_unique<std::vector<NetworkIO>>(network_io);
  auto profile_dims_ptr = std::make_unique<std::vector<ProfileDims>>(profile_dims);
  if (!trt_ptr->setup(std::move(profile_dims_ptr), std::move(network_io_ptr))) {
    throw std::runtime_error("Failed to setup TRT engine: " + model_path);
  }
  return trt_ptr;
}

bool enqueue_trt(
  TrtCommon & trt, CudaGraphExecutor & cuda_graph, cudaStream_t stream, const bool use_cuda_graph)
{
  if (!use_cuda_graph || cuda_graph.capture_failed()) {
    return trt.enqueueV3(stream);
  }

  if (cuda_graph.is_captured()) {
    if (cuda_graph.launch(stream)) {
      return true;
    }
    // A graph can become invalid after a TensorRT/plugin or CUDA runtime change.  Mark it as
    // disabled and try the ordinary enqueue for this frame; future frames use the same fallback.
    return trt.enqueueV3(stream);
  }

  const bool status = trt.enqueueV3(stream);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));
  if (!status) {
    return false;
  }

  // A failed capture is a runtime capability issue, not an inference failure.  Keep the normal
  // enqueue result for this frame and permanently fall back to normal TensorRT enqueue for the
  // lifetime of this engine instead of retrying capture on every 10 Hz tick.
  cuda_graph.capture(
    stream, [&trt](cudaStream_t capture_stream) { return trt.enqueueV3(capture_stream); });
  return true;
}

}  // namespace autoware::diffusion_planner
