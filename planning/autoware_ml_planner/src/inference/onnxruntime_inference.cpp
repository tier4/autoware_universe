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

#include "autoware/ml_planner/inference/onnxruntime_inference.hpp"

#include "autoware/ml_planner/dimensions.hpp"

#include <onnxruntime_c_api.h>

#include <chrono>
#include <functional>
#include <numeric>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::ml_planner
{
namespace
{
void append_cuda_provider(Ort::SessionOptions & options)
{
  OrtCUDAProviderOptionsV2 * provider_options = nullptr;
  Ort::ThrowOnError(Ort::GetApi().CreateCUDAProviderOptions(&provider_options));
  Ort::ThrowOnError(
    Ort::GetApi().SessionOptionsAppendExecutionProvider_CUDA_V2(options, provider_options));
  Ort::GetApi().ReleaseCUDAProviderOptions(provider_options);
}

void append_tensorrt_provider(Ort::SessionOptions & options, const std::string & plugins_path)
{
  OrtTensorRTProviderOptionsV2 * provider_options = nullptr;
  Ort::ThrowOnError(Ort::GetApi().CreateTensorRTProviderOptions(&provider_options));
  if (!plugins_path.empty()) {
    const char * key = "trt_extra_plugin_lib_paths";
    const char * value = plugins_path.c_str();
    Ort::ThrowOnError(
      Ort::GetApi().UpdateTensorRTProviderOptions(provider_options, &key, &value, 1));
  }
  Ort::ThrowOnError(
    Ort::GetApi().SessionOptionsAppendExecutionProvider_TensorRT_V2(options, provider_options));
  Ort::GetApi().ReleaseTensorRTProviderOptions(provider_options);
  append_cuda_provider(options);
}

template <size_t N>
std::vector<int64_t> batched_shape(const std::array<int64_t, N> & shape, const size_t data_size)
{
  const size_t elements_without_batch =
    std::accumulate(shape.begin() + 1, shape.end(), size_t{1}, std::multiplies<size_t>());
  std::vector<int64_t> result(shape.begin(), shape.end());
  result.front() = static_cast<int64_t>(data_size / elements_without_batch);
  return result;
}

std::vector<int64_t> input_shape(const std::string & name, const size_t size)
{
  if (name == "initial_noise") return batched_shape(INITIAL_NOISE_SHAPE, size);
  if (name == "ego_agent_past") return batched_shape(EGO_HISTORY_SHAPE, size);
  if (name == "neighbor_agents_past") return batched_shape(NEIGHBOR_SHAPE, size);
  if (name == "agent_shape") return batched_shape(AGENT_SHAPE_SHAPE, size);
  if (name == "agent_label") return batched_shape(AGENT_LABEL_SHAPE, size);
  if (name == "lanes") return batched_shape(LANES_SHAPE, size);
  if (name == "lane_types") return batched_shape(LANE_TYPES_SHAPE, size);
  if (name == "lanes_speed_limit") return batched_shape(LANES_SPEED_LIMIT_SHAPE, size);
  if (name == "lane_traffic_light_past") {
    return batched_shape(LANE_TRAFFIC_LIGHT_PAST_SHAPE, size);
  }
  if (name == "lane_traffic_light_future") {
    return batched_shape(LANE_TRAFFIC_LIGHT_FUTURE_SHAPE, size);
  }
  if (name == "route_lanes") return batched_shape(ROUTE_LANES_SHAPE, size);
  if (name == "route_lane_types") return batched_shape(ROUTE_LANE_TYPES_SHAPE, size);
  if (name == "route_lanes_speed_limit") {
    return batched_shape(ROUTE_LANES_SPEED_LIMIT_SHAPE, size);
  }
  if (name == "route_traffic_light_past") {
    return batched_shape(ROUTE_TRAFFIC_LIGHT_PAST_SHAPE, size);
  }
  if (name == "route_traffic_light_future") {
    return batched_shape(ROUTE_TRAFFIC_LIGHT_FUTURE_SHAPE, size);
  }
  if (name == "intersection_area") return batched_shape(INTERSECTION_AREA_SHAPE, size);
  if (name == "stop_lines") return batched_shape(STOP_LINES_SHAPE, size);
  if (name == "road_borders") return batched_shape(ROAD_BORDERS_SHAPE, size);
  if (name == "goal_pose") return batched_shape(GOAL_POSE_SHAPE, size);
  if (name == "ego_shape") return batched_shape(EGO_SHAPE_SHAPE, size);
  if (name == "turn_indicators") return batched_shape(TURN_INDICATORS_SHAPE, size);
  throw std::runtime_error("Unsupported ONNX input: " + name);
}

std::unordered_map<std::string, std::vector<float>> make_inputs(
  const preprocess::TensorMap & input_data_map)
{
  static const std::vector<std::string> names = {
    "initial_noise",
    "ego_agent_past",
    "neighbor_agents_past",
    "agent_shape",
    "agent_label",
    "lanes",
    "lane_types",
    "lanes_speed_limit",
    "lane_traffic_light_past",
    "lane_traffic_light_future",
    "route_lanes",
    "route_lane_types",
    "route_lanes_speed_limit",
    "route_traffic_light_past",
    "route_traffic_light_future",
    "intersection_area",
    "stop_lines",
    "road_borders",
    "goal_pose",
    "ego_shape",
    "turn_indicators"};
  std::unordered_map<std::string, std::vector<float>> inputs;
  for (const auto & name : names) {
    const auto & tensor = input_data_map.at(name);
    inputs.emplace(name, std::vector<float>(tensor.cbegin(), tensor.cend()));
  }
  return inputs;
}
}  // namespace

OnnxruntimeExecutionProvider parse_execution_provider(const std::string & provider)
{
  if (provider == "cpu") return OnnxruntimeExecutionProvider::CPU;
  if (provider == "cuda") return OnnxruntimeExecutionProvider::CUDA;
  if (provider == "tensorrt") return OnnxruntimeExecutionProvider::TensorRT;
  throw std::invalid_argument("Unsupported ONNX Runtime provider: " + provider);
}

OrtModel::OrtModel(
  const std::string & model_path, const OnnxruntimeExecutionProvider provider,
  const std::string & plugins_path)
: env_(ORT_LOGGING_LEVEL_WARNING, "ml_planner"),
  session_(nullptr),
  memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
  session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  if (provider == OnnxruntimeExecutionProvider::CUDA) {
    append_cuda_provider(session_options_);
  } else if (provider == OnnxruntimeExecutionProvider::TensorRT) {
    append_tensorrt_provider(session_options_, plugins_path);
  }
  session_ = Ort::Session(env_, model_path.c_str(), session_options_);
}

std::unordered_map<std::string, std::vector<float>> OrtModel::run(
  const std::unordered_map<std::string, std::vector<float>> & inputs,
  const std::vector<std::string> & output_names)
{
  std::vector<std::string> names;
  std::vector<const char *> name_ptrs;
  std::vector<Ort::Value> tensors;
  names.reserve(inputs.size());
  name_ptrs.reserve(inputs.size());
  tensors.reserve(inputs.size());
  for (const auto & [name, data] : inputs) {
    names.push_back(name);
    name_ptrs.push_back(names.back().c_str());
    const auto shape = input_shape(name, data.size());
    tensors.push_back(
      Ort::Value::CreateTensor<float>(
        memory_info_, const_cast<float *>(data.data()), data.size(), shape.data(), shape.size()));
  }
  std::vector<const char *> output_ptrs;
  for (const auto & name : output_names) output_ptrs.push_back(name.c_str());
  auto output_tensors = session_.Run(
    Ort::RunOptions{nullptr}, name_ptrs.data(), tensors.data(), tensors.size(), output_ptrs.data(),
    output_ptrs.size());

  std::unordered_map<std::string, std::vector<float>> outputs;
  for (size_t index = 0; index < output_tensors.size(); ++index) {
    const size_t size = output_tensors[index].GetTensorTypeAndShapeInfo().GetElementCount();
    const float * data = output_tensors[index].GetTensorData<float>();
    outputs.emplace(output_names[index], std::vector<float>(data, data + size));
  }
  return outputs;
}

OnnxruntimeSingleStepInference::OnnxruntimeSingleStepInference(
  const std::string & model_path, const std::string & provider, const std::string & plugins_path)
: model_(model_path, parse_execution_provider(provider), plugins_path)
{
}

InferenceResult OnnxruntimeSingleStepInference::infer(const preprocess::TensorMap & input_data_map)
{
  const auto start = std::chrono::steady_clock::now();
  try {
    auto outputs = model_.run(make_inputs(input_data_map), {"trajectory", "turn_indicator_logits"});
    InferenceOutput output;
    output.trajectory = std::move(outputs.at("trajectory"));
    output.turn_indicator_logits = std::move(outputs.at("turn_indicator_logits"));
    output.inference_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    output.is_denormalized = false;
    return output;
  } catch (const std::exception & error) {
    return tl::unexpected(std::string{error.what()});
  }
}

}  // namespace autoware::ml_planner
