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

#include "autoware/diffusion_planner/inference/onnxruntime_inference.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"

#include <onnxruntime_c_api.h>

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{
namespace
{
std::vector<uint8_t> make_speed_mask(const std::vector<float> & speed_limit)
{
  std::vector<uint8_t> mask(speed_limit.size(), 0);
  for (size_t i = 0; i < speed_limit.size(); ++i) {
    mask[i] = speed_limit[i] > 0.0f ? 1U : 0U;
  }
  return mask;
}

struct FloatInput
{
  std::string name;
  const std::vector<float> * data;
};

void append_cpu_provider(Ort::SessionOptions &)
{
  // CPUExecutionProvider is enabled by default.
}

void append_cuda_provider(Ort::SessionOptions & session_options)
{
  OrtCUDAProviderOptionsV2 * cuda_options = nullptr;
  Ort::ThrowOnError(Ort::GetApi().CreateCUDAProviderOptions(&cuda_options));
  Ort::ThrowOnError(
    Ort::GetApi().SessionOptionsAppendExecutionProvider_CUDA_V2(session_options, cuda_options));
  Ort::GetApi().ReleaseCUDAProviderOptions(cuda_options);
}

void append_tensorrt_provider(
  Ort::SessionOptions & session_options, const std::string & plugins_path)
{
  OrtTensorRTProviderOptionsV2 * trt_options = nullptr;
  Ort::ThrowOnError(Ort::GetApi().CreateTensorRTProviderOptions(&trt_options));

  std::vector<const char *> keys;
  std::vector<const char *> values;
  if (!plugins_path.empty()) {
    keys.push_back("trt_extra_plugin_lib_paths");
    values.push_back(plugins_path.c_str());
  }
  if (!keys.empty()) {
    Ort::ThrowOnError(
      Ort::GetApi().UpdateTensorRTProviderOptions(
        trt_options, keys.data(), values.data(), keys.size()));
  }
  Ort::ThrowOnError(
    Ort::GetApi().SessionOptionsAppendExecutionProvider_TensorRT_V2(session_options, trt_options));
  Ort::GetApi().ReleaseTensorRTProviderOptions(trt_options);
}

std::vector<FloatInput> encoder_float_inputs(
  const preprocess::InputDataMap & input_data_map, const ModelInputType input_type)
{
  if (input_type == ModelInputType::IMAGE) {
    // Everything drawable is in the raster; only the scene facts with no pixel representation
    // stay as tensors (see image_encoder.py).
    return {
      {"ego_current_state", &input_data_map.at("ego_current_state")},
      {"turn_indicators", &input_data_map.at("turn_indicators")}};
  }
  return {
    {"ego_agent_past", &input_data_map.at("ego_agent_past")},
    {"neighbor_agents_past", &input_data_map.at("neighbor_agents_past")},
    {"static_objects", &input_data_map.at("static_objects")},
    {"lanes", &input_data_map.at("lanes")},
    {"lanes_speed_limit", &input_data_map.at("lanes_speed_limit")},
    {"route_lanes", &input_data_map.at("route_lanes")},
    {"route_lanes_speed_limit", &input_data_map.at("route_lanes_speed_limit")},
    {"polygons", &input_data_map.at("polygons")},
    {"line_strings", &input_data_map.at("line_strings")},
    {"goal_pose", &input_data_map.at("goal_pose")},
    {"ego_shape", &input_data_map.at("ego_shape")},
    {"turn_indicators", &input_data_map.at("turn_indicators")}};
}

// The full graph takes the sampled trajectories and the control delay on top of whatever its
// encoder reads.
std::vector<FloatInput> single_step_float_inputs(
  const preprocess::InputDataMap & input_data_map, const ModelInputType input_type)
{
  std::vector<FloatInput> inputs = encoder_float_inputs(input_data_map, input_type);
  inputs.push_back({"sampled_trajectories", &input_data_map.at("sampled_trajectories")});
  inputs.push_back({"delay", &input_data_map.at("delay")});
  if (input_type == ModelInputType::IMAGE) {
    // The decoder reads the neighbor histories even though the image encoder does not.
    inputs.push_back({"neighbor_agents_past", &input_data_map.at("neighbor_agents_past")});
  }
  return inputs;
}

std::unordered_map<std::string, std::vector<uint8_t>> speed_limit_bool_inputs(
  const preprocess::InputDataMap & input_data_map)
{
  return {
    {"lanes_has_speed_limit", make_speed_mask(input_data_map.at("lanes_speed_limit"))},
    {"route_lanes_has_speed_limit", make_speed_mask(input_data_map.at("route_lanes_speed_limit"))}};
}

// Encoder-side byte inputs: the speed-limit flags for the vector encoder, the raster for the
// image encoder.
std::unordered_map<std::string, std::vector<uint8_t>> encoder_byte_inputs(
  const preprocess::InputDataMap & input_data_map, const ModelInputType input_type,
  const std::vector<uint8_t> & bev_image)
{
  if (input_type == ModelInputType::IMAGE) {
    return {{"bev_image", bev_image}};
  }
  return speed_limit_bool_inputs(input_data_map);
}

std::unordered_map<std::string, std::vector<float>> to_float_input_map(
  const std::vector<FloatInput> & inputs)
{
  std::unordered_map<std::string, std::vector<float>> map;
  for (const auto & input : inputs) {
    map.emplace(input.name, *input.data);
  }
  return map;
}

}  // namespace

OnnxruntimeExecutionProvider parse_execution_provider(const std::string & execution_provider)
{
  if (execution_provider == "cpu") {
    return OnnxruntimeExecutionProvider::CPU;
  }
  if (execution_provider == "cuda") {
    return OnnxruntimeExecutionProvider::CUDA;
  }
  if (execution_provider == "tensorrt") {
    return OnnxruntimeExecutionProvider::TensorRT;
  }
  throw std::invalid_argument(
    "Unsupported model.ort_execution_provider '" + execution_provider +
    "'. Expected 'cpu', 'cuda', or 'tensorrt'.");
}

OrtModel::OrtModel(
  const std::string & model_path, const OnnxruntimeExecutionProvider execution_provider,
  const std::string & plugins_path)
: env_(ORT_LOGGING_LEVEL_WARNING, "diffusion_planner"),
  session_(nullptr),
  memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
  session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

  if (execution_provider == OnnxruntimeExecutionProvider::TensorRT) {
    append_tensorrt_provider(session_options_, plugins_path);
    append_cuda_provider(session_options_);
  } else if (execution_provider == OnnxruntimeExecutionProvider::CUDA) {
    append_cuda_provider(session_options_);
  } else {
    append_cpu_provider(session_options_);
  }

  session_ = Ort::Session(env_, model_path.c_str(), session_options_);

  // Cache what the graph declares about each input, so callers only ever supply payloads. Every
  // dimension but the leading batch is fixed in these exports, and the declared element type is
  // what decides whether a byte payload becomes a BOOL or a UINT8 tensor.
  Ort::AllocatorWithDefaultOptions allocator;
  const size_t input_count = session_.GetInputCount();
  for (size_t i = 0; i < input_count; ++i) {
    const Ort::AllocatedStringPtr name = session_.GetInputNameAllocated(i, allocator);
    // GetTensorTypeAndShapeInfo hands back a non-owning view, so the TypeInfo has to outlive it.
    const Ort::TypeInfo type_info = session_.GetInputTypeInfo(i);
    const auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
    input_specs_.emplace(
      std::string{name.get()}, InputSpec{tensor_info.GetShape(), tensor_info.GetElementType()});
  }
}

const OrtModel::InputSpec & OrtModel::input_spec(const std::string & name) const
{
  const auto it = input_specs_.find(name);
  if (it == input_specs_.end()) {
    throw std::runtime_error("The ONNX graph declares no input named " + name);
  }
  return it->second;
}

std::vector<int64_t> OrtModel::shape_for(const std::string & name, const size_t element_count) const
{
  std::vector<int64_t> shape = input_spec(name).shape;
  if (shape.empty()) {
    throw std::runtime_error("Input " + name + " is declared as a scalar");
  }

  const size_t elements_per_batch = std::accumulate(
    shape.begin() + 1, shape.end(), size_t{1}, [&name](const size_t product, const int64_t dim) {
      if (dim <= 0) {
        throw std::runtime_error("Input " + name + " has a non-batch dynamic dimension");
      }
      return product * static_cast<size_t>(dim);
    });
  if (element_count % elements_per_batch != 0) {
    throw std::runtime_error("Input size mismatch for " + name);
  }
  shape[0] = static_cast<int64_t>(element_count / elements_per_batch);
  return shape;
}

std::unordered_map<std::string, std::vector<float>> OrtModel::run(
  const std::unordered_map<std::string, std::vector<float>> & float_inputs,
  const std::unordered_map<std::string, std::vector<uint8_t>> & byte_inputs,
  const std::vector<std::string> & output_names)
{
  std::vector<std::string> input_names;
  std::vector<const char *> input_name_ptrs;
  std::vector<Ort::Value> input_tensors;
  std::vector<std::vector<int64_t>> input_shapes;
  const auto input_count = float_inputs.size() + byte_inputs.size();
  input_names.reserve(input_count);
  input_name_ptrs.reserve(input_count);
  input_tensors.reserve(input_count);
  input_shapes.reserve(input_count);

  const auto push_name = [&](const std::string & name) {
    input_names.push_back(name);
    input_name_ptrs.push_back(input_names.back().c_str());
  };

  for (const auto & [name, data] : float_inputs) {
    if (input_spec(name).element_type != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
      throw std::runtime_error("Input " + name + " is not declared as float by the ONNX graph");
    }
    input_shapes.push_back(shape_for(name, data.size()));
    const std::vector<int64_t> & shape = input_shapes.back();
    push_name(name);
    input_tensors.push_back(
      Ort::Value::CreateTensor<float>(
        memory_info_, const_cast<float *>(data.data()), data.size(), shape.data(), shape.size()));
  }

  for (const auto & [name, data] : byte_inputs) {
    const ONNXTensorElementDataType element_type = input_spec(name).element_type;
    if (
      element_type != ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL &&
      element_type != ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8) {
      throw std::runtime_error(
        "Input " + name + " is not declared as bool or uint8 by the ONNX graph");
    }
    input_shapes.push_back(shape_for(name, data.size()));
    const std::vector<int64_t> & shape = input_shapes.back();
    push_name(name);
    input_tensors.push_back(
      Ort::Value::CreateTensor(
        memory_info_, const_cast<uint8_t *>(data.data()), data.size() * sizeof(uint8_t),
        shape.data(), shape.size(), element_type));
  }

  std::vector<const char *> output_name_ptrs;
  output_name_ptrs.reserve(output_names.size());
  for (const auto & name : output_names) {
    output_name_ptrs.push_back(name.c_str());
  }

  auto output_tensors = session_.Run(
    Ort::RunOptions{nullptr}, input_name_ptrs.data(), input_tensors.data(), input_tensors.size(),
    output_name_ptrs.data(), output_name_ptrs.size());

  std::unordered_map<std::string, std::vector<float>> outputs;
  for (size_t i = 0; i < output_tensors.size(); ++i) {
    auto info = output_tensors[i].GetTensorTypeAndShapeInfo();
    const auto element_count = info.GetElementCount();
    const float * data = output_tensors[i].GetTensorData<float>();
    outputs.emplace(output_names.at(i), std::vector<float>(data, data + element_count));
  }
  return outputs;
}

OnnxruntimeSingleStepInference::OnnxruntimeSingleStepInference(
  const std::string & model_path, const std::string & execution_provider,
  const std::string & plugins_path, const int, const ModelInputType input_type)
: input_type_(input_type),
  model_(model_path, parse_execution_provider(execution_provider), plugins_path)
{
}

InferenceResult OnnxruntimeSingleStepInference::infer(
  const preprocess::InputDataMap & input_data_map, const std::vector<uint8_t> & bev_image)
{
  auto start = std::chrono::steady_clock::now();
  try {
    const auto outputs = model_.run(
      to_float_input_map(single_step_float_inputs(input_data_map, input_type_)),
      encoder_byte_inputs(input_data_map, input_type_, bev_image),
      {"prediction", "turn_indicator_logit"});

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;

    InferenceOutput output;
    output.outputs = std::make_pair(outputs.at("prediction"), outputs.at("turn_indicator_logit"));
    output.inference_time_ms = elapsed.count();
    output.is_denormalized = true;
    return output;
  } catch (const std::exception & e) {
    return tl::unexpected(std::string{e.what()});
  }
}

OnnxruntimeMultiStepInference::OnnxruntimeMultiStepInference(
  const std::string & encoder_model_path, const std::string & decoder_model_path,
  const std::string & turn_indicator_model_path, const std::string & execution_provider,
  const std::string & plugins_path, const int batch_size, const int dpm_solver_steps,
  std::unordered_map<std::string, std::shared_ptr<Guidance>> guidances,
  const ModelInputType input_type)
: batch_size_(batch_size),
  input_type_(input_type),
  dpm_solver_steps_(dpm_solver_steps),
  guidances_(std::move(guidances)),
  encoder_model_(encoder_model_path, parse_execution_provider(execution_provider), plugins_path),
  decoder_model_(decoder_model_path, parse_execution_provider(execution_provider), plugins_path),
  turn_indicator_model_(
    turn_indicator_model_path, parse_execution_provider(execution_provider), plugins_path)
{
}

std::vector<float> OnnxruntimeMultiStepInference::create_diffusion_time(float t) const
{
  return std::vector<float>(batch_size_ * MAX_NUM_AGENTS * (OUTPUT_T + 1), t);
}

std::vector<float> OnnxruntimeMultiStepInference::create_current_states(
  const preprocess::InputDataMap & input_data_map) const
{
  const auto & ego_current_state = input_data_map.at("ego_current_state");
  const auto & neighbor_agents_past = input_data_map.at("neighbor_agents_past");

  std::vector<float> current_states(batch_size_ * MAX_NUM_AGENTS * POSE_DIM, 0.0f);
  for (int b = 0; b < batch_size_; ++b) {
    for (int64_t d = 0; d < POSE_DIM; ++d) {
      current_states[(b * MAX_NUM_AGENTS * POSE_DIM) + d] =
        ego_current_state[b * EGO_CURRENT_STATE_SHAPE[1] + d];
    }
    for (int64_t agent = 1; agent < MAX_NUM_AGENTS; ++agent) {
      for (int64_t d = 0; d < POSE_DIM; ++d) {
        const size_t neighbor_idx =
          (((static_cast<size_t>(b) * MAX_NUM_NEIGHBORS + (agent - 1)) * (INPUT_T + 1) + INPUT_T) *
           11) +
          d;
        const size_t current_idx = (static_cast<size_t>(b) * MAX_NUM_AGENTS + agent) * POSE_DIM + d;
        current_states[current_idx] = neighbor_agents_past[neighbor_idx];
      }
    }
  }
  return current_states;
}

void OnnxruntimeMultiStepInference::apply_prefix_constraint(
  std::vector<float> & x, const std::vector<float> & current_states) const
{
  for (int b = 0; b < batch_size_; ++b) {
    for (int64_t agent = 0; agent < MAX_NUM_AGENTS; ++agent) {
      for (int64_t d = 0; d < POSE_DIM; ++d) {
        const size_t x_idx =
          ((static_cast<size_t>(b) * MAX_NUM_AGENTS + agent) * (OUTPUT_T + 1)) * POSE_DIM + d;
        const size_t current_idx = (static_cast<size_t>(b) * MAX_NUM_AGENTS + agent) * POSE_DIM + d;
        x[x_idx] = current_states[current_idx];
      }
    }
  }
}

std::vector<float> OnnxruntimeMultiStepInference::evaluate_decoder(
  const std::vector<float> & x, const float t)
{
  const auto diffusion_time = create_diffusion_time(t);
  const auto outputs = decoder_model_.run(
    {{"encoding", encoding_},
     {"sampled_trajectories", x},
     {"diffusion_time", diffusion_time},
     {"neighbor_agents_past", decoder_neighbor_agents_past_}},
    {}, {"model_output"});
  return outputs.at("model_output");
}

DpmSolver::SampleResult OnnxruntimeMultiStepInference::run_dpm_solver(
  const preprocess::InputDataMap & input_data_map)
{
  const std::vector<float> current_states = create_current_states(input_data_map);
  decoder_neighbor_agents_past_ = input_data_map.at("neighbor_agents_past");
  const auto model_fn = [this](const std::vector<float> & x, float timestep) {
    return evaluate_decoder(x, timestep);
  };
  const auto correcting_fn = [this, &current_states](std::vector<float> & x) {
    apply_prefix_constraint(x, current_states);
  };

  const DpmSolver solver(dpm_solver_steps_);
  return solver.sample(
    input_data_map.at("sampled_trajectories"), model_fn, correcting_fn, guidances_);
}

InferenceResult OnnxruntimeMultiStepInference::infer(
  const preprocess::InputDataMap & input_data_map, const std::vector<uint8_t> & bev_image)
{
  auto start = std::chrono::steady_clock::now();
  try {
    const auto encoder_outputs = encoder_model_.run(
      to_float_input_map(encoder_float_inputs(input_data_map, input_type_)),
      encoder_byte_inputs(input_data_map, input_type_, bev_image), {"encoding"});
    encoding_ = encoder_outputs.at("encoding");

    auto solver_result = run_dpm_solver(input_data_map);

    const auto turn_indicator_outputs = turn_indicator_model_.run(
      {{"encoding", encoding_}, {"final_x0", solver_result.final_x}}, {}, {"turn_indicator_logit"});

    std::vector<float> denoising_predictions;
    for (const auto & step : solver_result.denoising_steps) {
      denoising_predictions.insert(denoising_predictions.end(), step.begin(), step.end());
    }

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;

    InferenceOutput output;
    output.outputs = std::make_pair(
      std::move(solver_result.final_x), turn_indicator_outputs.at("turn_indicator_logit"));
    output.denoising_predictions = std::move(denoising_predictions);
    output.denoising_timesteps = std::move(solver_result.denoising_timesteps);
    output.inference_time_ms = elapsed.count();
    output.is_denormalized = false;
    output.guidance_triggered = std::move(solver_result.guidance_triggered);
    return output;
  } catch (const std::exception & e) {
    return tl::unexpected(std::string{e.what()});
  }
}

}  // namespace autoware::diffusion_planner
