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

#include "autoware/diffusion_planner/inference/single_step_inference.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/inference/utils.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{
using autoware::tensorrt_common::ProfileDims;

SingleStepInference::SingleStepInference(
  const std::string & model_path, const std::string & plugins_path, int batch_size,
  const std::string & precision, bool use_cuda_graph, ModelInputType input_type)
: batch_size_(batch_size),
  input_type_(input_type),
  plugins_path_(plugins_path),
  precision_(precision),
  use_cuda_graph_(use_cuda_graph)
{
  const size_t sampled_trajectories_size =
    batch_size_ * num_elements_without_batch(SAMPLED_TRAJECTORIES_SHAPE);
  const size_t ego_history_size = batch_size_ * num_elements_without_batch(EGO_HISTORY_SHAPE);
  const size_t ego_current_state_size =
    batch_size_ * num_elements_without_batch(EGO_CURRENT_STATE_SHAPE);
  const size_t neighbor_agents_past_size = batch_size_ * num_elements_without_batch(NEIGHBOR_SHAPE);
  const size_t static_objects_size = batch_size_ * num_elements_without_batch(STATIC_OBJECTS_SHAPE);
  const size_t lanes_size = batch_size_ * num_elements_without_batch(LANES_SHAPE);
  const size_t lanes_has_speed_limit_size =
    batch_size_ * num_elements_without_batch(LANES_HAS_SPEED_LIMIT_SHAPE);
  const size_t lanes_speed_limit_size =
    batch_size_ * num_elements_without_batch(LANES_SPEED_LIMIT_SHAPE);
  const size_t route_lanes_size = batch_size_ * num_elements_without_batch(ROUTE_LANES_SHAPE);
  const size_t route_lanes_has_speed_limit_size =
    batch_size_ * num_elements_without_batch(ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE);
  const size_t route_lanes_speed_limit_size =
    batch_size_ * num_elements_without_batch(ROUTE_LANES_SPEED_LIMIT_SHAPE);
  const size_t polygons_size = batch_size_ * num_elements_without_batch(POLYGONS_SHAPE);
  const size_t line_strings_size = batch_size_ * num_elements_without_batch(LINE_STRINGS_SHAPE);
  const size_t goal_pose_size = batch_size_ * num_elements_without_batch(GOAL_POSE_SHAPE);
  const size_t ego_shape_size = batch_size_ * num_elements_without_batch(EGO_SHAPE_SHAPE);
  const size_t turn_indicators_size =
    batch_size_ * num_elements_without_batch(TURN_INDICATORS_SHAPE);
  const size_t delay_size = batch_size_ * num_elements_without_batch(DELAY_SHAPE);
  const size_t output_size = batch_size_ * num_elements_without_batch(OUTPUT_SHAPE);
  const size_t turn_indicator_logit_size =
    batch_size_ * num_elements_without_batch(TURN_INDICATOR_LOGIT_SHAPE);

  sampled_trajectories_d_ = autoware::cuda_utils::make_unique<float[]>(sampled_trajectories_size);
  ego_history_d_ = autoware::cuda_utils::make_unique<float[]>(ego_history_size);
  ego_current_state_d_ = autoware::cuda_utils::make_unique<float[]>(ego_current_state_size);
  neighbor_agents_past_d_ = autoware::cuda_utils::make_unique<float[]>(neighbor_agents_past_size);
  static_objects_d_ = autoware::cuda_utils::make_unique<float[]>(static_objects_size);
  lanes_d_ = autoware::cuda_utils::make_unique<float[]>(lanes_size);
  lanes_has_speed_limit_d_ = autoware::cuda_utils::make_unique<bool[]>(lanes_has_speed_limit_size);
  lanes_speed_limit_d_ = autoware::cuda_utils::make_unique<float[]>(lanes_speed_limit_size);
  route_lanes_d_ = autoware::cuda_utils::make_unique<float[]>(route_lanes_size);
  route_lanes_has_speed_limit_d_ =
    autoware::cuda_utils::make_unique<bool[]>(route_lanes_has_speed_limit_size);
  route_lanes_speed_limit_d_ =
    autoware::cuda_utils::make_unique<float[]>(route_lanes_speed_limit_size);
  polygons_d_ = autoware::cuda_utils::make_unique<float[]>(polygons_size);
  line_strings_d_ = autoware::cuda_utils::make_unique<float[]>(line_strings_size);
  goal_pose_d_ = autoware::cuda_utils::make_unique<float[]>(goal_pose_size);
  ego_shape_d_ = autoware::cuda_utils::make_unique<float[]>(ego_shape_size);
  turn_indicators_d_ = autoware::cuda_utils::make_unique<float[]>(turn_indicators_size);
  delay_d_ = autoware::cuda_utils::make_unique<float[]>(delay_size);
  // Every buffer above is allocated in both modes - together they are a few MB - so only the
  // engine bindings below actually differ per input type.
  bev_image_d_ = autoware::cuda_utils::make_unique<uint8_t[]>(
    batch_size_ * num_elements_without_batch(BEV_IMAGE_SHAPE));

  output_d_ = autoware::cuda_utils::make_unique<float[]>(output_size);
  turn_indicator_logit_d_ = autoware::cuda_utils::make_unique<float[]>(turn_indicator_logit_size);

  // Pre-allocate pinned host buffers for fast async D2H transfers
  output_num_elements_ = output_size;
  logit_num_elements_ = turn_indicator_logit_size;
  output_pinned_ =
    autoware::cuda_utils::make_unique_host<float[]>(output_num_elements_, cudaHostAllocDefault);
  logit_pinned_ =
    autoware::cuda_utils::make_unique_host<float[]>(logit_num_elements_, cudaHostAllocDefault);

  load_engine(model_path);
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
}

SingleStepInference::~SingleStepInference()
{
  if (stream_) {
    cudaStreamDestroy(stream_);
  }
}

void SingleStepInference::load_engine(const std::string & model_path)
{
  std::vector<ProfileDims> profile_dims;
  std::vector<autoware::tensorrt_common::NetworkIO> network_io;

  const auto add_input_tensor = [&](const std::string & name, const auto & shape) {
    const auto dims = to_dynamic_dims(shape, batch_size_);
    profile_dims.emplace_back(make_profile_dims(name, dims, batch_size_));
    network_io.emplace_back(name, dims);
  };

  add_input_tensor("sampled_trajectories", SAMPLED_TRAJECTORIES_SHAPE);
  add_input_tensor("neighbor_agents_past", NEIGHBOR_SHAPE);
  add_input_tensor("ego_current_state", EGO_CURRENT_STATE_SHAPE);
  add_input_tensor("turn_indicators", TURN_INDICATORS_SHAPE);
  add_input_tensor("delay", DELAY_SHAPE);
  if (input_type_ == ModelInputType::IMAGE) {
    // The raster replaces every drawable element; ego_agent_past, goal_pose and ego_shape are
    // gone with them because the image encoder reads none of them (see image_encoder.py).
    add_input_tensor("bev_image", BEV_IMAGE_SHAPE);
  } else {
    add_input_tensor("ego_agent_past", EGO_HISTORY_SHAPE);
    add_input_tensor("static_objects", STATIC_OBJECTS_SHAPE);
    add_input_tensor("lanes", LANES_SHAPE);
    add_input_tensor("lanes_has_speed_limit", LANES_HAS_SPEED_LIMIT_SHAPE);
    add_input_tensor("lanes_speed_limit", LANES_SPEED_LIMIT_SHAPE);
    add_input_tensor("route_lanes", ROUTE_LANES_SHAPE);
    add_input_tensor("polygons", POLYGONS_SHAPE);
    add_input_tensor("line_strings", LINE_STRINGS_SHAPE);
    add_input_tensor("route_lanes_has_speed_limit", ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE);
    add_input_tensor("route_lanes_speed_limit", ROUTE_LANES_SPEED_LIMIT_SHAPE);
    add_input_tensor("goal_pose", GOAL_POSE_SHAPE);
    add_input_tensor("ego_shape", EGO_SHAPE_SHAPE);
  }

  network_io.emplace_back("prediction", to_dynamic_dims(OUTPUT_SHAPE, batch_size_));
  network_io.emplace_back(
    "turn_indicator_logit", to_dynamic_dims(TURN_INDICATOR_LOGIT_SHAPE, batch_size_));

  network_trt_ptr_ =
    setup_engine(model_path, plugins_path_, batch_size_, precision_, network_io, profile_dims);

  bindBuffers();
}

void SingleStepInference::bindBuffers()
{
  // Set input shapes and bind addresses once: batch_size is fixed and the GPU buffers are stable.
  const auto bind_float = [this](const char * name, const auto & shape, float * buffer) {
    network_trt_ptr_->setInputShape(name, to_dims_with_batch(shape, batch_size_));
    network_trt_ptr_->setTensorAddress(name, buffer);
  };
  const auto bind_bool = [this](const char * name, const auto & shape, bool * buffer) {
    network_trt_ptr_->setInputShape(name, to_dims_with_batch(shape, batch_size_));
    network_trt_ptr_->setTensorAddress(name, buffer);
  };

  bind_float("sampled_trajectories", SAMPLED_TRAJECTORIES_SHAPE, sampled_trajectories_d_.get());
  bind_float("neighbor_agents_past", NEIGHBOR_SHAPE, neighbor_agents_past_d_.get());
  bind_float("ego_current_state", EGO_CURRENT_STATE_SHAPE, ego_current_state_d_.get());
  bind_float("turn_indicators", TURN_INDICATORS_SHAPE, turn_indicators_d_.get());
  bind_float("delay", DELAY_SHAPE, delay_d_.get());

  if (input_type_ == ModelInputType::IMAGE) {
    network_trt_ptr_->setInputShape("bev_image", to_dims_with_batch(BEV_IMAGE_SHAPE, batch_size_));
    network_trt_ptr_->setTensorAddress("bev_image", bev_image_d_.get());
  } else {
    bind_float("ego_agent_past", EGO_HISTORY_SHAPE, ego_history_d_.get());
    bind_float("static_objects", STATIC_OBJECTS_SHAPE, static_objects_d_.get());
    bind_float("lanes", LANES_SHAPE, lanes_d_.get());
    bind_float("lanes_speed_limit", LANES_SPEED_LIMIT_SHAPE, lanes_speed_limit_d_.get());
    bind_bool("lanes_has_speed_limit", LANES_HAS_SPEED_LIMIT_SHAPE, lanes_has_speed_limit_d_.get());
    bind_float("route_lanes", ROUTE_LANES_SHAPE, route_lanes_d_.get());
    bind_float(
      "route_lanes_speed_limit", ROUTE_LANES_SPEED_LIMIT_SHAPE, route_lanes_speed_limit_d_.get());
    bind_bool(
      "route_lanes_has_speed_limit", ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE,
      route_lanes_has_speed_limit_d_.get());
    bind_float("polygons", POLYGONS_SHAPE, polygons_d_.get());
    bind_float("line_strings", LINE_STRINGS_SHAPE, line_strings_d_.get());
    bind_float("goal_pose", GOAL_POSE_SHAPE, goal_pose_d_.get());
    bind_float("ego_shape", EGO_SHAPE_SHAPE, ego_shape_d_.get());
  }

  network_trt_ptr_->setTensorAddress("prediction", output_d_.get());
  network_trt_ptr_->setTensorAddress("turn_indicator_logit", turn_indicator_logit_d_.get());
}

void SingleStepInference::transferInputsToDevice(
  const preprocess::InputDataMap & input_data_map, const std::vector<uint8_t> & bev_image)
{
  transfer_float_input(input_data_map.at("sampled_trajectories"), sampled_trajectories_d_, stream_);
  transfer_float_input(input_data_map.at("neighbor_agents_past"), neighbor_agents_past_d_, stream_);
  transfer_float_input(input_data_map.at("ego_current_state"), ego_current_state_d_, stream_);
  transfer_float_input(input_data_map.at("turn_indicators"), turn_indicators_d_, stream_);
  transfer_float_input(input_data_map.at("delay"), delay_d_, stream_);

  if (input_type_ == ModelInputType::IMAGE) {
    transfer_uint8_input(bev_image, bev_image_d_, stream_);
    return;
  }

  transfer_float_input(input_data_map.at("ego_agent_past"), ego_history_d_, stream_);
  transfer_float_input(input_data_map.at("static_objects"), static_objects_d_, stream_);
  transfer_float_input(input_data_map.at("lanes"), lanes_d_, stream_);
  transfer_float_input(input_data_map.at("lanes_speed_limit"), lanes_speed_limit_d_, stream_);
  transfer_float_input(input_data_map.at("route_lanes"), route_lanes_d_, stream_);
  transfer_float_input(
    input_data_map.at("route_lanes_speed_limit"), route_lanes_speed_limit_d_, stream_);
  transfer_float_input(input_data_map.at("polygons"), polygons_d_, stream_);
  transfer_float_input(input_data_map.at("line_strings"), line_strings_d_, stream_);
  transfer_float_input(input_data_map.at("goal_pose"), goal_pose_d_, stream_);
  transfer_float_input(input_data_map.at("ego_shape"), ego_shape_d_, stream_);

  transfer_speed_mask(
    input_data_map.at("lanes_speed_limit"), lanes_has_speed_limit_d_,
    batch_size_ * num_elements_without_batch(LANES_SPEED_LIMIT_SHAPE), stream_);
  transfer_speed_mask(
    input_data_map.at("route_lanes_speed_limit"), route_lanes_has_speed_limit_d_,
    batch_size_ * num_elements_without_batch(ROUTE_LANES_SPEED_LIMIT_SHAPE), stream_);
}

SingleStepInference::InferenceResult SingleStepInference::infer(
  const preprocess::InputDataMap & input_data_map, const std::vector<uint8_t> & bev_image)
{
  auto start = std::chrono::steady_clock::now();

  transferInputsToDevice(input_data_map, bev_image);

  const bool status = enqueue_trt(*network_trt_ptr_, network_cuda_graph_, stream_, use_cuda_graph_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  if (!status) {
    return tl::unexpected(std::string{"Failed to enqueue and do inference."});
  }

  // Async D2H via pre-allocated pinned host buffers
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    output_pinned_.get(), output_d_.get(), output_num_elements_ * sizeof(float),
    cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    logit_pinned_.get(), turn_indicator_logit_d_.get(), logit_num_elements_ * sizeof(float),
    cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  std::vector<float> output_host(output_pinned_.get(), output_pinned_.get() + output_num_elements_);
  std::vector<float> logit_host(logit_pinned_.get(), logit_pinned_.get() + logit_num_elements_);

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double, std::milli> elapsed = end - start;

  InferenceOutput output;
  output.outputs = std::make_pair(std::move(output_host), std::move(logit_host));
  output.inference_time_ms = elapsed.count();
  output.is_denormalized = true;
  return output;
}

}  // namespace autoware::diffusion_planner
