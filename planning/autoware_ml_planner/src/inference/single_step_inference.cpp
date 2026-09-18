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

#include "autoware/ml_planner/inference/single_step_inference.hpp"

#include "autoware/ml_planner/dimensions.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <chrono>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ml_planner
{
using autoware::tensorrt_common::ProfileDims;

SingleStepInference::SingleStepInference(
  const std::string & model_path, const std::string & plugins_path, const int batch_size,
  const std::string & precision, const bool use_cuda_graph)
: batch_size_(batch_size),
  plugins_path_(plugins_path),
  precision_(precision),
  use_cuda_graph_(use_cuda_graph)
{
  const auto allocate = [this](const auto & shape) {
    return static_cast<size_t>(batch_size_) * num_elements_without_batch(shape);
  };
  initial_noise_d_ = autoware::cuda_utils::make_unique<float[]>(allocate(INITIAL_NOISE_SHAPE));
  ego_history_d_ = autoware::cuda_utils::make_unique<float[]>(allocate(EGO_HISTORY_SHAPE));
  neighbor_agents_past_d_ = autoware::cuda_utils::make_unique<float[]>(allocate(NEIGHBOR_SHAPE));
  agent_shape_d_ = autoware::cuda_utils::make_unique<float[]>(allocate(AGENT_SHAPE_SHAPE));
  agent_label_d_ = autoware::cuda_utils::make_unique<float[]>(allocate(AGENT_LABEL_SHAPE));
  lanes_d_ = autoware::cuda_utils::make_unique<float[]>(allocate(LANES_SHAPE));
  lane_types_d_ = autoware::cuda_utils::make_unique<float[]>(allocate(LANE_TYPES_SHAPE));
  lanes_speed_limit_d_ =
    autoware::cuda_utils::make_unique<float[]>(allocate(LANES_SPEED_LIMIT_SHAPE));
  lane_traffic_light_past_d_ =
    autoware::cuda_utils::make_unique<float[]>(allocate(LANE_TRAFFIC_LIGHT_PAST_SHAPE));
  lane_traffic_light_future_d_ =
    autoware::cuda_utils::make_unique<float[]>(allocate(LANE_TRAFFIC_LIGHT_FUTURE_SHAPE));
  route_lanes_d_ = autoware::cuda_utils::make_unique<float[]>(allocate(ROUTE_LANES_SHAPE));
  route_lane_types_d_ =
    autoware::cuda_utils::make_unique<float[]>(allocate(ROUTE_LANE_TYPES_SHAPE));
  route_lanes_speed_limit_d_ =
    autoware::cuda_utils::make_unique<float[]>(allocate(ROUTE_LANES_SPEED_LIMIT_SHAPE));
  route_traffic_light_past_d_ =
    autoware::cuda_utils::make_unique<float[]>(allocate(ROUTE_TRAFFIC_LIGHT_PAST_SHAPE));
  route_traffic_light_future_d_ =
    autoware::cuda_utils::make_unique<float[]>(allocate(ROUTE_TRAFFIC_LIGHT_FUTURE_SHAPE));
  intersection_area_d_ =
    autoware::cuda_utils::make_unique<float[]>(allocate(INTERSECTION_AREA_SHAPE));
  stop_lines_d_ = autoware::cuda_utils::make_unique<float[]>(allocate(STOP_LINES_SHAPE));
  road_borders_d_ = autoware::cuda_utils::make_unique<float[]>(allocate(ROAD_BORDERS_SHAPE));
  goal_pose_d_ = autoware::cuda_utils::make_unique<float[]>(allocate(GOAL_POSE_SHAPE));
  ego_shape_d_ = autoware::cuda_utils::make_unique<float[]>(allocate(EGO_SHAPE_SHAPE));
  turn_indicators_d_ = autoware::cuda_utils::make_unique<float[]>(allocate(TURN_INDICATORS_SHAPE));
  output_num_elements_ = allocate(OUTPUT_SHAPE);
  output_d_ = autoware::cuda_utils::make_unique<float[]>(output_num_elements_);
  output_pinned_ =
    autoware::cuda_utils::make_unique_host<float[]>(output_num_elements_, cudaHostAllocDefault);
  turn_indicator_logits_num_elements_ = allocate(TURN_INDICATOR_LOGIT_SHAPE);
  turn_indicator_logits_d_ =
    autoware::cuda_utils::make_unique<float[]>(turn_indicator_logits_num_elements_);
  turn_indicator_logits_pinned_ = autoware::cuda_utils::make_unique_host<float[]>(
    turn_indicator_logits_num_elements_, cudaHostAllocDefault);

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
  const auto add_input = [&](const std::string & name, const auto & shape) {
    const auto dims = to_dynamic_dims(shape, batch_size_);
    profile_dims.emplace_back(make_profile_dims(name, dims, batch_size_));
    network_io.emplace_back(name, dims);
  };

  add_input("initial_noise", INITIAL_NOISE_SHAPE);
  add_input("ego_agent_past", EGO_HISTORY_SHAPE);
  add_input("neighbor_agents_past", NEIGHBOR_SHAPE);
  add_input("agent_shape", AGENT_SHAPE_SHAPE);
  add_input("agent_label", AGENT_LABEL_SHAPE);
  add_input("lanes", LANES_SHAPE);
  add_input("lane_types", LANE_TYPES_SHAPE);
  add_input("lanes_speed_limit", LANES_SPEED_LIMIT_SHAPE);
  add_input("lane_traffic_light_past", LANE_TRAFFIC_LIGHT_PAST_SHAPE);
  add_input("lane_traffic_light_future", LANE_TRAFFIC_LIGHT_FUTURE_SHAPE);
  add_input("route_lanes", ROUTE_LANES_SHAPE);
  add_input("route_lane_types", ROUTE_LANE_TYPES_SHAPE);
  add_input("route_lanes_speed_limit", ROUTE_LANES_SPEED_LIMIT_SHAPE);
  add_input("route_traffic_light_past", ROUTE_TRAFFIC_LIGHT_PAST_SHAPE);
  add_input("route_traffic_light_future", ROUTE_TRAFFIC_LIGHT_FUTURE_SHAPE);
  add_input("intersection_area", INTERSECTION_AREA_SHAPE);
  add_input("stop_lines", STOP_LINES_SHAPE);
  add_input("road_borders", ROAD_BORDERS_SHAPE);
  add_input("goal_pose", GOAL_POSE_SHAPE);
  add_input("ego_shape", EGO_SHAPE_SHAPE);
  add_input("turn_indicators", TURN_INDICATORS_SHAPE);
  network_io.emplace_back("trajectory", to_dynamic_dims(OUTPUT_SHAPE, batch_size_));
  network_io.emplace_back(
    "turn_indicator_logits", to_dynamic_dims(TURN_INDICATOR_LOGIT_SHAPE, batch_size_));
  network_trt_ptr_ =
    setup_engine(model_path, plugins_path_, batch_size_, precision_, network_io, profile_dims);
  bind_buffers();
}

void SingleStepInference::bind_buffers()
{
  const auto bind_input = [&](const std::string & name, const auto & shape, auto * data) {
    network_trt_ptr_->setInputShape(name.c_str(), to_dims_with_batch(shape, batch_size_));
    network_trt_ptr_->setTensorAddress(name.c_str(), data);
  };
  bind_input("initial_noise", INITIAL_NOISE_SHAPE, initial_noise_d_.get());
  bind_input("ego_agent_past", EGO_HISTORY_SHAPE, ego_history_d_.get());
  bind_input("neighbor_agents_past", NEIGHBOR_SHAPE, neighbor_agents_past_d_.get());
  bind_input("agent_shape", AGENT_SHAPE_SHAPE, agent_shape_d_.get());
  bind_input("agent_label", AGENT_LABEL_SHAPE, agent_label_d_.get());
  bind_input("lanes", LANES_SHAPE, lanes_d_.get());
  bind_input("lane_types", LANE_TYPES_SHAPE, lane_types_d_.get());
  bind_input("lanes_speed_limit", LANES_SPEED_LIMIT_SHAPE, lanes_speed_limit_d_.get());
  bind_input(
    "lane_traffic_light_past", LANE_TRAFFIC_LIGHT_PAST_SHAPE, lane_traffic_light_past_d_.get());
  bind_input(
    "lane_traffic_light_future", LANE_TRAFFIC_LIGHT_FUTURE_SHAPE,
    lane_traffic_light_future_d_.get());
  bind_input("route_lanes", ROUTE_LANES_SHAPE, route_lanes_d_.get());
  bind_input("route_lane_types", ROUTE_LANE_TYPES_SHAPE, route_lane_types_d_.get());
  bind_input(
    "route_lanes_speed_limit", ROUTE_LANES_SPEED_LIMIT_SHAPE, route_lanes_speed_limit_d_.get());
  bind_input(
    "route_traffic_light_past", ROUTE_TRAFFIC_LIGHT_PAST_SHAPE, route_traffic_light_past_d_.get());
  bind_input(
    "route_traffic_light_future", ROUTE_TRAFFIC_LIGHT_FUTURE_SHAPE,
    route_traffic_light_future_d_.get());
  bind_input("intersection_area", INTERSECTION_AREA_SHAPE, intersection_area_d_.get());
  bind_input("stop_lines", STOP_LINES_SHAPE, stop_lines_d_.get());
  bind_input("road_borders", ROAD_BORDERS_SHAPE, road_borders_d_.get());
  bind_input("goal_pose", GOAL_POSE_SHAPE, goal_pose_d_.get());
  bind_input("ego_shape", EGO_SHAPE_SHAPE, ego_shape_d_.get());
  bind_input("turn_indicators", TURN_INDICATORS_SHAPE, turn_indicators_d_.get());
  network_trt_ptr_->setTensorAddress("trajectory", output_d_.get());
  network_trt_ptr_->setTensorAddress("turn_indicator_logits", turn_indicator_logits_d_.get());
}

void SingleStepInference::transfer_inputs_to_device(const preprocess::TensorMap & input_data_map)
{
  const auto transfer = [&](const std::string & name, auto & destination) {
    transfer_float_input(input_data_map.at(name), destination, stream_);
  };
  transfer("initial_noise", initial_noise_d_);
  transfer("ego_agent_past", ego_history_d_);
  transfer("neighbor_agents_past", neighbor_agents_past_d_);
  transfer("agent_shape", agent_shape_d_);
  transfer("agent_label", agent_label_d_);
  transfer("lanes", lanes_d_);
  transfer("lane_types", lane_types_d_);
  transfer("lanes_speed_limit", lanes_speed_limit_d_);
  transfer("lane_traffic_light_past", lane_traffic_light_past_d_);
  transfer("lane_traffic_light_future", lane_traffic_light_future_d_);
  transfer("route_lanes", route_lanes_d_);
  transfer("route_lane_types", route_lane_types_d_);
  transfer("route_lanes_speed_limit", route_lanes_speed_limit_d_);
  transfer("route_traffic_light_past", route_traffic_light_past_d_);
  transfer("route_traffic_light_future", route_traffic_light_future_d_);
  transfer("intersection_area", intersection_area_d_);
  transfer("stop_lines", stop_lines_d_);
  transfer("road_borders", road_borders_d_);
  transfer("goal_pose", goal_pose_d_);
  transfer("ego_shape", ego_shape_d_);
  transfer("turn_indicators", turn_indicators_d_);
}

InferenceResult SingleStepInference::infer(const preprocess::TensorMap & input_data_map)
{
  const auto start = std::chrono::steady_clock::now();
  transfer_inputs_to_device(input_data_map);
  const bool status = enqueue_trt(*network_trt_ptr_, network_cuda_graph_, stream_, use_cuda_graph_);
  if (!status) {
    return tl::unexpected(std::string{"Failed to execute TensorRT inference"});
  }
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    output_pinned_.get(), output_d_.get(), output_num_elements_ * sizeof(float),
    cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    turn_indicator_logits_pinned_.get(), turn_indicator_logits_d_.get(),
    turn_indicator_logits_num_elements_ * sizeof(float), cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  InferenceOutput output;
  output.trajectory.assign(output_pinned_.get(), output_pinned_.get() + output_num_elements_);
  output.turn_indicator_logits.assign(
    turn_indicator_logits_pinned_.get(),
    turn_indicator_logits_pinned_.get() + turn_indicator_logits_num_elements_);
  output.inference_time_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  output.is_denormalized = false;
  return output;
}

}  // namespace autoware::ml_planner
