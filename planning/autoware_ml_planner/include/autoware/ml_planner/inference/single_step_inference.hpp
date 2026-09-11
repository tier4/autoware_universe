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

#ifndef AUTOWARE__ML_PLANNER__INFERENCE__SINGLE_STEP_INFERENCE_HPP_
#define AUTOWARE__ML_PLANNER__INFERENCE__SINGLE_STEP_INFERENCE_HPP_

#include "autoware/ml_planner/inference/inference.hpp"
#include "autoware/ml_planner/inference/utils.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>

#include <cuda_runtime_api.h>

#include <memory>
#include <string>

namespace autoware::ml_planner
{

/// Native TensorRT execution of the fixed 20-step flow-matching sampler.
class SingleStepInference : public Inference
{
public:
  SingleStepInference(
    const std::string & model_path, const std::string & plugins_path, int batch_size,
    const std::string & precision = "fp32", bool use_cuda_graph = false);
  ~SingleStepInference() override;

  InferenceResult infer(const preprocess::TensorMap & input_data_map) override;
  void load_engine(const std::string & model_path);

private:
  int batch_size_{1};
  std::string plugins_path_;
  std::string precision_{"fp32"};
  bool use_cuda_graph_{false};
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> network_trt_ptr_{nullptr};
  CudaGraphExecutor network_cuda_graph_;

  autoware::cuda_utils::CudaUniquePtr<float[]> initial_noise_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> ego_history_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> neighbor_agents_past_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> agent_shape_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> agent_label_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> lanes_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> lane_types_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> lanes_speed_limit_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> lane_traffic_light_past_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> lane_traffic_light_future_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> route_lanes_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> route_lane_types_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> route_lanes_speed_limit_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> route_traffic_light_past_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> route_traffic_light_future_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> intersection_area_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> stop_lines_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> road_borders_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> goal_pose_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> ego_shape_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> turn_indicators_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> output_d_;
  autoware::cuda_utils::CudaUniquePtrHost<float[]> output_pinned_;
  size_t output_num_elements_{0};
  autoware::cuda_utils::CudaUniquePtr<float[]> turn_indicator_logits_d_;
  autoware::cuda_utils::CudaUniquePtrHost<float[]> turn_indicator_logits_pinned_;
  size_t turn_indicator_logits_num_elements_{0};
  cudaStream_t stream_{nullptr};

  void bind_buffers();
  void transfer_inputs_to_device(const preprocess::TensorMap & input_data_map);
};

}  // namespace autoware::ml_planner

#endif  // AUTOWARE__ML_PLANNER__INFERENCE__SINGLE_STEP_INFERENCE_HPP_
