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

#ifndef AUTOWARE__DIFFUSION_PLANNER__INFERENCE__ONNXRUNTIME_INFERENCE_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__INFERENCE__ONNXRUNTIME_INFERENCE_HPP_

#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/inference/guidance/guidance.hpp"
#include "autoware/diffusion_planner/inference/inference.hpp"
#include "autoware/diffusion_planner/inference/solver/dpm_solver.hpp"
#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"

#include <onnxruntime_cxx_api.h>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::diffusion_planner
{

enum class OnnxruntimeExecutionProvider { CPU, CUDA, TensorRT };

OnnxruntimeExecutionProvider parse_execution_provider(const std::string & execution_provider);

class OrtModel
{
public:
  OrtModel(
    const std::string & model_path, OnnxruntimeExecutionProvider execution_provider,
    const std::string & plugins_path);

  /**
   * @brief Feed the named inputs and read the named outputs.
   *
   * Shapes and element types come from the graph's own declarations, so a caller only supplies
   * the payload. Every declared dimension except the leading batch is fixed; the batch is
   * recovered from the payload size. Byte payloads cover both the BOOL and the UINT8 inputs -
   * which of the two a name is, is again what the graph declares.
   */
  std::unordered_map<std::string, std::vector<float>> run(
    const std::unordered_map<std::string, std::vector<float>> & float_inputs,
    const std::unordered_map<std::string, std::vector<uint8_t>> & byte_inputs,
    const std::vector<std::string> & output_names);

private:
  struct InputSpec
  {
    std::vector<int64_t> shape;  // as declared, with the batch dimension left as -1
    ONNXTensorElementDataType element_type;
  };

  const InputSpec & input_spec(const std::string & name) const;
  std::vector<int64_t> shape_for(const std::string & name, size_t element_count) const;

  Ort::Env env_;
  Ort::SessionOptions session_options_;
  Ort::Session session_;
  Ort::MemoryInfo memory_info_;
  std::unordered_map<std::string, InputSpec> input_specs_;
};

class OnnxruntimeSingleStepInference : public Inference
{
public:
  OnnxruntimeSingleStepInference(
    const std::string & model_path, const std::string & execution_provider,
    const std::string & plugins_path, int batch_size, ModelInputType input_type);

  InferenceResult infer(
    const preprocess::InputDataMap & input_data_map,
    const std::vector<uint8_t> & bev_image) override;

private:
  ModelInputType input_type_{ModelInputType::VECTOR};
  OrtModel model_;
};

class OnnxruntimeMultiStepInference : public Inference
{
public:
  OnnxruntimeMultiStepInference(
    const std::string & encoder_model_path, const std::string & decoder_model_path,
    const std::string & turn_indicator_model_path, const std::string & execution_provider,
    const std::string & plugins_path, int batch_size, int dpm_solver_steps,
    std::unordered_map<std::string, std::shared_ptr<Guidance>> guidances,
    ModelInputType input_type);

  InferenceResult infer(
    const preprocess::InputDataMap & input_data_map,
    const std::vector<uint8_t> & bev_image) override;

private:
  int batch_size_{1};
  ModelInputType input_type_{ModelInputType::VECTOR};
  int dpm_solver_steps_{10};
  std::unordered_map<std::string, std::shared_ptr<Guidance>> guidances_;
  OrtModel encoder_model_;
  OrtModel decoder_model_;
  OrtModel turn_indicator_model_;
  std::vector<float> encoding_;
  std::vector<float> decoder_neighbor_agents_past_;

  DpmSolver::SampleResult run_dpm_solver(const preprocess::InputDataMap & input_data_map);
  std::vector<float> evaluate_decoder(const std::vector<float> & x, float t);
  std::vector<float> create_diffusion_time(float t) const;
  std::vector<float> create_current_states(const preprocess::InputDataMap & input_data_map) const;
  void apply_prefix_constraint(
    std::vector<float> & x, const std::vector<float> & current_states) const;
};

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__INFERENCE__ONNXRUNTIME_INFERENCE_HPP_
