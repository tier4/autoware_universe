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

#ifndef AUTOWARE__ML_PLANNER__INFERENCE__ONNXRUNTIME_INFERENCE_HPP_
#define AUTOWARE__ML_PLANNER__INFERENCE__ONNXRUNTIME_INFERENCE_HPP_

#include "autoware/ml_planner/inference/inference.hpp"

#include <onnxruntime_cxx_api.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::ml_planner
{

enum class OnnxruntimeExecutionProvider { CPU, CUDA, TensorRT };
OnnxruntimeExecutionProvider parse_execution_provider(const std::string & execution_provider);

class OrtModel
{
public:
  OrtModel(
    const std::string & model_path, OnnxruntimeExecutionProvider execution_provider,
    const std::string & plugins_path = "");

  std::unordered_map<std::string, std::vector<float>> run(
    const std::unordered_map<std::string, std::vector<float>> & inputs,
    const std::vector<std::string> & output_names);

private:
  Ort::Env env_;
  Ort::SessionOptions session_options_;
  Ort::Session session_;
  Ort::MemoryInfo memory_info_;
};

class OnnxruntimeSingleStepInference : public Inference
{
public:
  OnnxruntimeSingleStepInference(
    const std::string & model_path, const std::string & execution_provider,
    const std::string & plugins_path);

  InferenceResult infer(const preprocess::TensorMap & input_data_map) override;

private:
  OrtModel model_;
};

}  // namespace autoware::ml_planner

#endif  // AUTOWARE__ML_PLANNER__INFERENCE__ONNXRUNTIME_INFERENCE_HPP_
