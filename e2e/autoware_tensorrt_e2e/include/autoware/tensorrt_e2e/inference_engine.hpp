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

#ifndef AUTOWARE__TENSORRT_E2E__INFERENCE_ENGINE_HPP_
#define AUTOWARE__TENSORRT_E2E__INFERENCE_ENGINE_HPP_

#include "autoware/tensorrt_e2e/types.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>

#include <cuda_runtime_api.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

/**
 * @class InferenceEngine
 * @brief Generic TensorRT execution over named tensors.
 *
 * Unlike model-specific wrappers, no tensor identity is known at compile time: the engine's IO
 * manifest is introspected at load time and exposed through `input_specs()` / `output_specs()`.
 * A dynamic batch dimension (-1 as dim 0) is resolved to 1; any other dynamic dimension is
 * rejected. Device buffers and pinned host output buffers are allocated and bound once.
 */
class InferenceEngine
{
public:
  struct Config
  {
    std::string model_path;    //!< ONNX model path; the built engine is cached alongside it.
    std::string plugins_path;  //!< Optional TensorRT plugin library path ("" to disable).
    std::string precision{"fp16"};
    //! TensorRT builder workspace. Too small a pool does not merely cost
    //! performance on these graphs: building the ResWorld planner with 1 GiB
    //! segfaults inside the builder, while 4 GiB builds cleanly. Configurable
    //! (`trt_workspace_mib`) because it is a property of the deployment host,
    //! not of the model.
    size_t max_workspace_size{4ULL << 30U};
  };

  struct Result
  {
    std::optional<TensorMap> outputs;
    std::string error_msg;
  };

  /**
   * @brief Build/load the engine and introspect its IO manifest.
   * @throws std::runtime_error on engine setup failure or unsupported IO tensors.
   */
  explicit InferenceEngine(const Config & config);
  ~InferenceEngine();

  InferenceEngine(const InferenceEngine &) = delete;
  InferenceEngine & operator=(const InferenceEngine &) = delete;

  const std::vector<TensorSpec> & input_specs() const { return input_specs_; }
  const std::vector<TensorSpec> & output_specs() const { return output_specs_; }

  /**
   * @brief Run inference. Every input spec must be present in `inputs` with a matching element
   * count; extra entries in `inputs` are ignored. Outputs are returned as host tensors.
   */
  Result infer(const TensorMap & inputs);

private:
  struct Binding
  {
    TensorSpec spec;
    size_t byte_size{0};
    autoware::cuda_utils::CudaUniquePtr<uint8_t[]> device;
    // Outputs only: pinned host buffer for fast async D2H.
    autoware::cuda_utils::CudaUniquePtrHost<float[]> pinned;
    // Bool/int32 inputs only: staging buffer for host-side dtype conversion.
    std::vector<uint8_t> staging;
  };

  void load_engine(const Config & config);
  void introspect_and_bind();
  /// Copy one input tensor to its device buffer (H2D with dtype conversion, or D2D).
  /// Returns an error message on failure, empty string on success.
  std::string transfer_input(Binding & binding, const Tensor & tensor);

  std::unique_ptr<autoware::tensorrt_common::TrtCommon> trt_common_;
  std::vector<TensorSpec> input_specs_;
  std::vector<TensorSpec> output_specs_;
  std::vector<Binding> input_bindings_;
  std::vector<Binding> output_bindings_;
  cudaStream_t stream_{nullptr};
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__INFERENCE_ENGINE_HPP_
