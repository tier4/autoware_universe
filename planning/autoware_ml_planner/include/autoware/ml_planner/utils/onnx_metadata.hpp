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

#ifndef AUTOWARE__ML_PLANNER__UTILS__ONNX_METADATA_HPP_
#define AUTOWARE__ML_PLANNER__UTILS__ONNX_METADATA_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace autoware::ml_planner::utils
{

/// Semantic version packed into ModelProto.model_version as
/// major * 10^12 + minor * 10^6 + patch (minor and patch < 10^6).
struct ModelVersion
{
  int64_t major{0};
  int64_t minor{0};
  int64_t patch{0};

  static ModelVersion decode(int64_t encoded);
  std::string to_string() const;
};

/// Top-level ModelProto fields of an ONNX file (graph is not parsed).
struct OnnxModelMetadata
{
  std::string producer_name;
  std::string producer_version;
  std::string domain;
  int64_t model_version{0};
  std::string doc_string;
  std::map<std::string, std::string> metadata_props;

  std::optional<std::string> get(const std::string & key) const;
  ModelVersion version() const { return ModelVersion::decode(model_version); }
};

/**
 * @brief Read ONNX model metadata without depending on onnx/protobuf/onnxruntime.
 *
 * Parses only the top-level ModelProto wire format and seeks past the graph,
 * so it is cheap even for large models.
 *
 * @throws std::runtime_error if the file cannot be opened or is malformed.
 */
OnnxModelMetadata read_onnx_metadata(const std::string & model_path);

}  // namespace autoware::ml_planner::utils

#endif  // AUTOWARE__ML_PLANNER__UTILS__ONNX_METADATA_HPP_
