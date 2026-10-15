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

#ifndef AUTOWARE__ML_PLANNER__UTILS__MODEL_METADATA_HPP_
#define AUTOWARE__ML_PLANNER__UTILS__MODEL_METADATA_HPP_

#include <nlohmann/json.hpp>

#include <string>

namespace autoware::ml_planner::utils
{

/// File name of the metadata placed next to the ONNX model.
constexpr const char * MODEL_METADATA_FILENAME = "metadata.json";

/**
 * @brief Path of metadata.json in the same directory as the ONNX model.
 */
std::string model_metadata_path(const std::string & model_path);

/**
 * @brief Load metadata.json.
 * @throws std::runtime_error if the file cannot be opened or parsed.
 */
nlohmann::json load_model_metadata(const std::string & metadata_path);

/**
 * @brief Compute the SHA-256 of a file as a lowercase hex string.
 * @throws std::runtime_error if the file cannot be read.
 */
std::string compute_file_sha256_hex(const std::string & path);

/**
 * @brief Check that metadata["onnx_sha256"] matches the SHA-256 of the model file.
 * @return SHA-256 of the model file (lowercase hex).
 * @throws std::runtime_error if the field is missing or does not match.
 */
std::string verify_model_sha256(const nlohmann::json & metadata, const std::string & model_path);

/**
 * @brief Check that metadata["major_version"] equals the expected major version.
 * @throws std::runtime_error if the field is missing or does not match.
 */
void verify_model_major_version(const nlohmann::json & metadata, int expected_major_version);

}  // namespace autoware::ml_planner::utils

#endif  // AUTOWARE__ML_PLANNER__UTILS__MODEL_METADATA_HPP_
