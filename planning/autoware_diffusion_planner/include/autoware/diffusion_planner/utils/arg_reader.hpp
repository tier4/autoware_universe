// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__DIFFUSION_PLANNER__UTILS__ARG_READER_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__UTILS__ARG_READER_HPP_

#include "autoware/diffusion_planner/constants.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
namespace autoware::diffusion_planner::utils
{
using json = nlohmann::json;

// Define normalization structure: {name -> (mean, std)}
using ObservationNormalization =
  std::unordered_map<std::string, std::pair<std::vector<float>, std::vector<float>>>;

using StateNormalization = std::pair<std::vector<float>, std::vector<float>>;

inline std::vector<float> flatten_json_floats(const json & value)
{
  std::vector<float> result;
  const auto visit = [&result](const json & node, const auto & self) -> void {
    if (node.is_array()) {
      for (const auto & item : node) {
        self(item, self);
      }
      return;
    }
    result.push_back(node.get<float>());
  };
  visit(value, visit);
  return result;
}

inline void check_weight_version(const std::string & json_path)
{
  std::ifstream file(json_path);
  if (!file) {
    throw std::runtime_error("Could not open JSON file: " + json_path);
  }

  json j;
  file >> j;

  const std::string error_msg =
    "Please use the appropriate version of diffusion_planner.onnx and "
    "diffusion_planner.param.json. "
    "Refer to README.md for more details.";

  if (!j.contains("major_version")) {
    throw std::runtime_error("Missing 'major_version' key in JSON. " + error_msg);
  }

  const int major_version = j["major_version"].get<int>();
  if (major_version < autoware::diffusion_planner::constants::WEIGHT_MINIMUM_VERSION) {
    throw std::runtime_error(
      "Unsupported major_version: " + std::to_string(major_version) + ". " + error_msg);
  }
}

// Prediction I/O dimensions that vary by trained model.
struct PredictionDims
{
  int64_t num_prediction_agents;   // ego + predicted neighbors (== predicted_neighbor_num + 1)
  int64_t sampled_trajectory_len;  // length of the sampled_trajectories input
};

// Reads args.json and derives the model-dependent prediction dimensions.
// Falls back to the provided defaults (the full-model contract) when keys are absent.
inline PredictionDims load_prediction_dims(
  const std::string & json_path, int64_t default_num_agents, int64_t output_t)
{
  std::ifstream file(json_path);
  if (!file) {
    throw std::runtime_error("Could not open JSON file: " + json_path);
  }
  json j;
  file >> j;

  const int64_t predicted_neighbor_num =
    (j.contains("predicted_neighbor_num") && !j["predicted_neighbor_num"].is_null())
      ? j["predicted_neighbor_num"].get<int64_t>()
      : (default_num_agents - 1);
  const int64_t future_len = (j.contains("future_len") && !j["future_len"].is_null())
                               ? j["future_len"].get<int64_t>()
                               : output_t;
  // The prediction horizon (OUTPUT_T) is compile-time constant throughout the node; a model with a
  // different future_len would be silently mis-parsed. Fail with a clear message instead.
  if (future_len != output_t) {
    throw std::runtime_error(
      "Model future_len (" + std::to_string(future_len) + ") does not match the node's OUTPUT_T (" +
      std::to_string(output_t) +
      "). This build only supports future_len == " + std::to_string(output_t) + ".");
  }
  const bool temporal_decoder = j.contains("decoder_tokenization") &&
                                j["decoder_tokenization"].is_string() &&
                                j["decoder_tokenization"].get<std::string>() == "temporal";

  PredictionDims dims;
  dims.num_prediction_agents = predicted_neighbor_num + 1;
  // The legacy action head appends the current state to the sampled trajectory (future_len + 1);
  // the temporal ego decoder does not.
  dims.sampled_trajectory_len = temporal_decoder ? future_len : future_len + 1;
  return dims;
}

// Validate the Python-side contract for the velocity/temporal ego-only model.
//
// The ONNX graph still exposes the raw, padded temporal width (time_len=31).  The
// Python encoder then keeps the newest ego_history_frames samples internally (21 for
// the current HDP checkpoint) and keeps the newest six neighbor samples.  The node
// must therefore continue to provide the full 31-frame tensors; changing the node
// tensor to 21 would be an input-protocol error.  This check prevents a checkpoint
// with silently different dimensions or decoder semantics from being deployed.
inline void validate_velocity_model_contract(const std::string & json_path)
{
  std::ifstream file(json_path);
  if (!file) {
    throw std::runtime_error("Could not open JSON file: " + json_path);
  }

  json j;
  file >> j;

  const bool has_velocity_flag =
    j.contains("use_velocity_representation") && !j["use_velocity_representation"].is_null();
  const bool use_velocity_representation =
    has_velocity_flag && j["use_velocity_representation"].get<bool>();
  const bool temporal_decoder = j.contains("decoder_tokenization") &&
                                j["decoder_tokenization"].is_string() &&
                                j["decoder_tokenization"].get<std::string>() == "temporal";

  // Legacy waypoint models do not carry this HDP contract.  Leave their existing
  // compatibility path untouched.
  if ((!has_velocity_flag || !use_velocity_representation) && !temporal_decoder) {
    return;
  }

  if (!has_velocity_flag || !use_velocity_representation) {
    throw std::runtime_error(
      "The model declares a temporal decoder but use_velocity_representation is not true. "
      "Use the matching HDP checkpoint/export.");
  }
  if (
    j.contains("policy_uses_turn_indicator_history") &&
    !j["policy_uses_turn_indicator_history"].is_null()) {
    if (!j["policy_uses_turn_indicator_history"].is_boolean()) {
      throw std::runtime_error(
        "Velocity model args.json key 'policy_uses_turn_indicator_history' must be boolean.");
    }
    if (j["policy_uses_turn_indicator_history"].get<bool>()) {
      throw std::runtime_error(
        "This HDP Node/export contract does not accept turn-indicator history as a policy input.");
    }
  }
  if (!temporal_decoder) {
    throw std::runtime_error(
      "The model declares use_velocity_representation=true but decoder_tokenization is not "
      "'temporal'. The Node cannot safely use the legacy waypoint decoder path.");
  }

  const auto require_int = [&j](const char * key, const int64_t expected) {
    if (!j.contains(key) || j[key].is_null() || !j[key].is_number_integer()) {
      throw std::runtime_error(
        std::string("Velocity model args.json is missing integer key '") + key + "'.");
    }
    const int64_t actual = j[key].get<int64_t>();
    if (actual != expected) {
      throw std::runtime_error(
        std::string("Velocity model args.json key '") + key + "' is " + std::to_string(actual) +
        ", but this Node requires " + std::to_string(expected) + ".");
    }
  };

  // Raw temporal context and prediction output.
  require_int("time_len", INPUT_T_WITH_CURRENT);
  require_int("future_len", OUTPUT_T);
  require_int("ego_prediction_horizon", OUTPUT_T);
  require_int("agent_num", MAX_NUM_NEIGHBORS);
  require_int("agent_state_dim", NEIGHBOR_SHAPE[3]);
  require_int("predicted_neighbor_num", 0);

  // Non-temporal context dimensions bound by the Node's TensorRT buffers.
  require_int("static_objects_num", NUM_STATIC_OBJECTS);
  require_int("static_objects_state_dim", STATIC_OBJECTS_SHAPE[2]);
  require_int("lane_num", NUM_SEGMENTS_IN_LANE);
  require_int("lane_len", POINTS_PER_SEGMENT);
  require_int("route_num", NUM_SEGMENTS_IN_ROUTE);
  require_int("route_len", POINTS_PER_SEGMENT);
  require_int("polygon_num", NUM_POLYGONS);
  require_int("polygon_len", POINTS_PER_POLYGON);
  require_int("line_string_num", NUM_LINE_STRINGS);
  require_int("line_string_len", POINTS_PER_LINE_STRING);

  // Older velocity checkpoints omitted this field.  Python's Config supplies its
  // historical fallback in that case, so absence is compatible; validate it when
  // the field is serialized by newer training runs.
  if (j.contains("ego_history_frames") && !j["ego_history_frames"].is_null()) {
    if (!j["ego_history_frames"].is_number_integer()) {
      throw std::runtime_error(
        "Velocity model args.json key 'ego_history_frames' must be an integer.");
    }
    const int64_t ego_history_frames = j["ego_history_frames"].get<int64_t>();
    if (ego_history_frames < 1 || ego_history_frames > INPUT_T_WITH_CURRENT) {
      throw std::runtime_error(
        "Velocity model ego_history_frames=" + std::to_string(ego_history_frames) +
        " is outside [1, " + std::to_string(INPUT_T_WITH_CURRENT) + "].");
    }
  }
}

inline ObservationNormalization load_observation_normalization(const std::string & json_path)
{
  std::ifstream file(json_path);
  if (!file) {
    throw std::runtime_error("Could not open JSON file: " + json_path);
  }

  json j;
  file >> j;

  if (!j.contains("observation_normalizer")) {
    throw std::runtime_error("Missing 'observation_normalizer' key in JSON.");
  }

  ObservationNormalization norm_map;

  for (const auto & [key, val] : j["observation_normalizer"].items()) {
    std::vector<float> mean;
    std::vector<float> std_dev;
    if (val.contains("mean")) {
      for (const auto & v : val["mean"]) {
        mean.push_back(v.get<float>());
      }
    }
    if (val.contains("std")) {
      for (const auto & v : val["std"]) {
        std_dev.push_back(v.get<float>());
      }
    }
    norm_map[key] = std::make_pair(mean, std_dev);
  }

  return norm_map;
}

inline StateNormalization load_state_normalization(const std::string & json_path)
{
  std::ifstream file(json_path);
  if (!file) {
    throw std::runtime_error("Could not open JSON file: " + json_path);
  }

  json j;
  file >> j;

  if (!j.contains("state_normalizer")) {
    throw std::runtime_error("Missing 'state_normalizer' key in JSON.");
  }

  std::vector<float> mean;
  std::vector<float> std_dev;
  const auto & state_normalizer = j["state_normalizer"];
  if (state_normalizer.contains("mean")) {
    mean = flatten_json_floats(state_normalizer["mean"]);
  }
  if (state_normalizer.contains("std")) {
    std_dev = flatten_json_floats(state_normalizer["std"]);
  }

  return std::make_pair(mean, std_dev);
}
}  // namespace autoware::diffusion_planner::utils
#endif  // AUTOWARE__DIFFUSION_PLANNER__UTILS__ARG_READER_HPP_
