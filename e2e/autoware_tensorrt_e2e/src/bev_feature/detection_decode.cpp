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

#include "autoware/tensorrt_e2e/bev_feature/detection_decode.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

ScoreThresholdTable make_score_threshold_table(
  const std::vector<double> & distance_bin_upper_limits,
  const std::vector<double> & score_thresholds, const size_t num_classes)
{
  const auto & limits = distance_bin_upper_limits;
  if (limits.empty()) {
    throw std::runtime_error(
      "bev_feature.detection.distance_bin_upper_limits is empty: the score thresholds are "
      "one row per distance band, and a package needs at least one band");
  }
  for (size_t i = 0; i < limits.size(); ++i) {
    if (!(limits[i] > 0.0) || (i > 0 && !(limits[i] > limits[i - 1]))) {
      throw std::runtime_error(
        "bev_feature.detection.distance_bin_upper_limits must be positive and strictly "
        "ascending");
    }
  }
  if (score_thresholds.size() != limits.size() * num_classes) {
    throw std::runtime_error(
      "bev_feature.detection.score_thresholds has " + std::to_string(score_thresholds.size()) +
      " entries for " + std::to_string(limits.size()) + " distance band(s) x " +
      std::to_string(num_classes) + " classes");
  }

  // As autoware_bevfusion's BEVFusionConfig builds them: the limits squared in float, so
  // the kernel compares squared distances; a threshold outside [0, 1) read as 0.
  ScoreThresholdTable table;
  table.squared_upper_limits.reserve(limits.size());
  for (const double limit : limits) {
    const auto value = static_cast<float>(limit);
    table.squared_upper_limits.push_back(value * value);
  }
  table.thresholds.reserve(score_thresholds.size());
  for (const double threshold : score_thresholds) {
    const auto value = static_cast<float>(threshold);
    table.thresholds.push_back((value >= 0.f && value < 1.f) ? value : 0.f);
  }
  return table;
}

void sort_like_bevfusion(std::vector<DecodedBox> & boxes)
{
  std::sort(boxes.begin(), boxes.end(), [](const DecodedBox & a, const DecodedBox & b) {
    return a.score != b.score ? a.score > b.score : a.proposal < b.proposal;
  });
}

}  // namespace autoware::tensorrt_e2e
