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

#ifndef AUTOWARE__TENSORRT_E2E__TURN_INDICATOR_FILTER_HPP_
#define AUTOWARE__TENSORRT_E2E__TURN_INDICATOR_FILTER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace autoware::tensorrt_e2e
{

/**
 * @brief Turns the model's per-cycle turn-indicator logits into a steady command.
 *
 * The model's head is stateless: it sees the scene, never the signal's own report, so each
 * cycle's answer is independent and can flicker at a decision boundary. The state lives here.
 * Logit `i` is `TurnIndicatorsCommand` value `i + 1` (DISABLE = 1, ENABLE_LEFT = 2,
 * ENABLE_RIGHT = 3). The command changes only when one other class has had at least
 * `min_probability` for `confirm_cycles` consecutive cycles and `hold_s` has passed since the
 * previous change. The command starts as DISABLE.
 *
 * Header-only, with no ROS types, so the rule is tested without a ROS build.
 */
class TurnIndicatorFilter
{
public:
  static constexpr size_t kNumClasses = 3;
  static constexpr uint8_t kDisable = 1;

  struct Params
  {
    double min_probability{0.7};
    int confirm_cycles{1};
    double hold_s{1.0};
  };

  explicit TurnIndicatorFilter(const Params & params) : params_(params) {}

  /// Command after this cycle. A malformed or non-finite `logits` leaves the state as it was.
  uint8_t update(const std::vector<float> & logits, double stamp_s)
  {
    if (logits.size() != kNumClasses || !std::all_of(logits.begin(), logits.end(), [](float v) {
          return std::isfinite(v);
        })) {
      return command_;
    }
    const float max_logit = *std::max_element(logits.begin(), logits.end());
    double sum = 0.0;
    double probability[kNumClasses];
    for (size_t i = 0; i < kNumClasses; ++i) {
      probability[i] = std::exp(static_cast<double>(logits[i] - max_logit));
      sum += probability[i];
    }
    const size_t best = static_cast<size_t>(
      std::distance(probability, std::max_element(probability, probability + kNumClasses)));
    const auto candidate = static_cast<uint8_t>(best + 1);
    if (candidate == command_ || probability[best] / sum < params_.min_probability) {
      pending_count_ = 0;
      return command_;
    }
    pending_count_ = (candidate == pending_) ? pending_count_ + 1 : 1;
    pending_ = candidate;
    if (pending_count_ >= params_.confirm_cycles && stamp_s - last_change_s_ >= params_.hold_s) {
      command_ = candidate;
      last_change_s_ = stamp_s;
      pending_count_ = 0;
    }
    return command_;
  }

  uint8_t command() const { return command_; }

private:
  Params params_;
  uint8_t command_{kDisable};
  uint8_t pending_{0};
  int pending_count_{0};
  double last_change_s_{-1e18};
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__TURN_INDICATOR_FILTER_HPP_
