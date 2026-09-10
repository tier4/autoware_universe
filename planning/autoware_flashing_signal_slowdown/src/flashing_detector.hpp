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

#ifndef FLASHING_DETECTOR_HPP_
#define FLASHING_DETECTOR_HPP_

#include <deque>
#include <optional>

namespace autoware::flashing_signal_slowdown
{

enum class LampState { Unknown, Off, On };

struct FlashingDetectorParams
{
  double history_window_sec{3.0};
  int detect_min_transitions{3};
  double detect_min_span_sec{0.8};
  double release_no_transition_sec{3.0};
};

// A dark lamp and the dark half of a flashing cycle are identical in a single sample, so this
// counts On/Off transitions inside a sliding window instead of trying to tell them apart.
// Time is passed in as seconds so that the state machine can be tested without a node.
class FlashingDetector
{
public:
  explicit FlashingDetector(const FlashingDetectorParams & params);

  void update(double stamp_sec, LampState state);
  void evaluate(double now_sec);
  bool is_flashing() const { return is_flashing_; }

private:
  void remove_expired_entries(double now_sec);
  bool has_flashing_pattern() const;

  FlashingDetectorParams params_;
  // The samples between two transitions carry no information, so only the transition stamps are
  // kept inside the window.
  std::deque<double> transitions_;
  std::optional<LampState> last_state_;
  // Never expired, so that the release stays independent of the window length.
  std::optional<double> last_transition_sec_;
  bool is_flashing_{false};
};

}  // namespace autoware::flashing_signal_slowdown

#endif  // FLASHING_DETECTOR_HPP_
