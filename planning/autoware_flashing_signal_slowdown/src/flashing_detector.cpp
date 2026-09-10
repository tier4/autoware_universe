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

#include "flashing_detector.hpp"

namespace autoware::flashing_signal_slowdown
{

FlashingDetector::FlashingDetector(const FlashingDetectorParams & params) : params_(params)
{
}

void FlashingDetector::update(double stamp_sec, LampState state)
{
  // Unknown says nothing about the lamp, so a dropped frame cannot change the state.
  if (state != LampState::Unknown) {
    if (last_state_ && *last_state_ != state) {
      transitions_.push_back(stamp_sec);
      last_transition_sec_ = stamp_sec;
    }
    last_state_ = state;
  }

  evaluate(stamp_sec);
}

void FlashingDetector::evaluate(double now_sec)
{
  remove_expired_entries(now_sec);

  if (!is_flashing_) {
    is_flashing_ = has_flashing_pattern();
    return;
  }

  // Release on elapsed time only, so that a single sample cannot cancel the slowdown.
  if (
    !last_transition_sec_ || now_sec - *last_transition_sec_ > params_.release_no_transition_sec) {
    is_flashing_ = false;
  }
}

void FlashingDetector::remove_expired_entries(double now_sec)
{
  while (!transitions_.empty() && now_sec - transitions_.front() > params_.history_window_sec) {
    transitions_.pop_front();
  }
}

bool FlashingDetector::has_flashing_pattern() const
{
  if (transitions_.empty()) {
    return false;
  }

  // Several transitions crammed into a few milliseconds are recognition noise, not a 1Hz lamp.
  return static_cast<int>(transitions_.size()) >= params_.detect_min_transitions &&
         transitions_.back() - transitions_.front() >= params_.detect_min_span_sec;
}

}  // namespace autoware::flashing_signal_slowdown
