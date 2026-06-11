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

#include <autoware_lane_event_classifier/lane_crossing/classifier.hpp>

namespace autoware::lane_event_classifier
{

IntentionalCrossingClassifier::IntentionalCrossingClassifier(
  bool enabled, const LaneTracker & tracker)
: enabled_{enabled}, tracker_{tracker}
{
}

void IntentionalCrossingClassifier::update(const LaneEventInput & /*input*/)
{
  // TODO(zulfaqar): implement in Phase 7 (reads geometry from tracker_).
}

uint8_t IntentionalCrossingClassifier::get_state() const
{
  // Not yet implemented: UNKNOWN = no confirmed event; the node falls back to the gate.
  return DrivingState::UNKNOWN;
}

bool IntentionalCrossingClassifier::is_enabled() const
{
  return enabled_;
}

}  // namespace autoware::lane_event_classifier
