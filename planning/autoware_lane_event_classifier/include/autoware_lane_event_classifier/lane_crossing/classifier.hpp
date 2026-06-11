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

#ifndef AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CROSSING__CLASSIFIER_HPP_
#define AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CROSSING__CLASSIFIER_HPP_

#include <autoware_lane_event_classifier/lane_event_classifier_base.hpp>

#include <cstdint>
#include <string>

namespace autoware::lane_event_classifier
{

class LaneTracker;

/** @brief Classifier for intentional lane crossings (stub; implemented later). */
class IntentionalCrossingClassifier : public LaneEventClassifierBase
{
public:
  IntentionalCrossingClassifier(bool enabled, const LaneTracker & tracker);
  void update(const LaneEventInput & input) override;
  [[nodiscard]] uint8_t get_state() const override;
  [[nodiscard]] bool is_enabled() const override;
  [[nodiscard]] std::string name() const override { return "lane_crossing"; }

private:
  bool enabled_{false};
  const LaneTracker & tracker_;  // generic lane queries (owned by the node)
};

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CROSSING__CLASSIFIER_HPP_
