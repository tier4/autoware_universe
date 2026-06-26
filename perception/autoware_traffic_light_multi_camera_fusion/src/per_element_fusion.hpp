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

#ifndef PER_ELEMENT_FUSION_HPP_
#define PER_ELEMENT_FUSION_HPP_

#include "conflict_rules.hpp"
#include "traffic_light_multi_camera_fusion_process.hpp"
#include "types.hpp"

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>

#include <lanelet2_core/Forward.h>

#include <cstdint>
#include <map>
#include <vector>

namespace autoware::traffic_light::per_element
{

struct Config
{
  // Per-element prior log-odds (mirrors MultiCameraFusionConfig::prior_log_odds for now).
  double prior_log_odds = 0.0;
  // Output an element when its accumulated log-odds exceed this threshold.
  double on_threshold = 0.0;
  // Drop classifier outputs whose confidence is below this gate.
  double confidence_gate = 0.0;
  // When true, any unresolved mutex conflict falls back to a failsafe (UNKNOWN) output for
  // the entire group. When false, the conflict is resolved by keeping the argmax element.
  bool strict_mode = false;
  ConflictRules rules;
};

struct ElementEvidence
{
  double accumulated_log_odds = 0.0;
  uint32_t observation_count = 0;
  double best_confidence = 0.0;
  utils::FusionRecord best_record;
};

struct GroupEvidence
{
  std::map<ElementKey, ElementEvidence> elements;
};

using IdType = lanelet::Id;
using GroupEvidenceMap = std::map<IdType, GroupEvidence>;

enum class GroupConflict {
  NONE,
  RESOLVED_BY_ARGMAX,
  FAILSAFE,
};

struct GroupDecision
{
  std::vector<autoware_perception_msgs::msg::TrafficLightElement> elements;
  utils::FusionRecord base_record;
  GroupConflict conflict = GroupConflict::NONE;
};

using GroupDecisionMap = std::map<IdType, GroupDecision>;

GroupEvidenceMap accumulate_group_evidence(
  const std::vector<utils::FusionRecord> & records,
  const std::map<IdType, std::vector<IdType>> & traffic_light_id_to_reg_ele_id,
  const Config & config);

GroupDecisionMap decide_group_states(
  const GroupEvidenceMap & group_evidence_map, const Config & config);

}  // namespace autoware::traffic_light::per_element

#endif  // PER_ELEMENT_FUSION_HPP_
