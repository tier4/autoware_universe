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

#include "per_element_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <vector>

namespace autoware::traffic_light::per_element
{

namespace
{
using T4Element = tier4_perception_msgs::msg::TrafficLightElement;
using OutputElement = autoware_perception_msgs::msg::TrafficLightElement;

// Visibility weight applied when an ROI touches the image boundary. Hardcoded for PR 2; revisit
// during PR 4 tuning if real-world data shows truncated observations need different treatment.
constexpr double TRUNCATED_VISIBILITY_DISCOUNT = 0.5;

double probability_to_log_odds(double prob)
{
  // Clamp away from {0, 1} to keep log-odds finite — same numerical guard as the legacy path.
  prob = std::clamp(prob, 1e-9, 1.0 - 1e-9);
  return std::log(prob / (1.0 - prob));
}

double log_odds_to_probability(double log_odds)
{
  return 1.0 / (1.0 + std::exp(-log_odds));
}

bool is_unknown_element(const T4Element & element)
{
  return element.color == T4Element::UNKNOWN && element.shape == T4Element::UNKNOWN;
}

// Map t4 color/shape/status onto the corresponding autoware_perception_msgs values. The mapping
// is identical to the legacy `utils::convert_t4_to_autoware`; we replicate it here so the
// per-element path can emit output elements without round-tripping through a FusionRecord.
OutputElement make_output_element(uint8_t t4_color, uint8_t t4_shape, double confidence)
{
  OutputElement out;
  switch (t4_color) {
    case T4Element::RED:
      out.color = OutputElement::RED;
      break;
    case T4Element::AMBER:
      out.color = OutputElement::AMBER;
      break;
    case T4Element::GREEN:
      out.color = OutputElement::GREEN;
      break;
    case T4Element::WHITE:
      out.color = OutputElement::WHITE;
      break;
    default:
      out.color = OutputElement::UNKNOWN;
      break;
  }
  switch (t4_shape) {
    case T4Element::CIRCLE:
      out.shape = OutputElement::CIRCLE;
      break;
    case T4Element::LEFT_ARROW:
      out.shape = OutputElement::LEFT_ARROW;
      break;
    case T4Element::RIGHT_ARROW:
      out.shape = OutputElement::RIGHT_ARROW;
      break;
    case T4Element::UP_ARROW:
      out.shape = OutputElement::UP_ARROW;
      break;
    case T4Element::UP_LEFT_ARROW:
      out.shape = OutputElement::UP_LEFT_ARROW;
      break;
    case T4Element::UP_RIGHT_ARROW:
      out.shape = OutputElement::UP_RIGHT_ARROW;
      break;
    case T4Element::DOWN_ARROW:
      out.shape = OutputElement::DOWN_ARROW;
      break;
    case T4Element::DOWN_LEFT_ARROW:
      out.shape = OutputElement::DOWN_LEFT_ARROW;
      break;
    case T4Element::DOWN_RIGHT_ARROW:
      out.shape = OutputElement::DOWN_RIGHT_ARROW;
      break;
    case T4Element::CROSS:
      out.shape = OutputElement::CROSS;
      break;
    default:
      out.shape = OutputElement::UNKNOWN;
      break;
  }
  out.status = OutputElement::SOLID_ON;
  out.confidence = static_cast<float>(confidence);
  return out;
}

OutputElement make_unknown_output_element()
{
  return make_output_element(T4Element::UNKNOWN, T4Element::UNKNOWN, 0.0);
}

void accumulate_record_to_group(
  GroupEvidence & group, const utils::FusionRecord & record, double weight, const Config & config)
{
  for (const auto & element : record.signal.elements) {
    if (is_unknown_element(element)) {
      continue;
    }
    if (element.confidence < config.confidence_gate) {
      continue;
    }
    const ElementKey key{element.color, element.shape};
    auto & evidence = group.elements[key];
    const double evidence_log_odds = probability_to_log_odds(element.confidence);
    evidence.accumulated_log_odds += weight * (evidence_log_odds - config.prior_log_odds);
    evidence.observation_count += 1;
    if (element.confidence > evidence.best_confidence) {
      evidence.best_confidence = element.confidence;
      evidence.best_record = record;
    }
  }
}

ElementSet collect_lit_elements(const GroupEvidence & group, double on_threshold)
{
  ElementSet lit;
  for (const auto & [key, evidence] : group.elements) {
    if (evidence.accumulated_log_odds > on_threshold) {
      lit.insert(key);
    }
  }
  return lit;
}

ElementSet intersect_sets(const ElementSet & a, const ElementSet & b)
{
  ElementSet out;
  std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), std::inserter(out, out.begin()));
  return out;
}

bool is_allowed_conjunction(
  const ElementSet & lit_in_group, const std::vector<ElementSet> & allowed_conjunctions)
{
  for (const auto & allowed : allowed_conjunctions) {
    if (lit_in_group == allowed) {
      return true;
    }
  }
  return false;
}

// Return the element key with the largest accumulated log-odds among the given keys.
ElementKey argmax_log_odds(const GroupEvidence & group, const ElementSet & candidates)
{
  ElementKey best_key = *candidates.begin();
  double best_log_odds = -std::numeric_limits<double>::infinity();
  for (const auto & key : candidates) {
    const double lo = group.elements.at(key).accumulated_log_odds;
    if (lo > best_log_odds) {
      best_log_odds = lo;
      best_key = key;
    }
  }
  return best_key;
}

// Resolve all mutex conflicts on `lit`. Returns true if any conflict was detected (regardless of
// whether it was resolved or escalated).
bool resolve_mutex_conflicts(
  ElementSet & lit, const GroupEvidence & group, const Config & config, bool & escalate_failsafe)
{
  bool conflict_detected = false;
  escalate_failsafe = false;
  for (const auto & mutex_group : config.rules.mutex_groups) {
    ElementSet lit_in_group = intersect_sets(lit, mutex_group);
    if (lit_in_group.size() <= 1) {
      continue;
    }
    if (is_allowed_conjunction(lit_in_group, config.rules.allowed_conjunctions)) {
      continue;
    }
    conflict_detected = true;
    if (config.strict_mode) {
      escalate_failsafe = true;
      return true;
    }
    const ElementKey winner = argmax_log_odds(group, lit_in_group);
    for (const auto & key : lit_in_group) {
      if (key != winner) {
        lit.erase(key);
      }
    }
  }
  return conflict_detected;
}

// Pick the record to attach to the group output. Defaults to the best_record of the element
// with the highest accumulated log-odds; falls back to a default-constructed record when the
// group has no observed elements.
utils::FusionRecord pick_base_record(const GroupEvidence & group)
{
  if (group.elements.empty()) {
    return utils::FusionRecord();
  }
  auto best_it = std::max_element(
    group.elements.begin(), group.elements.end(), [](const auto & a, const auto & b) {
      return a.second.accumulated_log_odds < b.second.accumulated_log_odds;
    });
  return best_it->second.best_record;
}

}  // namespace

GroupEvidenceMap accumulate_group_evidence(
  const std::vector<utils::FusionRecord> & records,
  const std::map<IdType, std::vector<IdType>> & traffic_light_id_to_reg_ele_id,
  const Config & config)
{
  GroupEvidenceMap result;
  for (const auto & record : records) {
    const auto tl_id = record.roi.traffic_light_id;
    const auto it = traffic_light_id_to_reg_ele_id.find(tl_id);
    if (it == traffic_light_id_to_reg_ele_id.end()) {
      continue;
    }
    if (record.signal.elements.empty()) {
      continue;
    }

    const double weight = utils::is_fully_visible(record) ? 1.0 : TRUNCATED_VISIBILITY_DISCOUNT;
    for (const auto & reg_ele_id : it->second) {
      // operator[] creates the entry even if the record contributes no evidence (e.g. all
      // elements were UNKNOWN or below the confidence gate). This preserves the invariant that
      // every observed regulatory element produces some output downstream.
      accumulate_record_to_group(result[reg_ele_id], record, weight, config);
    }
  }
  return result;
}

GroupDecisionMap decide_group_states(
  const GroupEvidenceMap & group_evidence_map, const Config & config)
{
  GroupDecisionMap result;

  for (const auto & [reg_ele_id, group] : group_evidence_map) {
    GroupDecision decision;
    decision.base_record = pick_base_record(group);

    ElementSet lit = collect_lit_elements(group, config.on_threshold);
    bool escalate_failsafe = false;
    const bool conflict_detected = resolve_mutex_conflicts(lit, group, config, escalate_failsafe);

    if (escalate_failsafe) {
      decision.conflict = GroupConflict::FAILSAFE;
      decision.elements.push_back(make_unknown_output_element());
    } else {
      decision.conflict =
        conflict_detected ? GroupConflict::RESOLVED_BY_ARGMAX : GroupConflict::NONE;
      if (lit.empty()) {
        // No element passed the threshold (or the group was observed-but-empty); emit UNKNOWN
        // so downstream consumers still see an entry for the observed regulatory element.
        decision.elements.push_back(make_unknown_output_element());
      } else {
        for (const auto & [color, shape] : lit) {
          const double prob =
            log_odds_to_probability(group.elements.at({color, shape}).accumulated_log_odds);
          decision.elements.push_back(make_output_element(color, shape, prob));
        }
      }
    }
    result[reg_ele_id] = decision;
  }
  return result;
}

}  // namespace autoware::traffic_light::per_element
