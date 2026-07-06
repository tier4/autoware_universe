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

#include "conflict_rules.hpp"

#include <tier4_perception_msgs/msg/traffic_light_element.hpp>

#include <fstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::traffic_light::per_element
{

namespace
{
using T4 = tier4_perception_msgs::msg::TrafficLightElement;

// String -> enum mappings for YAML readability. We accept only canonical names matching the
// TrafficLightElement constants; typos surface as ConflictRulesParseError at load time.
const std::unordered_map<std::string, uint8_t> & color_lookup()
{
  static const std::unordered_map<std::string, uint8_t> map{
    {"UNKNOWN", T4::UNKNOWN}, {"RED", T4::RED},     {"AMBER", T4::AMBER},
    {"GREEN", T4::GREEN},     {"WHITE", T4::WHITE},
  };
  return map;
}

const std::unordered_map<std::string, uint8_t> & shape_lookup()
{
  static const std::unordered_map<std::string, uint8_t> map{
    {"UNKNOWN", T4::UNKNOWN},
    {"CIRCLE", T4::CIRCLE},
    {"LEFT_ARROW", T4::LEFT_ARROW},
    {"RIGHT_ARROW", T4::RIGHT_ARROW},
    {"UP_ARROW", T4::UP_ARROW},
    {"UP_LEFT_ARROW", T4::UP_LEFT_ARROW},
    {"UP_RIGHT_ARROW", T4::UP_RIGHT_ARROW},
    {"DOWN_ARROW", T4::DOWN_ARROW},
    {"DOWN_LEFT_ARROW", T4::DOWN_LEFT_ARROW},
    {"DOWN_RIGHT_ARROW", T4::DOWN_RIGHT_ARROW},
    {"CROSS", T4::CROSS},
  };
  return map;
}

uint8_t parse_color(const std::string & name)
{
  const auto & m = color_lookup();
  auto it = m.find(name);
  if (it == m.end()) {
    throw ConflictRulesParseError("unknown color name: '" + name + "'");
  }
  return it->second;
}

uint8_t parse_shape(const std::string & name)
{
  const auto & m = shape_lookup();
  auto it = m.find(name);
  if (it == m.end()) {
    throw ConflictRulesParseError("unknown shape name: '" + name + "'");
  }
  return it->second;
}

ElementKey parse_element(const YAML::Node & node)
{
  if (!node.IsMap()) {
    throw ConflictRulesParseError("expected element mapping with 'color' and 'shape' keys");
  }
  const auto color_node = node["color"];
  const auto shape_node = node["shape"];
  if (!color_node || !shape_node) {
    throw ConflictRulesParseError("element entry missing 'color' or 'shape'");
  }
  return ElementKey{
    parse_color(color_node.as<std::string>()), parse_shape(shape_node.as<std::string>())};
}

ElementSet parse_element_set(const YAML::Node & group_node, const std::string & context)
{
  const auto elements_node = group_node["elements"];
  if (!elements_node || !elements_node.IsSequence()) {
    throw ConflictRulesParseError(context + " entry missing 'elements' sequence");
  }
  ElementSet set;
  for (const auto & elem_node : elements_node) {
    set.insert(parse_element(elem_node));
  }
  if (set.empty()) {
    throw ConflictRulesParseError(context + " entry has empty 'elements' sequence");
  }
  return set;
}

std::vector<ElementSet> parse_group_list(const YAML::Node & list_node, const std::string & context)
{
  std::vector<ElementSet> out;
  if (!list_node) {
    return out;  // missing key -> empty list (allowed_conjunctions can be omitted)
  }
  if (!list_node.IsSequence()) {
    throw ConflictRulesParseError("'" + context + "' must be a sequence");
  }
  for (const auto & group_node : list_node) {
    out.push_back(parse_element_set(group_node, context));
  }
  return out;
}

}  // namespace

// based on the Japanese traffic rules
ConflictRules default_rules()
{
  ConflictRules rules;
  rules.mutex_groups.push_back(
    ElementSet{{T4::RED, T4::CIRCLE}, {T4::AMBER, T4::CIRCLE}, {T4::GREEN, T4::CIRCLE}});
  return rules;
}

ConflictRules load_rules_from_yaml(const YAML::Node & node)
{
  if (!node.IsMap()) {
    throw ConflictRulesParseError("top-level YAML must be a mapping");
  }
  ConflictRules rules;
  rules.mutex_groups = parse_group_list(node["mutex_groups"], "mutex_groups");
  rules.allowed_conjunctions =
    parse_group_list(node["allowed_conjunctions"], "allowed_conjunctions");
  // Sanity: allowed_conjunctions should each be a subset of some mutex group, otherwise the rule
  // can never fire. Treat as a hard error so authoring typos surface early rather than silently
  // disabling the override.
  for (const auto & allowed : rules.allowed_conjunctions) {
    bool subset_of_any = false;
    for (const auto & mutex_group : rules.mutex_groups) {
      bool is_subset = true;
      for (const auto & key : allowed) {
        if (mutex_group.find(key) == mutex_group.end()) {
          is_subset = false;
          break;
        }
      }
      if (is_subset) {
        subset_of_any = true;
        break;
      }
    }
    if (!subset_of_any) {
      throw ConflictRulesParseError(
        "allowed_conjunctions entry is not a subset of any mutex_group; "
        "it would have no effect");
    }
  }
  return rules;
}

ConflictRules load_rules_from_file(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream.is_open()) {
    throw ConflictRulesParseError("failed to open conflict_rules_file: " + path);
  }
  try {
    return load_rules_from_yaml(YAML::Load(stream));
  } catch (const ConflictRulesParseError &) {
    throw;
  } catch (const YAML::Exception & e) {
    throw ConflictRulesParseError(
      "YAML parse error in conflict_rules_file '" + path + "': " + e.what());
  }
}

}  // namespace autoware::traffic_light::per_element
