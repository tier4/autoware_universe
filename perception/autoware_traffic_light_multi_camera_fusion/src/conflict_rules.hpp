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

#ifndef CONFLICT_RULES_HPP_
#define CONFLICT_RULES_HPP_

#include "types.hpp"

#include <yaml-cpp/yaml.h>

#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::traffic_light::per_element
{

using ElementSet = std::set<ElementKey>;

struct ConflictRules
{
  // Each mutex group lists ElementKeys that cannot all be lit simultaneously
  // (e.g. RED-CIRCLE, AMBER-CIRCLE, GREEN-CIRCLE).
  std::vector<ElementSet> mutex_groups;
  // Each allowed_conjunction is a subset of some mutex group that is, in fact, valid
  // (e.g. UK transition state {RED-CIRCLE, AMBER-CIRCLE}).
  std::vector<ElementSet> allowed_conjunctions;
};

// JP-equivalent default used as fallback when no rules file is configured. Equivalent to
// the contents of config/conflict_rules_japan.yaml.
ConflictRules default_rules();

// Raised by the YAML loaders when an input cannot be parsed (missing keys, unknown color/shape
// strings, malformed structure, file I/O failure, etc.).
class ConflictRulesParseError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// Parse rules from an already-loaded YAML node. Exposed separately from the file loader so unit
// tests can drive the parser with in-memory YAML strings.
ConflictRules load_rules_from_yaml(const YAML::Node & node);

// Read a rules YAML file from disk and parse it. Throws ConflictRulesParseError on any failure.
ConflictRules load_rules_from_file(const std::string & path);

}  // namespace autoware::traffic_light::per_element

#endif  // CONFLICT_RULES_HPP_
