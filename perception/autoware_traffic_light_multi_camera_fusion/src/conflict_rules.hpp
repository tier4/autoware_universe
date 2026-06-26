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

#include <set>
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

// JP-equivalent default: {RED, AMBER, GREEN} circles are mutually exclusive, no allowed
// conjunctions, arrows / WHITE / CROSS are unconstrained. Used until PR 3 introduces
// YAML-loaded region presets.
ConflictRules default_japan_rules();

}  // namespace autoware::traffic_light::per_element

#endif  // CONFLICT_RULES_HPP_
