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

#include "../src/conflict_rules.hpp"

#include <tier4_perception_msgs/msg/traffic_light_element.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <string>

namespace
{
namespace per_element = autoware::traffic_light::per_element;
using T4 = tier4_perception_msgs::msg::TrafficLightElement;

per_element::ConflictRules load(const std::string & yaml_text)
{
  return per_element::load_rules_from_yaml(YAML::Load(yaml_text));
}
}  // namespace

TEST(ConflictRulesLoader, JapanPresetEquivalentToHardcodedDefault)
{
  // The shipped Japan YAML preset should produce the exact same ConflictRules as the
  // hardcoded default_rules() fallback.
  const auto from_yaml = load(R"(
mutex_groups:
  - elements:
      - { color: RED, shape: CIRCLE }
      - { color: AMBER, shape: CIRCLE }
      - { color: GREEN, shape: CIRCLE }
allowed_conjunctions: []
)");

  const auto from_default = per_element::default_rules();
  EXPECT_EQ(from_yaml.mutex_groups, from_default.mutex_groups);
  EXPECT_EQ(from_yaml.allowed_conjunctions, from_default.allowed_conjunctions);
}

TEST(ConflictRulesLoader, UkPresetHasRedAmberAllowedConjunction)
{
  const auto rules = load(R"(
mutex_groups:
  - elements:
      - { color: RED, shape: CIRCLE }
      - { color: AMBER, shape: CIRCLE }
      - { color: GREEN, shape: CIRCLE }
allowed_conjunctions:
  - elements:
      - { color: RED, shape: CIRCLE }
      - { color: AMBER, shape: CIRCLE }
)");

  ASSERT_EQ(rules.mutex_groups.size(), 1u);
  ASSERT_EQ(rules.allowed_conjunctions.size(), 1u);
  const per_element::ElementSet expected{{T4::RED, T4::CIRCLE}, {T4::AMBER, T4::CIRCLE}};
  EXPECT_EQ(rules.allowed_conjunctions.front(), expected);
}

TEST(ConflictRulesLoader, OmittedAllowedConjunctionsKeyDefaultsToEmpty)
{
  const auto rules = load(R"(
mutex_groups:
  - elements:
      - { color: RED, shape: CIRCLE }
      - { color: GREEN, shape: CIRCLE }
)");
  EXPECT_TRUE(rules.allowed_conjunctions.empty());
}

TEST(ConflictRulesLoader, UnknownColorRaisesParseError)
{
  EXPECT_THROW(
    load(R"(
mutex_groups:
  - elements:
      - { color: PURPLE, shape: CIRCLE }
)"),
    per_element::ConflictRulesParseError);
}

TEST(ConflictRulesLoader, UnknownShapeRaisesParseError)
{
  EXPECT_THROW(
    load(R"(
mutex_groups:
  - elements:
      - { color: RED, shape: SQUARE }
)"),
    per_element::ConflictRulesParseError);
}

TEST(ConflictRulesLoader, MissingShapeKeyRaisesParseError)
{
  EXPECT_THROW(
    load(R"(
mutex_groups:
  - elements:
      - { color: RED }
)"),
    per_element::ConflictRulesParseError);
}

TEST(ConflictRulesLoader, NonSubsetAllowedConjunctionIsRejected)
{
  // {RED-LEFT_ARROW, AMBER-CIRCLE} is not a subset of the only mutex_group (R/A/G circles),
  // so it would never fire as an override. The loader should reject it.
  EXPECT_THROW(
    load(R"(
mutex_groups:
  - elements:
      - { color: RED, shape: CIRCLE }
      - { color: AMBER, shape: CIRCLE }
      - { color: GREEN, shape: CIRCLE }
allowed_conjunctions:
  - elements:
      - { color: RED, shape: LEFT_ARROW }
      - { color: AMBER, shape: CIRCLE }
)"),
    per_element::ConflictRulesParseError);
}

TEST(ConflictRulesLoader, EmptyElementsListIsRejected)
{
  EXPECT_THROW(
    load(R"(
mutex_groups:
  - elements: []
)"),
    per_element::ConflictRulesParseError);
}

TEST(ConflictRulesLoader, NonMapTopLevelIsRejected)
{
  EXPECT_THROW(load("- not_a_map"), per_element::ConflictRulesParseError);
}

TEST(ConflictRulesLoader, MissingFileRaisesParseError)
{
  EXPECT_THROW(
    per_element::load_rules_from_file("/nonexistent/path/conflict_rules.yaml"),
    per_element::ConflictRulesParseError);
}
