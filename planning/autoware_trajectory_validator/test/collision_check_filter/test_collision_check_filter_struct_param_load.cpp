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

#include "autoware/trajectory_validator/filters/safety/collision_check_filter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

// =============================================================================
// Unit-level: resolve_per_class() spec — base / NaN-fallback semantics.
//
// generate_parameter_library populates every class in object_class with the
// schema default. For float maps with NaN default, non-overridden classes show
// up with NaN and resolve_per_class redirects them to "base". Bool maps have
// no base fallback — missing class keys throw so callers fail loud.
// =============================================================================
namespace autoware::trajectory_validator::plugin::safety
{
namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

TEST(ResolvePerClass, SampleParamsDouble_overrideTakesPrecedence)
{
  validator::Params::CollisionCheck::Rss::SampleParamsDouble sdm;
  sdm.object_class_map["base"].value = 2.0;
  sdm.object_class_map["pedestrian"].value = 5.0;

  EXPECT_DOUBLE_EQ(resolve_per_class(sdm, "pedestrian"), 5.0);
}

TEST(ResolvePerClass, SampleParamsDouble_nanFallsBackToBase)
{
  validator::Params::CollisionCheck::Rss::SampleParamsDouble sdm;
  sdm.object_class_map["base"].value = 2.0;
  sdm.object_class_map["pedestrian"].value = 5.0;
  sdm.object_class_map["car"].value = kNaN;
  sdm.object_class_map["truck"].value = kNaN;

  EXPECT_DOUBLE_EQ(resolve_per_class(sdm, "car"), 2.0);
  EXPECT_DOUBLE_EQ(resolve_per_class(sdm, "truck"), 2.0);
}

TEST(ResolvePerClass, SampleParamsDouble_classMissingEntirelyFallsBackToBase)
{
  validator::Params::CollisionCheck::Rss::SampleParamsDouble sdm;
  sdm.object_class_map["base"].value = 2.0;
  EXPECT_DOUBLE_EQ(resolve_per_class(sdm, "car"), 2.0);
}

TEST(ResolvePerClass, SampleParamsBool_perClassFlagsRespected)
{
  validator::Params::CollisionCheck::Rss::SampleParamsBool ea;
  ea.object_class_map["base"].value = true;
  ea.object_class_map["over_drivable"].value = false;
  ea.object_class_map["car"].value = true;

  EXPECT_TRUE(resolve_per_class(ea, "car"));
  EXPECT_FALSE(resolve_per_class(ea, "over_drivable"));
  EXPECT_THROW(resolve_per_class(ea, "missing"), std::runtime_error);
}

TEST(ResolvePerClass, DracWarnThresholdSampleParamsDouble_nanFallsBackToBase)
{
  validator::Params::CollisionCheck::Drac::WarnThreshold::SampleParamsDouble ea;
  ea.object_class_map["base"].value = -3.0;
  ea.object_class_map["pedestrian"].value = -2.0;
  ea.object_class_map["truck"].value = kNaN;

  EXPECT_DOUBLE_EQ(resolve_per_class(ea, "pedestrian"), -2.0);
  EXPECT_DOUBLE_EQ(resolve_per_class(ea, "truck"), -3.0);
}

TEST(ResolvePerClass, PetWarnSampleParamsDouble_nanFallsBackToBase)
{
  validator::Params::CollisionCheck::PetCollision::WarnThreshold::SampleParamsDouble g;
  g.object_class_map["base"].value = 1.0;
  g.object_class_map["car"].value = 2.5;
  g.object_class_map["bus"].value = kNaN;

  EXPECT_DOUBLE_EQ(resolve_per_class(g, "car"), 2.5);
  EXPECT_DOUBLE_EQ(resolve_per_class(g, "bus"), 1.0);
}

TEST(ResolvePerClass, MissingBaseThrows)
{
  validator::Params::CollisionCheck::Rss::SampleParamsDouble sdm;
  sdm.object_class_map["pedestrian"].value = 5.0;
  EXPECT_THROW(resolve_per_class(sdm, "car"), std::runtime_error);
}

TEST(ResolvePerClass, AcceptsWrapperStructDirectly)
{
  validator::Params::CollisionCheck::Rss::SampleParamsDouble sdm;
  sdm.object_class_map["base"].value = 2.0;
  sdm.object_class_map["pedestrian"].value = 5.0;

  EXPECT_DOUBLE_EQ(resolve_per_class(sdm, "pedestrian"), 5.0);
  EXPECT_DOUBLE_EQ(resolve_per_class(sdm, "car"), 2.0);  // base fallback
}

TEST(ResolvePerClass, ScalarPassesThrough)
{
  EXPECT_DOUBLE_EQ(resolve_per_class(0.4, "car"), 0.4);
  EXPECT_TRUE(resolve_per_class(true, "anything"));
}

namespace
{
// Test-only helper: dumps a snapshot of the collision_check params so
// reviewers can eyeball that YAML overrides (and schema defaults) flow into
// the generated Params struct. Mirrors the prior in-filter print_debug_info()
// but lives here because it is pure debug output, not production logic.
template <class Wrapper>
std::string format_class_map(const Wrapper & wrapper)
{
  std::string out = "{";
  bool first = true;
  for (const auto & [cls, entry] : wrapper.object_class_map) {
    if (!first) out += ", ";
    first = false;
    using ValueT = std::decay_t<decltype(entry.value)>;
    if constexpr (std::is_floating_point_v<ValueT>) {
      out += cls + ":" + (std::isnan(entry.value) ? "NaN" : std::to_string(entry.value));
    } else if constexpr (std::is_same_v<ValueT, bool>) {
      out += cls + ":" + (entry.value ? "true" : "false");
    } else {
      out += cls + ":<unsupported>";
    }
  }
  return out + "}";
}

void print_debug_info(const validator::Params::CollisionCheck & cc)
{
  const auto & pet = cc.pet_collision;
  const auto & drac = cc.drac;
  const auto & rss = cc.rss;
  const auto logger = rclcpp::get_logger("CollisionCheckFilterParamLoadTest");

  RCLCPP_INFO(
    logger,
    "pet_collision: enable=%d, ego_braking_delay=%.3f, ego_assumed_accel=%.3f, "
    "warn(ego_first=%.3f, object_first=%.3f), error(ego_first=%.3f, object_first=%.3f)",
    pet.enable_assessment, pet.ego_total_braking_delay, pet.ego_assumed_acceleration,
    pet.warn_threshold.ego_first_passing_time_gap, pet.warn_threshold.object_first_passing_time_gap,
    pet.error_threshold.ego_first_passing_time_gap,
    pet.error_threshold.object_first_passing_time_gap);
  RCLCPP_INFO(
    logger, "pet.warn.sample_params_double=%s",
    format_class_map(pet.warn_threshold.sample_params_double).c_str());

  RCLCPP_INFO(
    logger, "drac: enable=%d, ego_braking_delay=%.3f, warn.ego_accel=%.3f, error.ego_accel=%.3f",
    drac.enable_assessment, drac.ego_total_braking_delay, drac.warn_threshold.ego_acceleration,
    drac.error_threshold.ego_acceleration);
  RCLCPP_INFO(
    logger, "drac.warn.sample_params_double=%s",
    format_class_map(drac.warn_threshold.sample_params_double).c_str());

  RCLCPP_INFO(
    logger,
    "rss: enable=%d, stop_distance_margin=%.3f, ego_braking_delay=%.3f, "
    "object_assumed_accel=%.3f, error.ego_accel=%.3f",
    rss.enable_assessment, rss.stop_distance_margin, rss.ego_total_braking_delay,
    rss.object_assumed_acceleration, rss.error_threshold.ego_acceleration);
  RCLCPP_INFO(
    logger, "rss.sample_params_double=%s", format_class_map(rss.sample_params_double).c_str());
  RCLCPP_INFO(
    logger, "rss.sample_params_bool=%s", format_class_map(rss.sample_params_bool).c_str());
}
}  // namespace

}  // namespace autoware::trajectory_validator::plugin::safety

// =============================================================================
// Integration-level: verifies that config/trajectory_validator.param.yaml is
// loaded end-to-end (YAML -> NodeOptions parameter overrides -> ParamListener
// -> validator::Params). The test prints a snapshot via print_debug_info() and
// asserts a representative subset of the values.
// =============================================================================
namespace autoware::trajectory_validator
{

class CollisionCheckFilterParamLoadTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(CollisionCheckFilterParamLoadTest, YamlParamsReachCollisionCheckParams)
{
  rclcpp::NodeOptions options;

  const auto trajectory_validator_param_path =
    ament_index_cpp::get_package_share_directory("autoware_trajectory_validator") +
    "/config/trajectory_validator.param.yaml";
  autoware::test_utils::updateNodeOptions(options, {trajectory_validator_param_path});

  auto node = std::make_shared<rclcpp::Node>("collision_check_param_load_test_node", options);

  validator::ParamListener listener(node->get_node_parameters_interface());
  const auto params = listener.get_params();
  const auto & cc = params.collision_check;

  // Visible snapshot for human review of YAML flow.
  plugin::safety::print_debug_info(cc);

  // Assertions match config/trajectory_validator.param.yaml.
  EXPECT_FALSE(cc.drac.enable_assessment);
  EXPECT_DOUBLE_EQ(cc.drac.warn_threshold.ego_acceleration, -2.0);
  EXPECT_DOUBLE_EQ(cc.drac.error_threshold.ego_acceleration, -5.0);
  EXPECT_DOUBLE_EQ(cc.pet_collision.error_threshold.ego_first_passing_time_gap, 0.6);
  EXPECT_DOUBLE_EQ(cc.pet_collision.error_threshold.object_first_passing_time_gap, 0.3);
  EXPECT_TRUE(cc.rss.enable_assessment);
  EXPECT_DOUBLE_EQ(cc.rss.stop_distance_margin, 2.0);

  // Per-class maps: object_class array has 13 entries, schema default expands
  // them all; YAML only overrides a subset, the rest remain at NaN / false.
  EXPECT_EQ(cc.drac.warn_threshold.sample_params_double.object_class_map.size(), 13U);
  EXPECT_DOUBLE_EQ(
    cc.drac.warn_threshold.sample_params_double.object_class_map.at("base").value, -2.0);
  EXPECT_DOUBLE_EQ(
    cc.drac.warn_threshold.sample_params_double.object_class_map.at("pedestrian").value, -1.0);
  EXPECT_TRUE(
    std::isnan(cc.drac.warn_threshold.sample_params_double.object_class_map.at("car").value));

  EXPECT_EQ(cc.rss.sample_params_bool.object_class_map.size(), 13U);
  EXPECT_TRUE(cc.rss.sample_params_bool.object_class_map.at("base").value);
  EXPECT_FALSE(cc.rss.sample_params_bool.object_class_map.at("over_drivable").value);
  EXPECT_FALSE(cc.rss.sample_params_bool.object_class_map.at("under_drivable").value);
  // Non-overridden classes default to false.
  EXPECT_FALSE(cc.rss.sample_params_bool.object_class_map.at("car").value);
}

}  // namespace autoware::trajectory_validator
