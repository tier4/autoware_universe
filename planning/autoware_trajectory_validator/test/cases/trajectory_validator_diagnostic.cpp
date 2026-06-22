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

#include "autoware/trajectory_validator/detail/trajectory_validator_diagnostic.hpp"

#include <autoware_trajectory_validator/msg/metric_report.hpp>
#include <autoware_trajectory_validator/msg/validation_report.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <optional>
#include <string>
#include <vector>

using autoware::trajectory_validator::Action;
using autoware::trajectory_validator::FilterStatusBinding;
using autoware::trajectory_validator::FilterStatusMap;
using autoware::trajectory_validator::published_level_of;
using autoware::trajectory_validator::to_action;
using autoware::trajectory_validator::TrajectoryValidatorDiagnostic;
using autoware_trajectory_validator::msg::MetricReport;
using autoware_trajectory_validator::msg::ValidationReport;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{

// Build a MetricReport fixture
MetricReport make_metric(const std::string & validator_name, uint8_t level)
{
  MetricReport m;
  m.validator_name = validator_name;
  m.level = level;
  m.metric_name = "test_metric";
  m.metric_value = 0.0;
  return m;
}

// Build a ValidationReport fixture with given metrics
ValidationReport make_report(std::vector<MetricReport> metrics)
{
  ValidationReport r;
  r.metrics = std::move(metrics);
  return r;
}

// Spin until predicate is true or timeout
bool spin_until(
  rclcpp::Node::SharedPtr node, std::function<bool()> pred,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(2000))
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  rclcpp::Rate rate(200);
  while (std::chrono::steady_clock::now() < end) {
    rclcpp::spin_some(node);
    if (pred()) return true;
    rate.sleep();
  }
  return false;
}

// Find a DiagnosticStatus in an array by name suffix (the part after ": ")
const DiagnosticStatus * find_status(const DiagnosticArray & arr, const std::string & name_suffix)
{
  for (const auto & diag_status : arr.status) {
    const auto pos = diag_status.name.find(": ");
    if (pos != std::string::npos && diag_status.name.substr(pos + 2) == name_suffix) {
      return &diag_status;
    }
  }
  return nullptr;
}

// Helper: create a node, subscribe, run diagnostic once, collect all published arrays
struct DiagHarness
{
  rclcpp::Node::SharedPtr node;
  std::vector<DiagnosticArray> received;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr sub;

  explicit DiagHarness(const std::string & node_name = "test_node")
  {
    node = rclcpp::Node::make_shared(node_name);
    sub = node->create_subscription<DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(20),
      [this](const DiagnosticArray::SharedPtr msg) { received.push_back(*msg); });
  }

  // Wait for publishers to be connected, then call update_and_publish, then collect messages.
  void run(TrajectoryValidatorDiagnostic & diag, const std::vector<ValidationReport> & reports)
  {
    // Wait until at least one publisher is connected on /diagnostics
    spin_until(node, [this]() { return sub->get_publisher_count() > 0; });
    received.clear();
    const auto stamp = node->get_clock()->now();
    diag.update_and_publish(reports, stamp);
    // Spin briefly to collect all published messages from this call
    spin_until(node, [this]() { return !received.empty(); });
    // Spin a bit more to collect any additional messages from the same call
    rclcpp::spin_some(node);
    rclcpp::spin_some(node);
  }

  // Merge all received arrays into one flat list of statuses
  std::vector<DiagnosticStatus> all_statuses() const
  {
    std::vector<DiagnosticStatus> result;
    for (const auto & arr : received) {
      for (const auto & s : arr.status) {
        result.push_back(s);
      }
    }
    return result;
  }

  // Find a status by name suffix across all received messages (last wins)
  std::optional<DiagnosticStatus> find(const std::string & name_suffix) const
  {
    std::optional<DiagnosticStatus> found;
    for (const auto & arr : received) {
      if (const auto * matched = find_status(arr, name_suffix)) {
        found = *matched;
      }
    }
    return found;
  }
};

}  // namespace

// ============================================================================
// Tests 1-2: to_action and published_level_of mapping helpers
// ============================================================================

TEST(RiskAction, ToActionMapping)
{
  EXPECT_EQ(to_action(MetricReport::OK), Action::NONE);
  EXPECT_EQ(to_action(MetricReport::WARN), Action::NONE);
  EXPECT_EQ(to_action(MetricReport::ERROR), Action::MODERATE);
}

TEST(RiskAction, PublishedLevelOf)
{
  EXPECT_EQ(published_level_of(Action::NONE), DiagnosticStatus::OK);
  EXPECT_EQ(published_level_of(Action::COMFORTABLE), DiagnosticStatus::WARN);
  EXPECT_EQ(published_level_of(Action::MODERATE), DiagnosticStatus::ERROR);
  EXPECT_EQ(published_level_of(Action::EMERGENCY), DiagnosticStatus::ERROR);
}

// ============================================================================
// Tests 3-10: Core diagnostic behaviour
// ============================================================================

class TrajectoryValidatorDiagnosticTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }

  // Build a simple single-entry map for "validator_a" -> MODERATE -> "status_a"
  static FilterStatusMap single_map(
    const std::string & validator = "validator_a", const std::string & status_name = "status_a")
  {
    FilterStatusMap filter_map;
    filter_map[validator].name_by_action[Action::MODERATE] = status_name;
    return filter_map;
  }
};

// Test 3: Best trajectory all-OK (another worse trajectory at ERROR) -> every status OK
TEST_F(TrajectoryValidatorDiagnosticTest, BestAllOkOtherError)
{
  DiagHarness h("t3_node");
  TrajectoryValidatorDiagnostic diag(*h.node, single_map(), "");

  std::vector<ValidationReport> reports = {
    make_report({make_metric("validator_a", MetricReport::OK)}),    // best
    make_report({make_metric("validator_a", MetricReport::ERROR)})  // worse
  };

  h.run(diag, reports);

  auto status = h.find("status_a");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->level, DiagnosticStatus::OK);
}

// Test 4: Best trajectory has one binding validator at ERROR -> status fires at ERROR
TEST_F(TrajectoryValidatorDiagnosticTest, BestHasBindingValidatorAtError)
{
  DiagHarness h("t4_node");
  TrajectoryValidatorDiagnostic diag(*h.node, single_map(), "");

  std::vector<ValidationReport> reports = {
    make_report({make_metric("validator_a", MetricReport::ERROR)})};

  h.run(diag, reports);

  auto status = h.find("status_a");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->level, DiagnosticStatus::ERROR);
}

// Test 5: Best-available: best trajectory OK/WARN while worse is ERROR -> every status OK
TEST_F(TrajectoryValidatorDiagnosticTest, BestAvailableOkWhileWorseError)
{
  DiagHarness h("t5_node");
  TrajectoryValidatorDiagnostic diag(*h.node, single_map(), "");

  std::vector<ValidationReport> reports = {
    make_report({make_metric("validator_a", MetricReport::WARN)}),  // best (WARN -> NONE action)
    make_report({make_metric("validator_a", MetricReport::ERROR)})  // worse
  };

  h.run(diag, reports);

  auto status = h.find("status_a");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->level, DiagnosticStatus::OK);
}

// Test 6: Two validators binding at the same action on the best trajectory -> both fire
TEST_F(TrajectoryValidatorDiagnosticTest, TwoValidatorsBindingBothFire)
{
  DiagHarness h("t6_node");
  FilterStatusMap filter_map;
  filter_map["validator_a"].name_by_action[Action::MODERATE] = "status_a";
  filter_map["validator_b"].name_by_action[Action::MODERATE] = "status_b";

  TrajectoryValidatorDiagnostic diag(*h.node, filter_map, "");

  std::vector<ValidationReport> reports = {make_report(
    {make_metric("validator_a", MetricReport::ERROR),
     make_metric("validator_b", MetricReport::ERROR)})};

  h.run(diag, reports);

  auto sa = h.find("status_a");
  auto sb = h.find("status_b");
  ASSERT_TRUE(sa.has_value());
  ASSERT_TRUE(sb.has_value());
  EXPECT_EQ(sa->level, DiagnosticStatus::ERROR);
  EXPECT_EQ(sb->level, DiagnosticStatus::ERROR);
}

// Test 7: Renew — ERROR cycle then all-OK cycle -> status returns to OK
TEST_F(TrajectoryValidatorDiagnosticTest, Renew)
{
  DiagHarness h("t7_node");
  TrajectoryValidatorDiagnostic diag(*h.node, single_map(), "");

  // First cycle: ERROR
  std::vector<ValidationReport> error_reports = {
    make_report({make_metric("validator_a", MetricReport::ERROR)})};
  h.run(diag, error_reports);
  {
    auto status = h.find("status_a");
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->level, DiagnosticStatus::ERROR);
  }

  // Second cycle: OK
  std::vector<ValidationReport> ok_reports = {
    make_report({make_metric("validator_a", MetricReport::OK)})};
  h.run(diag, ok_reports);
  {
    auto status = h.find("status_a");
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->level, DiagnosticStatus::OK);
  }
}

// Test 8: (validator, action) with empty name -> no status fired (no preset entry)
TEST_F(TrajectoryValidatorDiagnosticTest, EmptyNameNoStatusFired)
{
  DiagHarness h("t8_node");
  FilterStatusMap filter_map;
  filter_map["validator_a"].name_by_action[Action::MODERATE] = "";  // empty name

  // No DiagnosticsInterface is created, so nothing is published
  TrajectoryValidatorDiagnostic diag(*h.node, filter_map, "");

  std::vector<ValidationReport> reports = {
    make_report({make_metric("validator_a", MetricReport::ERROR)})};

  // Since no DiagnosticsInterface is created, no messages will arrive
  // Just verify it doesn't crash and received is empty
  spin_until(h.node, [&h]() { return h.sub->get_publisher_count() > 0; });
  h.received.clear();
  diag.update_and_publish(reports, h.node->get_clock()->now());
  // Spin briefly — no messages expected
  rclcpp::spin_some(h.node);
  rclcpp::spin_some(h.node);

  EXPECT_TRUE(h.all_statuses().empty());
}

// Test 9: No reports -> no_candidate_name fires at ERROR
TEST_F(TrajectoryValidatorDiagnosticTest, NoReportsFiresNoCandidateName)
{
  DiagHarness h("t9_node");
  TrajectoryValidatorDiagnostic diag(*h.node, {}, "no_candidate_status");

  h.run(diag, {});

  auto status = h.find("no_candidate_status");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->level, DiagnosticStatus::ERROR);
}

// Test 9b: No reports, empty no_candidate_name -> nothing fires
TEST_F(TrajectoryValidatorDiagnosticTest, NoReportsEmptyNoCandidateNameNothing)
{
  DiagHarness h("t9b_node");
  TrajectoryValidatorDiagnostic diag(*h.node, {}, "");

  spin_until(h.node, [&h]() { return h.sub->get_publisher_count() > 0; });
  h.received.clear();
  diag.update_and_publish({}, h.node->get_clock()->now());
  rclcpp::spin_some(h.node);
  rclcpp::spin_some(h.node);

  EXPECT_TRUE(h.all_statuses().empty());
}

// Test 10: Two validators mapped with distinct names -> both in preset, both published each cycle
TEST_F(TrajectoryValidatorDiagnosticTest, TwoValidatorsDistinctNamesPublishedEachCycle)
{
  DiagHarness h("t10_node");
  FilterStatusMap filter_map;
  filter_map["validator_a"].name_by_action[Action::MODERATE] = "status_a";
  filter_map["validator_b"].name_by_action[Action::MODERATE] = "status_b";

  TrajectoryValidatorDiagnostic diag(*h.node, filter_map, "");

  // OK cycle — both statuses still published (at OK level)
  std::vector<ValidationReport> reports = {make_report(
    {make_metric("validator_a", MetricReport::OK), make_metric("validator_b", MetricReport::OK)})};

  h.run(diag, reports);

  auto sa = h.find("status_a");
  auto sb = h.find("status_b");
  ASSERT_TRUE(sa.has_value()) << "status_a must be published every cycle";
  ASSERT_TRUE(sb.has_value()) << "status_b must be published every cycle";
  EXPECT_EQ(sa->level, DiagnosticStatus::OK);
  EXPECT_EQ(sb->level, DiagnosticStatus::OK);
}

// ============================================================================
// Tests 11-13: Shadow mode (dedicated)
// ============================================================================

class ShadowModeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }
};

// Test 11: Shadow validator at ERROR on best trajectory (enforced ones OK) -> every status OK
TEST_F(ShadowModeTest, ShadowErrorOnBestDoesNotFireStatus)
{
  DiagHarness h("t11_node");
  FilterStatusMap filter_map;
  filter_map["shadow_v"].name_by_action[Action::MODERATE] = "shadow_status";

  TrajectoryValidatorDiagnostic diag(*h.node, filter_map, "", {"shadow_v"});

  std::vector<ValidationReport> reports = {
    make_report({make_metric("shadow_v", MetricReport::ERROR)})};

  h.run(diag, reports);

  auto status = h.find("shadow_status");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->level, DiagnosticStatus::OK);
}

// Test 12: Best trajectory: enforced OK + shadow ERROR -> shadow never binds; no status fires
TEST_F(ShadowModeTest, EnforcedOkShadowErrorNoStatusFires)
{
  DiagHarness h("t12_node");
  FilterStatusMap filter_map;
  filter_map["enforced_v"].name_by_action[Action::MODERATE] = "enforced_status";
  filter_map["shadow_v"].name_by_action[Action::MODERATE] = "shadow_status";

  TrajectoryValidatorDiagnostic diag(*h.node, filter_map, "", {"shadow_v"});

  std::vector<ValidationReport> reports = {make_report(
    {make_metric("enforced_v", MetricReport::OK), make_metric("shadow_v", MetricReport::ERROR)})};

  h.run(diag, reports);

  // enforced_v is OK -> action NONE -> nothing fires
  auto se = h.find("enforced_status");
  auto ss = h.find("shadow_status");
  ASSERT_TRUE(se.has_value());
  ASSERT_TRUE(ss.has_value());
  EXPECT_EQ(se->level, DiagnosticStatus::OK);
  EXPECT_EQ(ss->level, DiagnosticStatus::OK);
}

// Test 13: Shadow validator with mapping entry + ERROR + no enforced above OK -> still raises
// nothing
TEST_F(ShadowModeTest, ShadowWithMappingEntryErrorStillRaisesNothing)
{
  DiagHarness h("t13_node");
  FilterStatusMap filter_map;
  // Only shadow_v is in the map
  filter_map["shadow_v"].name_by_action[Action::MODERATE] = "shadow_status";

  TrajectoryValidatorDiagnostic diag(*h.node, filter_map, "", {"shadow_v"});

  // Only metrics come from shadow_v (enforced set is empty)
  std::vector<ValidationReport> reports = {
    make_report({make_metric("shadow_v", MetricReport::ERROR)})};

  h.run(diag, reports);

  auto status = h.find("shadow_status");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->level, DiagnosticStatus::OK);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
