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

#include "autoware/diffusion_planner/postprocessing/turn_indicator_manager.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner::test
{
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using postprocess::TurnIndicatorManager;

namespace
{
constexpr double kHold = 1.0;       // release-confirmation window [s]
constexpr double kOnConfirm = 0.2;  // activation-confirmation window [s]

std::vector<float> logits_for(const std::size_t dense_class)
{
  std::vector<float> logits(TURN_INDICATOR_OUTPUT_DIM, -10.0F);
  logits.at(dense_class) = 10.0F;
  return logits;
}

rclcpp::Time at(const double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
}

TurnIndicatorManager make_manager(const double hold = kHold, const double on_confirm = kOnConfirm)
{
  return TurnIndicatorManager(
    rclcpp::Duration::from_seconds(hold), rclcpp::Duration::from_seconds(on_confirm));
}

TEST(TurnIndicatorManagerTest, MapsDenseThreeClassOutputWithoutKeepClass)
{
  // With zero-length windows the manager degenerates to per-frame argmax mapping.
  const auto stamp = at(1.0);
  for (const auto & [dense_class, command] :
       {std::pair<std::size_t, uint8_t>{0U, TurnIndicatorsCommand::DISABLE},
        std::pair<std::size_t, uint8_t>{1U, TurnIndicatorsCommand::ENABLE_LEFT},
        std::pair<std::size_t, uint8_t>{2U, TurnIndicatorsCommand::ENABLE_RIGHT}}) {
    auto manager = make_manager(0.0, 0.0);
    EXPECT_EQ(manager.evaluate(logits_for(dense_class), stamp).command, command);
  }
}

TEST(TurnIndicatorManagerTest, ActivationRequiresSustainedEvidence)
{
  auto manager = make_manager();

  // Onset is confirmed only after the same observation persists for on_confirmation.
  EXPECT_EQ(manager.evaluate(logits_for(1U), at(1.0)).command, TurnIndicatorsCommand::DISABLE);
  EXPECT_EQ(manager.evaluate(logits_for(1U), at(1.1)).command, TurnIndicatorsCommand::DISABLE);
  EXPECT_EQ(manager.evaluate(logits_for(1U), at(1.2)).command, TurnIndicatorsCommand::ENABLE_LEFT);
}

TEST(TurnIndicatorManagerTest, SingleFrameGlitchIsRejected)
{
  auto manager = make_manager();

  EXPECT_EQ(manager.evaluate(logits_for(1U), at(1.0)).command, TurnIndicatorsCommand::DISABLE);
  // The contrary evidence is interrupted; the window restarts.
  EXPECT_EQ(manager.evaluate(logits_for(0U), at(1.1)).command, TurnIndicatorsCommand::DISABLE);
  EXPECT_EQ(manager.evaluate(logits_for(1U), at(1.2)).command, TurnIndicatorsCommand::DISABLE);
  EXPECT_EQ(manager.evaluate(logits_for(1U), at(1.3)).command, TurnIndicatorsCommand::DISABLE);
  EXPECT_EQ(manager.evaluate(logits_for(1U), at(1.4)).command, TurnIndicatorsCommand::ENABLE_LEFT);
}

TEST(TurnIndicatorManagerTest, ReleaseRequiresSustainedContraryEvidence)
{
  auto manager = make_manager();

  // Activate LEFT.
  manager.evaluate(logits_for(1U), at(1.0));
  manager.evaluate(logits_for(1U), at(1.1));
  ASSERT_EQ(manager.evaluate(logits_for(1U), at(1.2)).command, TurnIndicatorsCommand::ENABLE_LEFT);

  // DISABLE evidence shorter than hold_duration is ignored...
  EXPECT_EQ(manager.evaluate(logits_for(0U), at(1.3)).command, TurnIndicatorsCommand::ENABLE_LEFT);
  EXPECT_EQ(manager.evaluate(logits_for(0U), at(2.2)).command, TurnIndicatorsCommand::ENABLE_LEFT);
  // ...an agreeing frame resets the release window...
  EXPECT_EQ(manager.evaluate(logits_for(1U), at(2.25)).command, TurnIndicatorsCommand::ENABLE_LEFT);
  // ...and only a full hold_duration of consistent DISABLE releases the signal.
  EXPECT_EQ(manager.evaluate(logits_for(0U), at(2.3)).command, TurnIndicatorsCommand::ENABLE_LEFT);
  EXPECT_EQ(manager.evaluate(logits_for(0U), at(3.2)).command, TurnIndicatorsCommand::ENABLE_LEFT);
  EXPECT_EQ(manager.evaluate(logits_for(0U), at(3.3)).command, TurnIndicatorsCommand::DISABLE);
}

TEST(TurnIndicatorManagerTest, DirectionFlipUsesReleaseWindow)
{
  auto manager = make_manager();

  // Activate LEFT.
  manager.evaluate(logits_for(1U), at(1.0));
  manager.evaluate(logits_for(1U), at(1.1));
  ASSERT_EQ(manager.evaluate(logits_for(1U), at(1.2)).command, TurnIndicatorsCommand::ENABLE_LEFT);

  // A direct LEFT->RIGHT flip needs the same sustained window as turning off.
  EXPECT_EQ(manager.evaluate(logits_for(2U), at(1.3)).command, TurnIndicatorsCommand::ENABLE_LEFT);
  EXPECT_EQ(manager.evaluate(logits_for(2U), at(2.2)).command, TurnIndicatorsCommand::ENABLE_LEFT);
  EXPECT_EQ(manager.evaluate(logits_for(2U), at(2.3)).command, TurnIndicatorsCommand::ENABLE_RIGHT);
}

TEST(TurnIndicatorManagerTest, MixedContraryEvidenceKeepsRestartingTheWindow)
{
  auto manager = make_manager();

  // Activate LEFT.
  manager.evaluate(logits_for(1U), at(1.0));
  manager.evaluate(logits_for(1U), at(1.1));
  ASSERT_EQ(manager.evaluate(logits_for(1U), at(1.2)).command, TurnIndicatorsCommand::ENABLE_LEFT);

  // Alternating DISABLE / RIGHT never accumulates a consistent window.
  double t = 1.3;
  for (int i = 0; i < 30; ++i) {
    const std::size_t dense_class = (i % 2 == 0) ? 0U : 2U;
    EXPECT_EQ(
      manager.evaluate(logits_for(dense_class), at(t)).command,
      TurnIndicatorsCommand::ENABLE_LEFT);
    t += 0.1;
  }
}

TEST(TurnIndicatorManagerTest, InvalidLogitShapeDisablesStaleCommand)
{
  auto manager = make_manager(0.0, 0.0);
  EXPECT_EQ(
    manager.evaluate(logits_for(2U), at(1.0)).command, TurnIndicatorsCommand::ENABLE_RIGHT);
  EXPECT_EQ(manager.evaluate({}, at(2.0)).command, TurnIndicatorsCommand::DISABLE);
}

TEST(TurnIndicatorManagerTest, BackwardsTimeRestartsEvidenceWindow)
{
  auto manager = make_manager();

  // Partially confirmed onset...
  EXPECT_EQ(manager.evaluate(logits_for(1U), at(10.0)).command, TurnIndicatorsCommand::DISABLE);
  EXPECT_EQ(manager.evaluate(logits_for(1U), at(10.1)).command, TurnIndicatorsCommand::DISABLE);
  // ...a time regression (sim reset / bag loop) restarts confirmation instead of
  // promoting against a stale window.
  EXPECT_EQ(manager.evaluate(logits_for(1U), at(1.0)).command, TurnIndicatorsCommand::DISABLE);
  EXPECT_EQ(manager.evaluate(logits_for(1U), at(1.1)).command, TurnIndicatorsCommand::DISABLE);
  EXPECT_EQ(manager.evaluate(logits_for(1U), at(1.2)).command, TurnIndicatorsCommand::ENABLE_LEFT);
}
}  // namespace
}  // namespace autoware::diffusion_planner::test
