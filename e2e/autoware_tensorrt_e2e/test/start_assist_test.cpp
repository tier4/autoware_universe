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


#include "autoware/tensorrt_e2e/start_assist.hpp"

#include <gtest/gtest.h>
#include <rclcpp/duration.hpp>

#include <cmath>

namespace autoware::tensorrt_e2e
{
namespace
{
StartAssistParams enabled()
{
  StartAssistParams params;
  params.enable = true;
  return params;
}
}  // namespace

TEST(StartAssistTest, StartsOnlyOnAnEngagementAtStandstill)
{
  StartAssist assist(enabled());
  // Already autonomous when the node starts: no edge, no assist.
  EXPECT_EQ(assist.update(true, 0.0, 0.0), StartAssist::Event::kNone);
  EXPECT_FALSE(assist.active());
  // Manual, then engaged while moving: the model keeps it.
  EXPECT_EQ(assist.update(false, 2.0, 0.1), StartAssist::Event::kNone);
  EXPECT_EQ(assist.update(true, 2.0, 0.2), StartAssist::Event::kNone);
  // A stop later in the same session is the model's.
  EXPECT_EQ(assist.update(true, 0.0, 5.0), StartAssist::Event::kNone);
  // Manual, then engaged at standstill.
  EXPECT_EQ(assist.update(false, 0.0, 6.0), StartAssist::Event::kNone);
  EXPECT_EQ(assist.update(true, 0.0, 6.1), StartAssist::Event::kStarted);
  EXPECT_TRUE(assist.active());
}

TEST(StartAssistTest, HandsOverAtSpeedTimeoutOrDisengagement)
{
  StartAssist assist(enabled());
  assist.update(false, 0.0, 0.0);
  ASSERT_EQ(assist.update(true, 0.0, 0.1), StartAssist::Event::kStarted);
  EXPECT_EQ(assist.update(true, 0.3, 1.0), StartAssist::Event::kNone);
  EXPECT_EQ(assist.update(true, 0.5, 1.5), StartAssist::Event::kHandedOver);
  EXPECT_FALSE(assist.active());

  assist.update(false, 0.0, 10.0);
  ASSERT_EQ(assist.update(true, 0.0, 10.1), StartAssist::Event::kStarted);
  EXPECT_EQ(assist.update(true, 0.0, 14.1), StartAssist::Event::kTimedOut);

  assist.update(false, 0.0, 20.0);
  ASSERT_EQ(assist.update(true, 0.0, 20.1), StartAssist::Event::kStarted);
  EXPECT_EQ(assist.update(false, 0.0, 20.5), StartAssist::Event::kDisengaged);
  EXPECT_FALSE(assist.active());
}

TEST(StartAssistTest, DisabledNeverStarts)
{
  StartAssist assist{StartAssistParams{}};
  assist.update(false, 0.0, 0.0);
  EXPECT_EQ(assist.update(true, 0.0, 0.1), StartAssist::Event::kNone);
  EXPECT_FALSE(assist.active());
}

TEST(StartAssistTest, RampReachesTheDistanceAtTheDurationAlongTheHeading)
{
  StartAssist assist(enabled());  // 5 m after 4 s: 0.625 m/s^2, 2.5 m/s at 4 s
  assist.update(false, 0.0, 0.0);
  assist.update(true, 0.0, 0.0);
  geometry_msgs::msg::Pose pose;
  pose.position.x = 10.0;
  pose.position.y = 20.0;
  pose.orientation.z = std::sin(M_PI / 4.0);  // yaw 90 deg: along +y
  pose.orientation.w = std::cos(M_PI / 4.0);
  const auto points = assist.make_points(pose, 0.0, 0.1, 4.0);
  ASSERT_EQ(points.size(), 40u);
  EXPECT_NEAR(points.back().pose.position.y - 20.0, 5.0, 1e-6);
  EXPECT_NEAR(points.back().pose.position.x, 10.0, 1e-6);
  EXPECT_NEAR(points.back().longitudinal_velocity_mps, 2.5, 1e-5);
  EXPECT_NEAR(points.front().longitudinal_velocity_mps, 0.0625, 1e-5);
  EXPECT_DOUBLE_EQ(rclcpp::Duration(points.front().time_from_start).seconds(), 0.1);

  // One second in, the trajectory starts from wherever the vehicle is and continues the ramp.
  const auto later = assist.make_points(pose, 1.0, 0.1, 4.0);
  EXPECT_NEAR(later.front().longitudinal_velocity_mps, 0.6875, 1e-5);
  // Distance 1.0 -> 5.0 s on the ramp, capped at 2.5 m/s after 4 s: 4.6875 + 2.5 - 0.3125.
  EXPECT_NEAR(later.back().pose.position.y - 20.0, 5.0 - 0.3125 + 2.5, 1e-6);
  EXPECT_NEAR(later.back().longitudinal_velocity_mps, 2.5, 1e-5);
}

}  // namespace autoware::tensorrt_e2e
