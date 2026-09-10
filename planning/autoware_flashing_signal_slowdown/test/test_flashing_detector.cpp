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

#include "../src/flashing_detector.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
using autoware::flashing_signal_slowdown::FlashingDetector;
using autoware::flashing_signal_slowdown::FlashingDetectorParams;
using autoware::flashing_signal_slowdown::LampState;

constexpr double sample_period_sec = 0.1;
constexpr double flash_half_period_sec = 0.5;

double feed_constant(
  FlashingDetector & detector, double start_sec, double duration_sec, LampState state)
{
  double time_sec = start_sec;
  while (time_sec < start_sec + duration_sec) {
    detector.update(time_sec, state);
    time_sec += sample_period_sec;
  }
  return time_sec;
}

double feed_flashing(FlashingDetector & detector, double start_sec, double duration_sec)
{
  double time_sec = start_sec;
  while (time_sec < start_sec + duration_sec) {
    const auto half_periods = static_cast<int>(std::floor(time_sec / flash_half_period_sec));
    detector.update(time_sec, half_periods % 2 == 0 ? LampState::On : LampState::Off);
    time_sec += sample_period_sec;
  }
  return time_sec;
}
}  // namespace

// A steady lamp must never trigger, otherwise detecting the flashing proves nothing.
TEST(FlashingDetector, steadyOnIsNotFlashing)
{
  FlashingDetector detector{FlashingDetectorParams{}};
  feed_constant(detector, 0.0, 10.0, LampState::On);
  EXPECT_FALSE(detector.is_flashing());
}

TEST(FlashingDetector, steadyOffIsNotFlashing)
{
  FlashingDetector detector{FlashingDetectorParams{}};
  feed_constant(detector, 0.0, 10.0, LampState::Off);
  EXPECT_FALSE(detector.is_flashing());
}

TEST(FlashingDetector, flashingIsDetected)
{
  FlashingDetector detector{FlashingDetectorParams{}};
  feed_flashing(detector, 0.0, 2.0);
  EXPECT_TRUE(detector.is_flashing());
}

TEST(FlashingDetector, flashingIsReleasedAfterItStops)
{
  FlashingDetector detector{FlashingDetectorParams{}};
  double next_sec = feed_flashing(detector, 0.0, 3.0);
  ASSERT_TRUE(detector.is_flashing());

  next_sec = feed_constant(detector, next_sec, 2.0, LampState::On);
  EXPECT_TRUE(detector.is_flashing());

  feed_constant(detector, next_sec, 2.0, LampState::On);
  EXPECT_FALSE(detector.is_flashing());
}

TEST(FlashingDetector, droppedFrameKeepsFlashing)
{
  FlashingDetector detector{FlashingDetectorParams{}};
  const double next_sec = feed_flashing(detector, 0.0, 3.0);
  ASSERT_TRUE(detector.is_flashing());

  feed_constant(detector, next_sec, 1.5, LampState::Unknown);
  EXPECT_TRUE(detector.is_flashing());
}

TEST(FlashingDetector, chatteringIsRejected)
{
  FlashingDetector detector{FlashingDetectorParams{}};
  double time_sec = 0.0;
  for (int i = 0; i < 6; ++i) {
    detector.update(time_sec, i % 2 == 0 ? LampState::On : LampState::Off);
    time_sec += sample_period_sec;
  }
  EXPECT_FALSE(detector.is_flashing());
}
