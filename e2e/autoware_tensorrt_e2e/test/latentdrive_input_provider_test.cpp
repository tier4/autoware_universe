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

#include "autoware/tensorrt_e2e/providers/latentdrive_input_provider.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace autoware::tensorrt_e2e::latentdrive
{

namespace
{
constexpr int64_t kWidth = 512;
constexpr int64_t kHeight = 256;
const std::array<float, 3> kImageNetMean = {0.485f, 0.456f, 0.406f};
const std::array<float, 3> kImageNetInverseStd = {1.0f / 0.229f, 1.0f / 0.224f, 1.0f / 0.225f};

/// Expected normalized value of an 8-bit channel value.
float normalized(const int value, const int channel)
{
  return (static_cast<float>(value) / 255.0f - kImageNetMean[channel]) *
         kImageNetInverseStd[channel];
}
}  // namespace

TEST(LatentDrivePreprocessTest, RejectsInvalidFrames)
{
  std::vector<float> out(3 * kHeight * kWidth);
  EXPECT_FALSE(
    preprocess_frame(cv::Mat(), kWidth, kHeight, kImageNetMean, kImageNetInverseStd, out.data()));
  const cv::Mat gray(64, 128, CV_8UC1, cv::Scalar(7));
  EXPECT_FALSE(
    preprocess_frame(gray, kWidth, kHeight, kImageNetMean, kImageNetInverseStd, out.data()));
}

TEST(LatentDrivePreprocessTest, SwapsChannelsNormalizesAndPacksChw)
{
  // A frame already at the model size: no crop, no resize, so every value is exact.
  const cv::Mat bgr(
    static_cast<int>(kHeight), static_cast<int>(kWidth), CV_8UC3, cv::Scalar(30, 120, 210));
  std::vector<float> out(3 * kHeight * kWidth);
  ASSERT_TRUE(
    preprocess_frame(bgr, kWidth, kHeight, kImageNetMean, kImageNetInverseStd, out.data()));

  const size_t plane = kHeight * kWidth;
  // Channel 0 of the output is R (the source's third channel), channel 2 is B.
  EXPECT_NEAR(out[0], normalized(210, 0), 1e-5f);
  EXPECT_NEAR(out[plane], normalized(120, 1), 1e-5f);
  EXPECT_NEAR(out[2 * plane], normalized(30, 2), 1e-5f);
  EXPECT_NEAR(out[plane - 1], normalized(210, 0), 1e-5f);
  EXPECT_NEAR(out[3 * plane - 1], normalized(30, 2), 1e-5f);
}

TEST(LatentDrivePreprocessTest, CentreCropsHeightToHalfWidthBeforeResizing)
{
  // 2880x1860 as the vehicle camera delivers: the crop keeps rows [210, 1650). Paint the
  // bands outside the crop white and the crop itself a solid colour; if the crop were wrong,
  // white would bleed into the output.
  cv::Mat bgr(1860, 2880, CV_8UC3, cv::Scalar(255, 255, 255));
  bgr(cv::Rect(0, 210, 2880, 1440)).setTo(cv::Scalar(40, 80, 160));
  std::vector<float> out(3 * kHeight * kWidth);
  ASSERT_TRUE(
    preprocess_frame(bgr, kWidth, kHeight, kImageNetMean, kImageNetInverseStd, out.data()));

  const size_t plane = kHeight * kWidth;
  for (size_t i = 0; i < plane; i += 97) {
    EXPECT_NEAR(out[i], normalized(160, 0), 1e-4f);
    EXPECT_NEAR(out[plane + i], normalized(80, 1), 1e-4f);
    EXPECT_NEAR(out[2 * plane + i], normalized(40, 2), 1e-4f);
  }
}

TEST(LatentDriveFrameSlotsTest, PicksNearestFramePerSlot)
{
  // 10 Hz stamps over 2 s; four slots 0.5 s apart end at the newest stamp.
  std::vector<double> stamps;
  for (int i = 0; i <= 20; ++i) {
    stamps.push_back(100.0 + 0.1 * i);
  }
  const auto slots = select_frame_slots(stamps, 4, 0.5, 0.15);
  ASSERT_TRUE(slots.has_value());
  EXPECT_EQ(*slots, (std::vector<size_t>{5, 10, 15, 20}));
}

TEST(LatentDriveFrameSlotsTest, ToleratesADroppedFrame)
{
  // The frame that slot 1 would take (t = 100.5) is missing; its neighbour is 0.1 s away.
  std::vector<double> stamps;
  for (int i = 0; i <= 15; ++i) {
    if (i == 5) {
      continue;
    }
    stamps.push_back(100.0 + 0.1 * i);
  }
  const auto slots = select_frame_slots(stamps, 4, 0.5, 0.15);
  ASSERT_TRUE(slots.has_value());
  EXPECT_NEAR(stamps[(*slots)[1]], 100.5, 0.1 + 1e-9);
  EXPECT_NEAR(stamps[(*slots)[0]], 100.0, 1e-9);
  EXPECT_NEAR(stamps[(*slots)[3]], 101.5, 1e-9);
}

TEST(LatentDriveFrameSlotsTest, NotReadyWithoutEnoughHistory)
{
  const std::vector<double> stamps = {100.0, 100.1, 100.2, 100.3};  // 0.3 s, need 1.5 s
  EXPECT_FALSE(select_frame_slots(stamps, 4, 0.5, 0.15).has_value());
  EXPECT_FALSE(select_frame_slots({}, 4, 0.5, 0.15).has_value());
  // A single-frame model is ready with one frame.
  EXPECT_TRUE(select_frame_slots({100.0}, 1, 0.5, 0.15).has_value());
}

TEST(LatentDriveSubgoalTest, WalksArcLengthFromTheNearestPoint)
{
  // A straight reference along x, 1 m apart, from 0 to 100 m.
  std::vector<Eigen::Vector2d> reference;
  for (int i = 0; i <= 100; ++i) {
    reference.emplace_back(static_cast<double>(i), 0.0);
  }
  const Eigen::Vector2d subgoal = subgoal_along(reference, Eigen::Vector2d(20.3, 0.4), 50.0);
  EXPECT_DOUBLE_EQ(subgoal.x(), 70.0);
  EXPECT_DOUBLE_EQ(subgoal.y(), 0.0);
}

TEST(LatentDriveSubgoalTest, StopsAtTheEndOfTheReference)
{
  std::vector<Eigen::Vector2d> reference;
  for (int i = 0; i <= 30; ++i) {
    reference.emplace_back(static_cast<double>(i), 0.0);
  }
  const Eigen::Vector2d subgoal = subgoal_along(reference, Eigen::Vector2d(0.0, 0.0), 50.0);
  EXPECT_DOUBLE_EQ(subgoal.x(), 30.0);
  EXPECT_THROW(subgoal_along({}, Eigen::Vector2d(0.0, 0.0), 50.0), std::invalid_argument);
}

}  // namespace autoware::tensorrt_e2e::latentdrive
