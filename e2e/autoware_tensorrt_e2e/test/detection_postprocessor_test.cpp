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

#include "autoware/tensorrt_e2e/postprocess/detection_postprocessor.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

namespace
{

using autoware::bevfusion::Box3D;

Box3D box(const float x, const float y, const int label, const float score)
{
  Box3D b{};
  b.x = x;
  b.y = y;
  b.length = 4.0f;
  b.width = 2.0f;
  b.height = 1.5f;
  b.label = label;
  b.score = score;
  return b;
}

std_msgs::msg::Header header()
{
  std_msgs::msg::Header h;
  h.frame_id = "base_link";
  return h;
}

DetectionPostprocessor::Config config()
{
  DetectionPostprocessor::Config c;
  c.class_names = {"CAR", "PEDESTRIAN"};
  return c;
}

}  // namespace

TEST(CircleNms, SuppressesCloserThanTheThresholdAcrossClasses)
{
  // Score-descending, as the decode hands them over.
  const auto kept = circle_nms(
    {box(10.0f, 0.0f, 0, 0.9f), box(10.3f, 0.0f, 1, 0.8f), box(10.6f, 0.0f, 0, 0.7f)}, 0.5f);
  // The second is within 0.5 m of the first and goes; the third is 0.6 m from the first
  // and only near the suppressed second, so it stays.
  ASSERT_EQ(kept.size(), 2u);
  EXPECT_FLOAT_EQ(kept[0].score, 0.9f);
  EXPECT_FLOAT_EQ(kept[1].score, 0.7f);
}

TEST(CircleNms, ExactlyAtTheThresholdIsKept)
{
  EXPECT_EQ(circle_nms({box(0.0f, 0.0f, 0, 0.9f), box(0.5f, 0.0f, 0, 0.8f)}, 0.5f).size(), 2u);
}

TEST(CircleNms, ZeroDisables)
{
  EXPECT_EQ(circle_nms({box(0.0f, 0.0f, 0, 0.9f), box(0.0f, 0.0f, 0, 0.8f)}, 0.0f).size(), 2u);
}

TEST(DetectionPostprocessor, BuildsOneObjectPerSurvivor)
{
  DetectionPostprocessor post(config());
  const auto objects = post.build(
    {box(10.0f, 0.0f, 0, 0.9f), box(10.2f, 0.0f, 1, 0.3f), box(-30.0f, 5.0f, 1, 0.4f)}, header());
  ASSERT_EQ(objects.objects.size(), 2u);
  EXPECT_EQ(objects.header.frame_id, "base_link");
  EXPECT_FLOAT_EQ(objects.objects[0].existence_probability, 0.9f);
  EXPECT_EQ(
    objects.objects[1].classification[0].label,
    autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN);
}

TEST(DetectionPostprocessor, RejectsBadParameters)
{
  auto c = config();
  c.circle_nms_dist_threshold = -1.0;
  EXPECT_THROW(DetectionPostprocessor{c}, std::runtime_error);
  c = config();
  c.class_names.clear();
  EXPECT_THROW(DetectionPostprocessor{c}, std::runtime_error);
}

}  // namespace autoware::tensorrt_e2e
