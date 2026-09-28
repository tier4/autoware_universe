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

#include "autoware/tensorrt_e2e/bev_feature/detection_decode.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace autoware::tensorrt_e2e
{

namespace
{

// A proposal in the head's own encoding, with a geometry that makes the encoded centre
// the metric one (out_size_factor 1, voxel 1 m, range starting at 0).
struct Proposal
{
  float x;
  float y;
  int64_t label;
  float score;
  float yaw_norm{1.0f};
};

class Decode : public ::testing::Test
{
protected:
  void SetUp() override
  {
    int devices = 0;
    if (cudaGetDeviceCount(&devices) != cudaSuccess || devices == 0) {
      GTEST_SKIP() << "no CUDA device";
    }
  }

  // Two classes, two bands: [0, 50) and [50, 100).
  std::vector<double> limits{50.0, 100.0};
  //                              CAR  PEDESTRIAN
  std::vector<double> thresholds{
    0.2, 0.3,   // [0, 50)
    0.4, 0.5};  // [50, 100)
  std::vector<float> yaw_norm_floor{0.0f, 0.0f};

  std::vector<DecodedBox> run(const std::vector<Proposal> & proposals)
  {
    const int n = static_cast<int>(proposals.size());
    std::vector<float> bbox(10 * n, 0.0f);
    std::vector<float> score(n);
    std::vector<int64_t> label(n);
    for (int i = 0; i < n; ++i) {
      bbox[0 * n + i] = proposals[i].x;
      bbox[1 * n + i] = proposals[i].y;
      bbox[6 * n + i] = 0.0f;                   // sin
      bbox[7 * n + i] = proposals[i].yaw_norm;  // cos
      score[i] = proposals[i].score;
      label[i] = proposals[i].label;
    }
    const auto table = make_score_threshold_table(limits, thresholds, 2);
    DetectionDecodeGeometry geometry{};
    geometry.num_proposals = n;
    geometry.num_classes = 2;
    geometry.num_bands = static_cast<int32_t>(table.squared_upper_limits.size());
    geometry.out_size_factor = 1.0f;
    geometry.voxel_size_x = 1.0f;
    geometry.voxel_size_y = 1.0f;

    auto upload = [](const auto & host) {
      using T = typename std::decay_t<decltype(host)>::value_type;
      T * device = nullptr;
      EXPECT_EQ(cudaMalloc(&device, host.size() * sizeof(T)), cudaSuccess);
      EXPECT_EQ(
        cudaMemcpy(device, host.data(), host.size() * sizeof(T), cudaMemcpyHostToDevice),
        cudaSuccess);
      return device;
    };
    float * bbox_d = upload(bbox);
    float * score_d = upload(score);
    int64_t * label_d = upload(label);
    float * yaw_d = upload(yaw_norm_floor);
    float * limits_d = upload(table.squared_upper_limits);
    float * thresholds_d = upload(table.thresholds);
    DecodedBox * boxes_d = nullptr;
    int32_t * count_d = nullptr;
    EXPECT_EQ(cudaMalloc(&boxes_d, std::max(n, 1) * sizeof(DecodedBox)), cudaSuccess);
    EXPECT_EQ(cudaMalloc(&count_d, sizeof(int32_t)), cudaSuccess);
    EXPECT_EQ(cudaMemset(count_d, 0, sizeof(int32_t)), cudaSuccess);

    EXPECT_EQ(
      launch_decode_detections(
        label_d, bbox_d, score_d, yaw_d, limits_d, thresholds_d, geometry, boxes_d, count_d,
        nullptr),
      cudaSuccess);
    int32_t count = 0;
    EXPECT_EQ(cudaMemcpy(&count, count_d, sizeof(int32_t), cudaMemcpyDeviceToHost), cudaSuccess);
    std::vector<DecodedBox> boxes(count);
    EXPECT_EQ(
      cudaMemcpy(boxes.data(), boxes_d, count * sizeof(DecodedBox), cudaMemcpyDeviceToHost),
      cudaSuccess);
    for (void * p : std::vector<void *>{
           bbox_d, score_d, label_d, yaw_d, limits_d, thresholds_d, boxes_d, count_d}) {
      cudaFree(p);
    }
    sort_like_bevfusion(boxes);
    return boxes;
  }
};

}  // namespace

TEST_F(Decode, ThresholdIsPerDistanceBandAndClass)
{
  const auto boxes = run({
    {10.0f, 0.0f, 0, 0.25f},   // near CAR, 0.25 >= 0.2: kept
    {0.0f, 30.0f, 1, 0.25f},   // near PEDESTRIAN, 0.25 < 0.3: dropped
    {60.0f, 0.0f, 0, 0.35f},   // far CAR, 0.35 < 0.4: dropped
    {0.0f, -70.0f, 1, 0.55f},  // far PEDESTRIAN, 0.55 >= 0.5: kept
  });
  ASSERT_EQ(boxes.size(), 2u);
  EXPECT_EQ(boxes[0].proposal, 3);
  EXPECT_EQ(boxes[1].proposal, 0);
}

TEST_F(Decode, ThresholdItselfIsKept)
{
  EXPECT_EQ(run({{10.0f, 0.0f, 0, 0.2f}}).size(), 1u);
}

TEST_F(Decode, BandUpperLimitBelongsToTheNextBand)
{
  // Radial distance exactly 50: `squared_distance < limit^2` is false, so [50, 100).
  EXPECT_EQ(run({{30.0f, 40.0f, 0, 0.3f}}).size(), 0u);
  EXPECT_EQ(run({{29.9f, 40.0f, 0, 0.3f}}).size(), 1u);
}

TEST_F(Decode, BeyondTheLastBandIsDropped)
{
  EXPECT_EQ(run({{100.0f, 0.0f, 0, 0.99f}}).size(), 0u);
}

TEST_F(Decode, YawNormGateZeroesTheScore)
{
  yaw_norm_floor = {0.3f, 0.0f};
  EXPECT_EQ(run({{10.0f, 0.0f, 0, 0.9f, 0.2f}}).size(), 0u);
  EXPECT_EQ(run({{10.0f, 0.0f, 1, 0.9f, 0.2f}}).size(), 1u);
}

TEST_F(Decode, OutOfRangeLabelIsDropped)
{
  EXPECT_EQ(run({{10.0f, 0.0f, 2, 0.9f}, {10.0f, 0.0f, -1, 0.9f}}).size(), 0u);
}

TEST_F(Decode, OrderIsScoreDescendingThenProposal)
{
  const auto boxes = run({
    {1.0f, 0.0f, 0, 0.5f},
    {2.0f, 0.0f, 0, 0.9f},
    {3.0f, 0.0f, 0, 0.5f},
    {4.0f, 0.0f, 0, 0.7f},
  });
  ASSERT_EQ(boxes.size(), 4u);
  EXPECT_EQ(boxes[0].proposal, 1);
  EXPECT_EQ(boxes[1].proposal, 3);
  EXPECT_EQ(boxes[2].proposal, 0);
  EXPECT_EQ(boxes[3].proposal, 2);
}

TEST_F(Decode, DecodesTheBoxLikeTheBbboxCoder)
{
  const auto boxes = run({{12.5f, -3.0f, 1, 0.8f}});
  ASSERT_EQ(boxes.size(), 1u);
  EXPECT_FLOAT_EQ(boxes[0].x, 12.5f);
  EXPECT_FLOAT_EQ(boxes[0].y, -3.0f);
  EXPECT_FLOAT_EQ(boxes[0].length, 1.0f);  // exp(0)
  EXPECT_FLOAT_EQ(boxes[0].yaw, 0.0f);     // atan2(0, 1)
  EXPECT_EQ(boxes[0].label, 1);
}

TEST(ScoreThresholdTable, BuiltAsAutowareBevfusionBuildsIt)
{
  const auto table = make_score_threshold_table({50.0, 90.0}, {0.2, 1.5, -0.1, 0.4}, 2);
  EXPECT_EQ(table.squared_upper_limits, (std::vector<float>{2500.0f, 8100.0f}));
  EXPECT_EQ(table.thresholds, (std::vector<float>{0.2f, 0.0f, 0.0f, 0.4f}));
}

TEST(ScoreThresholdTable, RejectsInconsistentTables)
{
  EXPECT_THROW(make_score_threshold_table({}, {}, 2), std::runtime_error);
  EXPECT_THROW(make_score_threshold_table({50.0, 90.0}, {0.2, 0.3}, 2), std::runtime_error);
  EXPECT_THROW(
    make_score_threshold_table({90.0, 50.0}, {0.2, 0.3, 0.2, 0.3}, 2), std::runtime_error);
  EXPECT_THROW(make_score_threshold_table({0.0}, {0.2, 0.3}, 2), std::runtime_error);
}

}  // namespace autoware::tensorrt_e2e
