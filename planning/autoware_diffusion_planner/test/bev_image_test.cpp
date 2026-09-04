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

#include "autoware/diffusion_planner/preprocessing/bev_image.hpp"

#include "autoware/diffusion_planner/conversion/agent.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner::test
{
namespace
{
constexpr int64_t NEAR_SCALE = 0;
constexpr int64_t FAR_SCALE = 1;
constexpr int64_t CENTER_PIXEL = BEV_IMAGE_SIZE / 2;

constexpr size_t PLANE_SIZE = static_cast<size_t>(BEV_IMAGE_SIZE) * BEV_IMAGE_SIZE;
constexpr size_t SAMPLE_SIZE = static_cast<size_t>(BEV_NUM_SCALES) * BEV_NUM_CHANNELS * PLANE_SIZE;

/// An all-zero input map: every polyline is pure padding and no agent is present, so a render of
/// it draws nothing except the ego box and the ego history sitting at the origin.
preprocess::InputDataMap make_empty_input_data(const int64_t batch_size)
{
  const auto zeros = [batch_size](const int64_t elements_per_batch) {
    return std::vector<float>(static_cast<size_t>(batch_size * elements_per_batch), 0.0f);
  };

  preprocess::InputDataMap input_data_map;
  input_data_map["lanes"] = zeros(NUM_SEGMENTS_IN_LANE * POINTS_PER_SEGMENT * SEGMENT_POINT_DIM);
  input_data_map["route_lanes"] =
    zeros(NUM_SEGMENTS_IN_ROUTE * POINTS_PER_SEGMENT * SEGMENT_POINT_DIM);
  input_data_map["polygons"] = zeros(NUM_POLYGONS * POINTS_PER_POLYGON * (2 + POLYGON_TYPE_NUM));
  input_data_map["line_strings"] =
    zeros(NUM_LINE_STRINGS * POINTS_PER_LINE_STRING * (2 + LINE_STRING_TYPE_NUM));
  input_data_map["static_objects"] = zeros(NUM_STATIC_OBJECTS * STATIC_OBJECTS_SHAPE[2]);
  input_data_map["neighbor_agents_past"] =
    zeros(MAX_NUM_NEIGHBORS * INPUT_T_WITH_CURRENT * static_cast<int64_t>(AGENT_STATE_DIM));
  input_data_map["ego_agent_past"] = zeros(INPUT_T_WITH_CURRENT * POSE_DIM);
  input_data_map["goal_pose"] = zeros(POSE_DIM);

  // Ego sits at the origin facing along +x, and is 5 m long by 2 m wide.
  input_data_map["ego_current_state"] = zeros(EGO_CURRENT_STATE_SHAPE[1]);
  input_data_map["ego_shape"] = zeros(EGO_SHAPE_SHAPE[1]);
  for (int64_t b = 0; b < batch_size; ++b) {
    input_data_map["ego_current_state"][static_cast<size_t>(b * EGO_CURRENT_STATE_SHAPE[1]) + 2] =
      1.0f;
    const size_t shape_base = static_cast<size_t>(b * EGO_SHAPE_SHAPE[1]);
    input_data_map["ego_shape"][shape_base + 0] = 2.7f;  // wheel base, unused by the renderer
    input_data_map["ego_shape"][shape_base + 1] = 5.0f;  // length
    input_data_map["ego_shape"][shape_base + 2] = 2.0f;  // width
  }
  return input_data_map;
}

uint8_t pixel_at(
  const std::vector<uint8_t> & image, const int64_t scale, const int64_t channel, const int64_t row,
  const int64_t col)
{
  const size_t plane = (static_cast<size_t>(scale) * BEV_NUM_CHANNELS + channel) * PLANE_SIZE;
  return image[plane + static_cast<size_t>(row * BEV_IMAGE_SIZE + col)];
}

int64_t count_set(const std::vector<uint8_t> & image, const int64_t scale, const int64_t channel)
{
  const size_t plane = (static_cast<size_t>(scale) * BEV_NUM_CHANNELS + channel) * PLANE_SIZE;
  int64_t count = 0;
  for (size_t pixel = 0; pixel < PLANE_SIZE; ++pixel) {
    count += image[plane + pixel] != 0U ? 1 : 0;
  }
  return count;
}

/// Pixel a point lands on, recomputed independently of the renderer.
int64_t to_row(const double x_rel, const double extent_m)
{
  return CENTER_PIXEL - static_cast<int64_t>(std::lround(x_rel * BEV_IMAGE_SIZE / extent_m));
}

int64_t to_col(const double y_rel, const double extent_m)
{
  return CENTER_PIXEL - static_cast<int64_t>(std::lround(y_rel * BEV_IMAGE_SIZE / extent_m));
}

/// Write a straight lane segment running from x = 1 m to x = 20 m along y = 0, with boundaries
/// 1.75 m to either side. Points at the exact origin would read as padding, so the first sample
/// starts one meter ahead.
void fill_straight_lane(
  std::vector<float> & lanes, const int64_t traffic_light_state, const int64_t num_points)
{
  for (int64_t p = 0; p < num_points; ++p) {
    float * point = lanes.data() + p * SEGMENT_POINT_DIM;
    point[X] = static_cast<float>(p + 1);
    point[Y] = 0.0f;
    point[LB_X] = 0.0f;
    point[LB_Y] = 1.75f;
    point[RB_X] = 0.0f;
    point[RB_Y] = -1.75f;
    if (traffic_light_state >= 0) {
      point[traffic_light_state] = 1.0f;
    }
  }
}
}  // namespace

TEST(BevImageTest, ShapeAndBinaryValues)
{
  const std::vector<uint8_t> image = preprocess::create_bev_image(make_empty_input_data(1), 1);

  ASSERT_EQ(image.size(), SAMPLE_SIZE);
  for (const uint8_t value : image) {
    EXPECT_TRUE(value == 0U || value == 255U);
  }
}

TEST(BevImageTest, EgoBoxIsCenteredAndOnItsOwnChannel)
{
  const std::vector<uint8_t> image = preprocess::create_bev_image(make_empty_input_data(1), 1);

  for (const int64_t scale : {NEAR_SCALE, FAR_SCALE}) {
    EXPECT_EQ(pixel_at(image, scale, preprocess::BEV_CH_EGO, CENTER_PIXEL, CENTER_PIXEL), 255U);
    // A 5 m x 2 m box, so the near view (0.223 m per pixel) covers far more pixels than the far
    // one (0.893 m per pixel).
    EXPECT_GT(count_set(image, scale, preprocess::BEV_CH_EGO), 0);
  }
  EXPECT_GT(
    count_set(image, NEAR_SCALE, preprocess::BEV_CH_EGO),
    count_set(image, FAR_SCALE, preprocess::BEV_CH_EGO));

  // Nothing else is present in an empty scene, and the ego history collapses onto the origin.
  EXPECT_EQ(count_set(image, NEAR_SCALE, preprocess::BEV_CH_LANE_CENTERLINE), 0);
  EXPECT_EQ(count_set(image, NEAR_SCALE, preprocess::BEV_CH_VEHICLE), 0);
  EXPECT_EQ(count_set(image, NEAR_SCALE, preprocess::BEV_CH_GOAL_POSE), 0);
  EXPECT_EQ(count_set(image, NEAR_SCALE, preprocess::BEV_CH_TRAFFIC_LIGHT_GO), 0);
  EXPECT_EQ(count_set(image, NEAR_SCALE, preprocess::BEV_CH_TRAFFIC_LIGHT_CAUTION), 0);
  EXPECT_EQ(count_set(image, NEAR_SCALE, preprocess::BEV_CH_TRAFFIC_LIGHT_STOP), 0);
  EXPECT_EQ(count_set(image, NEAR_SCALE, preprocess::BEV_CH_TRAFFIC_LIGHT_UNKNOWN), 0);
}

TEST(BevImageTest, LaneCenterlineAndBoundariesLandOnTheirOwnChannels)
{
  preprocess::InputDataMap input_data_map = make_empty_input_data(1);
  fill_straight_lane(input_data_map["lanes"], -1, POINTS_PER_SEGMENT);

  const std::vector<uint8_t> image = preprocess::create_bev_image(input_data_map, 1);

  const int64_t row = to_row(10.0, BEV_NEAR_EXTENT_M);
  EXPECT_EQ(
    pixel_at(
      image, NEAR_SCALE, preprocess::BEV_CH_LANE_CENTERLINE, row, to_col(0.0, BEV_NEAR_EXTENT_M)),
    255U);
  // Ego-left maps to image-left, so the left boundary sits at the smaller column.
  const int64_t left_col = to_col(1.75, BEV_NEAR_EXTENT_M);
  const int64_t right_col = to_col(-1.75, BEV_NEAR_EXTENT_M);
  EXPECT_LT(left_col, right_col);
  EXPECT_EQ(pixel_at(image, NEAR_SCALE, preprocess::BEV_CH_LANE_BOUNDARY, row, left_col), 255U);
  EXPECT_EQ(pixel_at(image, NEAR_SCALE, preprocess::BEV_CH_LANE_BOUNDARY, row, right_col), 255U);
  // The boundaries never bleed into the centerline channel and vice versa.
  EXPECT_EQ(pixel_at(image, NEAR_SCALE, preprocess::BEV_CH_LANE_CENTERLINE, row, left_col), 0U);
  EXPECT_EQ(
    pixel_at(
      image, NEAR_SCALE, preprocess::BEV_CH_LANE_BOUNDARY, row, to_col(0.0, BEV_NEAR_EXTENT_M)),
    0U);

  // fill_straight_lane left every state slot zero, so no light plane is touched.
  EXPECT_EQ(count_set(image, NEAR_SCALE, preprocess::BEV_CH_TRAFFIC_LIGHT_GO), 0);
  EXPECT_EQ(count_set(image, NEAR_SCALE, preprocess::BEV_CH_TRAFFIC_LIGHT_CAUTION), 0);
  EXPECT_EQ(count_set(image, NEAR_SCALE, preprocess::BEV_CH_TRAFFIC_LIGHT_STOP), 0);
  EXPECT_EQ(count_set(image, NEAR_SCALE, preprocess::BEV_CH_TRAFFIC_LIGHT_UNKNOWN), 0);
}

TEST(BevImageTest, PolylineStopsAtThePadding)
{
  preprocess::InputDataMap input_data_map = make_empty_input_data(1);
  // Only the first five samples are real; the rest of the segment is zero padding.
  fill_straight_lane(input_data_map["lanes"], -1, 5);

  const std::vector<uint8_t> image = preprocess::create_bev_image(input_data_map, 1);

  EXPECT_EQ(
    pixel_at(
      image, NEAR_SCALE, preprocess::BEV_CH_LANE_CENTERLINE, to_row(5.0, BEV_NEAR_EXTENT_M),
      to_col(0.0, BEV_NEAR_EXTENT_M)),
    255U);
  EXPECT_EQ(
    pixel_at(
      image, NEAR_SCALE, preprocess::BEV_CH_LANE_CENTERLINE, to_row(10.0, BEV_NEAR_EXTENT_M),
      to_col(0.0, BEV_NEAR_EXTENT_M)),
    0U);
}

TEST(BevImageTest, EveryTrafficLightStateGetsItsOwnChannel)
{
  // A lane with no light at all is drawn on none of the four planes; that silence is what
  // distinguishes it from a lane whose light state never arrived.
  const std::vector<std::pair<int64_t, int64_t>> channel_of_state = {
    {TRAFFIC_LIGHT_GREEN, preprocess::BEV_CH_TRAFFIC_LIGHT_GO},
    {TRAFFIC_LIGHT_YELLOW, preprocess::BEV_CH_TRAFFIC_LIGHT_CAUTION},
    {TRAFFIC_LIGHT_RED, preprocess::BEV_CH_TRAFFIC_LIGHT_STOP},
    {TRAFFIC_LIGHT_WHITE, preprocess::BEV_CH_TRAFFIC_LIGHT_UNKNOWN},
    {TRAFFIC_LIGHT_NO_TRAFFIC_LIGHT, -1},
  };

  for (const auto & [light_index, expected_channel] : channel_of_state) {
    preprocess::InputDataMap input_data_map = make_empty_input_data(1);
    fill_straight_lane(input_data_map["lanes"], light_index, POINTS_PER_SEGMENT);

    const std::vector<uint8_t> image = preprocess::create_bev_image(input_data_map, 1);

    for (const auto & [other_state, channel] : channel_of_state) {
      if (channel < 0) {
        continue;
      }
      const int64_t drawn = count_set(image, NEAR_SCALE, channel);
      if (channel == expected_channel) {
        EXPECT_GT(drawn, 0) << "state " << light_index << " should light channel " << channel;
      } else {
        EXPECT_EQ(drawn, 0) << "state " << light_index << " must not touch channel " << channel
                            << " (belonging to state " << other_state << ")";
      }
    }
  }
}

TEST(BevImageTest, YellowIsNotInterchangeableWithRed)
{
  preprocess::InputDataMap yellow = make_empty_input_data(1);
  fill_straight_lane(yellow["lanes"], TRAFFIC_LIGHT_YELLOW, POINTS_PER_SEGMENT);
  preprocess::InputDataMap red = make_empty_input_data(1);
  fill_straight_lane(red["lanes"], TRAFFIC_LIGHT_RED, POINTS_PER_SEGMENT);

  EXPECT_NE(preprocess::create_bev_image(yellow, 1), preprocess::create_bev_image(red, 1));
}

TEST(BevImageTest, NeighborBoxAndTrackUseTheClassChannel)
{
  preprocess::InputDataMap input_data_map = make_empty_input_data(1);
  std::vector<float> & neighbors = input_data_map["neighbor_agents_past"];
  constexpr int64_t STATE_DIM = static_cast<int64_t>(AGENT_STATE_DIM);

  // One bicycle driving from 5 m to 10 m ahead, 4 m to the left of ego.
  for (int64_t t = 0; t < INPUT_T_WITH_CURRENT; ++t) {
    float * state = neighbors.data() + t * STATE_DIM;
    state[X] = 5.0f + static_cast<float>(t) * (5.0f / static_cast<float>(INPUT_T));
    state[Y] = 4.0f;
    state[2] = 1.0f;   // cos yaw
    state[3] = 0.0f;   // sin yaw
    state[6] = 0.6f;   // width
    state[7] = 1.8f;   // length
    state[10] = 1.0f;  // bicycle one-hot
  }

  const std::vector<uint8_t> image = preprocess::create_bev_image(input_data_map, 1);

  EXPECT_EQ(
    pixel_at(
      image, NEAR_SCALE, preprocess::BEV_CH_BICYCLE, to_row(10.0, BEV_NEAR_EXTENT_M),
      to_col(4.0, BEV_NEAR_EXTENT_M)),
    255U);
  EXPECT_EQ(count_set(image, NEAR_SCALE, preprocess::BEV_CH_VEHICLE), 0);
  EXPECT_EQ(count_set(image, NEAR_SCALE, preprocess::BEV_CH_PEDESTRIAN), 0);
  // The track runs from 5 m to 10 m, so it is drawn well behind the current box.
  EXPECT_EQ(
    pixel_at(
      image, NEAR_SCALE, preprocess::BEV_CH_NEIGHBOR_HISTORY, to_row(5.0, BEV_NEAR_EXTENT_M),
      to_col(4.0, BEV_NEAR_EXTENT_M)),
    255U);
}

TEST(BevImageTest, ScaleChangesTheMetersPerPixel)
{
  preprocess::InputDataMap input_data_map = make_empty_input_data(1);
  fill_straight_lane(input_data_map["lanes"], -1, POINTS_PER_SEGMENT);

  const std::vector<uint8_t> image = preprocess::create_bev_image(input_data_map, 1);

  // 20 m ahead is inside both views, but four times closer to the center in the far one.
  EXPECT_EQ(
    pixel_at(
      image, NEAR_SCALE, preprocess::BEV_CH_LANE_CENTERLINE, to_row(20.0, BEV_NEAR_EXTENT_M),
      to_col(0.0, BEV_NEAR_EXTENT_M)),
    255U);
  EXPECT_EQ(
    pixel_at(
      image, FAR_SCALE, preprocess::BEV_CH_LANE_CENTERLINE, to_row(20.0, BEV_FAR_EXTENT_M),
      to_col(0.0, BEV_FAR_EXTENT_M)),
    255U);
  EXPECT_GT(
    count_set(image, NEAR_SCALE, preprocess::BEV_CH_LANE_CENTERLINE),
    count_set(image, FAR_SCALE, preprocess::BEV_CH_LANE_CENTERLINE));
}

TEST(BevImageTest, GoalPoseBeyondTheFarViewDrawsNothing)
{
  preprocess::InputDataMap input_data_map = make_empty_input_data(1);
  std::vector<float> & goal_pose = input_data_map["goal_pose"];
  goal_pose[X] = 5000.0f;
  goal_pose[Y] = 0.0f;
  goal_pose[2] = 1.0f;

  const std::vector<uint8_t> image = preprocess::create_bev_image(input_data_map, 1);

  EXPECT_EQ(count_set(image, NEAR_SCALE, preprocess::BEV_CH_GOAL_POSE), 0);
  EXPECT_EQ(count_set(image, FAR_SCALE, preprocess::BEV_CH_GOAL_POSE), 0);
}

TEST(BevImageTest, GoalPoseInsideTheFarViewIsMarked)
{
  preprocess::InputDataMap input_data_map = make_empty_input_data(1);
  std::vector<float> & goal_pose = input_data_map["goal_pose"];
  goal_pose[X] = 80.0f;
  goal_pose[Y] = 0.0f;
  goal_pose[2] = 1.0f;

  const std::vector<uint8_t> image = preprocess::create_bev_image(input_data_map, 1);

  // 80 m ahead is outside the 50 m view but inside the 200 m one.
  EXPECT_EQ(count_set(image, NEAR_SCALE, preprocess::BEV_CH_GOAL_POSE), 0);
  EXPECT_EQ(
    pixel_at(
      image, FAR_SCALE, preprocess::BEV_CH_GOAL_POSE, to_row(80.0, BEV_FAR_EXTENT_M),
      to_col(0.0, BEV_FAR_EXTENT_M)),
    255U);
}

TEST(BevImageTest, EveryBatchEntryCarriesTheSameRaster)
{
  constexpr int64_t BATCH_SIZE = 3;
  preprocess::InputDataMap input_data_map = make_empty_input_data(BATCH_SIZE);
  fill_straight_lane(input_data_map["lanes"], -1, POINTS_PER_SEGMENT);

  const std::vector<uint8_t> image = preprocess::create_bev_image(input_data_map, BATCH_SIZE);

  ASSERT_EQ(image.size(), SAMPLE_SIZE * BATCH_SIZE);
  for (int64_t b = 1; b < BATCH_SIZE; ++b) {
    EXPECT_TRUE(
      std::equal(
        image.begin(), image.begin() + static_cast<std::ptrdiff_t>(SAMPLE_SIZE),
        image.begin() + static_cast<std::ptrdiff_t>(static_cast<size_t>(b) * SAMPLE_SIZE)))
      << "batch entry " << b << " differs";
  }
}

TEST(BevImageTest, ColorizeMarksTheEgoInWhite)
{
  const std::vector<uint8_t> image = preprocess::create_bev_image(make_empty_input_data(1), 1);
  const std::vector<uint8_t> preview = preprocess::colorize_bev_image(image, NEAR_SCALE);

  ASSERT_EQ(preview.size(), PLANE_SIZE * 3);
  const size_t center = static_cast<size_t>(CENTER_PIXEL * BEV_IMAGE_SIZE + CENTER_PIXEL) * 3;
  EXPECT_EQ(preview[center + 0], 255U);
  EXPECT_EQ(preview[center + 1], 255U);
  EXPECT_EQ(preview[center + 2], 255U);
  // A corner of an otherwise empty scene stays background.
  EXPECT_EQ(preview[0], 0U);
}

}  // namespace autoware::diffusion_planner::test
