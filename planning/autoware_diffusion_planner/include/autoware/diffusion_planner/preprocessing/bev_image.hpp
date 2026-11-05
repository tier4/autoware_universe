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

#ifndef AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__BEV_IMAGE_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__BEV_IMAGE_HPP_

#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"

#include <cstdint>
#include <vector>

namespace autoware::diffusion_planner::preprocess
{

// Semantic plane each element is drawn onto. Every plane is its own binary mask, so overlapping
// elements never hide each other. The order is part of the trained weights: the ResNet's first
// convolution has one input plane per entry, in exactly this order.
enum BevChannel {
  BEV_CH_LANE_BOUNDARY = 0,
  BEV_CH_LANE_CENTERLINE = 1,
  BEV_CH_ROUTE = 2,
  BEV_CH_TRAFFIC_LIGHT_GO = 3,       // green
  BEV_CH_TRAFFIC_LIGHT_CAUTION = 4,  // yellow
  BEV_CH_TRAFFIC_LIGHT_STOP = 5,     // red
  BEV_CH_TRAFFIC_LIGHT_UNKNOWN = 6,  // has a light, colour unknown or never received
  BEV_CH_POLYGON = 7,
  BEV_CH_LINE_STRING = 8,
  BEV_CH_STATIC_OBJECT = 9,
  BEV_CH_GOAL_POSE = 10,
  BEV_CH_VEHICLE = 11,
  BEV_CH_PEDESTRIAN = 12,
  BEV_CH_BICYCLE = 13,
  BEV_CH_EGO = 14,
  BEV_CH_NEIGHBOR_HISTORY = 15,
  BEV_CH_EGO_HISTORY = 16,
};

/**
 * @brief Rasterize the vector scene of one frame into the BEV stack the image encoder consumes.
 *
 * Mirrors diffusion_planner/utils/render_bev.py: one image per spatial scale
 * (BEV_VIEW_EXTENTS_M), each a stack of BEV_NUM_CHANNELS binary masks centered on the current ego
 * pose with the ego heading pointing up. An extent is the full side length of a view, so the
 * 50 m scale reaches +/-25 m around the ego. Motion is carried by the history channels - an
 * agent's past track is drawn as a polyline ending at that agent's current bounding box - so no
 * per-timestep raster is needed.
 *
 * @param input_data_map Un-normalized model inputs, i.e. meters, exactly as create_input_data
 *                       returns them. Every key read here is batch-replicated from the same
 *                       single frame, so only the first batch entry is rasterized.
 * @param batch_size     Number of identical copies to emit, matching the other input tensors.
 * @return Row-major [batch_size, BEV_NUM_SCALES, BEV_NUM_CHANNELS, BEV_IMAGE_SIZE,
 *         BEV_IMAGE_SIZE] bytes, each either 0 or 255.
 */
std::vector<uint8_t> create_bev_image(const InputDataMap & input_data_map, int64_t batch_size);

/**
 * @brief Composite one scale of a rendered stack into a BGR preview, for debugging only.
 *
 * Mirrors render_bev.py::colorize, including the draw order that keeps the agents visible on top
 * of the map.
 *
 * @param bev_image Output of create_bev_image; only its first batch entry is read.
 * @param scale     Index into BEV_VIEW_EXTENTS_M.
 * @return Row-major [BEV_IMAGE_SIZE, BEV_IMAGE_SIZE, 3] BGR bytes.
 */
std::vector<uint8_t> colorize_bev_image(const std::vector<uint8_t> & bev_image, int64_t scale);

}  // namespace autoware::diffusion_planner::preprocess

#endif  // AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__BEV_IMAGE_HPP_
