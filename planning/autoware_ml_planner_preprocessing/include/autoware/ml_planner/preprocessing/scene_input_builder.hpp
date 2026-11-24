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

#ifndef AUTOWARE__ML_PLANNER__PREPROCESSING__SCENE_INPUT_BUILDER_HPP_
#define AUTOWARE__ML_PLANNER__PREPROCESSING__SCENE_INPUT_BUILDER_HPP_

#include "autoware/ml_planner/preprocessing/items/map.hpp"
#include "autoware/ml_planner/preprocessing/preprocessing_utils.hpp"

#include <t4_scene/scene.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::ml_planner::preprocess
{

enum class TrafficLightFuture { Infer, Recorded };

struct SceneInputBuilderOutput
{
  TensorMap tensors;
  std::vector<std::string> selected_track_ids;  ///< In neighbor tensor row order.
  Eigen::Matrix4d ego_to_map;
};

using SceneInputBuilderResult = tl::expected<SceneInputBuilderOutput, std::string>;

/// Build immutable lane features once for each resolved Lanelet2 map.
std::unique_ptr<LaneSegmentContext> build_map_context(
  const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map_ptr);

/// Convert one 10 Hz T4Scene window at current_index. Map context must use scene.map's
/// native geometry; the Scene's map registration is applied for each window.
/// Produces raw model features for one frame, with no batch dimension. Call normalize_input_data for inference
/// and supply initial_noise separately. Recorded mode reads only future traffic lights.
SceneInputBuilderResult build_scene_input(
  const t4scene::T4Scene & scene, const LaneSegmentContext & map_context,
  TrafficLightFuture traffic_light_future = TrafficLightFuture::Infer,
  double traffic_light_timeout_s = 0.2);

}  // namespace autoware::ml_planner::preprocess

#endif  // AUTOWARE__ML_PLANNER__PREPROCESSING__SCENE_INPUT_BUILDER_HPP_
