// Copyright 2026 TIER IV, inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__DECELERATION_AWARE_PATH_CUT_VRU_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__DECELERATION_AWARE_PATH_CUT_VRU_HPP_

#include "autoware/map_based_prediction/path_cut/path_cut_utils.hpp"
#include "autoware/map_based_prediction/path_generator/path_generator.hpp"

#include <autoware_perception_msgs/msg/tracked_object.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{

// Deceleration-aware path cut for VRU: trims a predicted path at a map boundary only when the
// object can stop before it under its class max deceleration (crosswalk crossings are exempted).
class DecelerationAwarePathCutVruModule
{
public:
  DecelerationAwarePathCutVruModule() = default;

  void build_from_map(
    std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr,
    const std::vector<std::string> & boundary_types);

  [[nodiscard]] PredictedPath cut_path_at_boundary(
    const PredictedPath & predicted_path,
    const autoware_perception_msgs::msg::TrackedObject & object,
    const path_cut::MaxDecelerationParams & max_decel_params) const;

private:
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_{nullptr};
  lanelet::LaneletMapUPtr boundary_layer_{nullptr};
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__DECELERATION_AWARE_PATH_CUT_VRU_HPP_
