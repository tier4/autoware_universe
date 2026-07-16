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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__ROAD_BORDER_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__ROAD_BORDER_HPP_

#include "autoware/map_based_prediction/path_cut/path_cut_utils.hpp"
#include "autoware/map_based_prediction/path_generator/path_generator.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/tracked_object.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>

namespace autoware::map_based_prediction
{

class RoadBorderModule
{
public:
  RoadBorderModule() = default;

  void build_from_map(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr);

  [[nodiscard]] PredictedPath cut_path_at_road_border(
    const PredictedPath & predicted_path,
    const autoware_perception_msgs::msg::TrackedObject & object,
    const path_cut::MaxDecelerationParams & max_decel_params) const;

private:
  // road_border linestrings with the crosswalk sections clipped out. synthesized geometry: ids are
  // freshly generated and z is dropped, so this is not a view of the original map.
  lanelet::LaneletMapConstUPtr cut_road_border_map_;
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__ROAD_BORDER_HPP_
