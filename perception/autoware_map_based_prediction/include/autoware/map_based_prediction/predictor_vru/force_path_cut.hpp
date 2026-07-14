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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__FORCE_PATH_CUT_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__FORCE_PATH_CUT_HPP_

#include "autoware/map_based_prediction/path_generator/path_generator.hpp"

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{

/// Unconditional (deceleration-agnostic) path cut where a VRU path crosses a map boundary.
/// Boundary types are matched against both polygon subtypes (e.g. vegetation area) and
/// linestring types (e.g. guard_rail), so a single config list covers both geometries.
class ForcePathCutModule
{
public:
  ForcePathCutModule() = default;

  /// @pre lanelet_map_ptr is non-null when building from a map; nullptr clears the layers.
  void build_from_map(
    std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr,
    const std::vector<std::string> & boundary_types);

  /// Return the object's predicted paths trimmed where they cross a boundary.
  [[nodiscard]] std::vector<PredictedPath> cut_paths_crossing_boundary(
    const autoware_perception_msgs::msg::PredictedObject & predicted_object) const;

private:
  lanelet::LaneletMapUPtr polygon_layer_{nullptr};
  lanelet::LaneletMapUPtr linestring_layer_{nullptr};
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__FORCE_PATH_CUT_HPP_
