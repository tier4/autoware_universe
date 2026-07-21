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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PATH_CUT__FOOTPRINT_PATH_CUT_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PATH_CUT__FOOTPRINT_PATH_CUT_HPP_

#include "autoware/map_based_prediction/path_cut/path_cut_utils.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/BoundingBox.h>

#include <optional>
#include <vector>

namespace autoware::map_based_prediction::path_cut
{

double calc_footprint_search_margin(const autoware_perception_msgs::msg::Shape & shape);

bool shape_has_footprint(const autoware_perception_msgs::msg::Shape & shape);

bool has_required_info(const autoware_perception_msgs::msg::PredictedObject & predicted_object);

lanelet::BoundingBox2d get_bbox_contain_path_with_footprint(
  const std::vector<PredictedPath> & predicted_paths,
  const autoware_perception_msgs::msg::Shape & object_shape);
lanelet::BoundingBox2d get_bbox_contain_path_with_footprint(
  const PredictedPath & predicted_path, const autoware_perception_msgs::msg::Shape & object_shape);

std::optional<size_t> find_footprint_crossing_index(
  const PredictedPath & predicted_path, const autoware_perception_msgs::msg::Shape & object_shape,
  const std::vector<autoware_utils_geometry::LineString2d> & linestrings_2d);

std::optional<size_t> find_footprint_crossing_index(
  const PredictedPath & predicted_path, const autoware_perception_msgs::msg::Shape & object_shape,
  const lanelet::ConstLineStrings3d & linestrings);

std::optional<size_t> find_footprint_crossing_index(
  const PredictedPath & predicted_path, const autoware_perception_msgs::msg::Shape & object_shape,
  const std::vector<autoware_utils_geometry::Polygon2d> & polygons_2d);

}  // namespace autoware::map_based_prediction::path_cut

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PATH_CUT__FOOTPRINT_PATH_CUT_HPP_
