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

#include "autoware/map_based_prediction/predictor_vru/vegetation.hpp"

#include "autoware/map_based_prediction/path_cut/footprint_path_cut.hpp"
#include "autoware/map_based_prediction/path_cut/path_cut_utils.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{

namespace
{
std::vector<autoware_utils_geometry::Polygon2d> collect_candidate_vegetation_polygons(
  const lanelet::LaneletMap & vegetation_layer, const std::vector<PredictedPath> & predicted_paths,
  const autoware_perception_msgs::msg::Shape & object_shape)
{
  const auto candidates = vegetation_layer.polygonLayer.search(
    path_cut::footprint_search_bbox(predicted_paths, object_shape));
  std::vector<autoware_utils_geometry::Polygon2d> vegetation_polygons_2d;
  vegetation_polygons_2d.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    autoware_utils_geometry::Polygon2d polygon;
    boost::geometry::convert(lanelet::utils::to2D(candidate.basicPolygon()), polygon);
    boost::geometry::correct(polygon);
    vegetation_polygons_2d.push_back(polygon);
  }
  return vegetation_polygons_2d;
}
}  // namespace

void VegetationModule::build_from_map(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr)
{
  if (!lanelet_map_ptr) {
    vegetation_layer_ = nullptr;
    return;
  }

  lanelet::Polygons3d vegetations;
  for (const auto & polygon : lanelet_map_ptr->polygonLayer) {
    const std::string type = polygon.attributeOr(lanelet::AttributeName::Type, "none");
    const std::string subtype = polygon.attributeOr(lanelet::AttributeName::Subtype, "none");
    if (type == "area" && subtype == "vegetation") {
      vegetations.emplace_back(
        std::const_pointer_cast<lanelet::LineStringData>(polygon.constData()));
    }
  }
  vegetation_layer_ = lanelet::utils::createMap(vegetations);
}

std::vector<PredictedPath> VegetationModule::cut_paths_crossing_vegetation(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object) const
{
  std::vector<PredictedPath> cut_paths = predicted_object.kinematics.predicted_paths;
  if (!path_cut::has_required_info(predicted_object) || !vegetation_layer_) {
    return cut_paths;
  }

  const autoware_perception_msgs::msg::Shape & object_shape = predicted_object.shape;
  const std::vector<autoware_utils_geometry::Polygon2d> candidate_polygons =
    collect_candidate_vegetation_polygons(*vegetation_layer_, cut_paths, object_shape);

  for (PredictedPath & predicted_path : cut_paths) {
    const std::optional<size_t> crossing_index =
      path_cut::find_footprint_crossing_index(predicted_path, object_shape, candidate_polygons);
    if (crossing_index) {
      const size_t last_kept_index = std::max<size_t>(*crossing_index, 1UL) - 1UL;
      predicted_path = path_cut::force_cut_at_index(predicted_path, last_kept_index);
    }
  }
  return cut_paths;
}
}  // namespace autoware::map_based_prediction
