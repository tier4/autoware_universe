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

#include "autoware/map_based_prediction/path_cut/path_cut_utils.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <optional>
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{

namespace
{
std::vector<autoware_utils_geometry::Polygon2d> collect_candidate_vegetation_polygons(
  const lanelet::LaneletMap & vegetation_layer, const std::vector<PredictedPath> & predicted_paths)
{
  lanelet::BoundingBox2d search_bbox;
  for (const auto & predicted_path : predicted_paths) {
    for (const auto & pose : predicted_path.path) {
      search_bbox.extend(lanelet::BasicPoint2d(pose.position.x, pose.position.y));
    }
  }

  const auto candidates = vegetation_layer.polygonLayer.search(search_bbox);
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

std::optional<size_t> find_vegetation_crossing_index(
  const PredictedPath & predicted_path,
  const std::vector<autoware_utils_geometry::Polygon2d> & vegetation_polygons_2d)
{
  const auto & path = predicted_path.path;
  if (path.size() < 2 || vegetation_polygons_2d.empty()) {
    return std::nullopt;
  }
  // Judge on the predicted-path segment (the polyline between consecutive path points) rather than
  // the object footprint: the discretized footprint polygon can miss a thin/oblique crossing with
  // the vegetation area.
  for (auto i = 0UL; i + 1 < path.size(); ++i) {
    autoware_utils_geometry::LineString2d path_segment;
    path_segment.emplace_back(path.at(i).position.x, path.at(i).position.y);
    path_segment.emplace_back(path.at(i + 1).position.x, path.at(i + 1).position.y);
    for (const auto & vegetation_polygon : vegetation_polygons_2d) {
      if (boost::geometry::intersects(path_segment, vegetation_polygon)) {
        return i;
      }
    }
  }
  return std::nullopt;
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
  if (cut_paths.empty() || !vegetation_layer_) {
    return cut_paths;
  }

  const std::vector<autoware_utils_geometry::Polygon2d> candidate_polygons =
    collect_candidate_vegetation_polygons(*vegetation_layer_, cut_paths);

  for (PredictedPath & predicted_path : cut_paths) {
    const std::optional<size_t> crossing_index =
      find_vegetation_crossing_index(predicted_path, candidate_polygons);
    if (crossing_index) {
      // Keep the last pose before the segment enters the vegetation area.
      predicted_path = path_cut::force_cut_at_index(predicted_path, crossing_index.value());
    }
  }
  return cut_paths;
}
}  // namespace autoware::map_based_prediction
