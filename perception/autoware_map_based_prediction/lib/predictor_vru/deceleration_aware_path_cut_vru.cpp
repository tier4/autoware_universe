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

#include "autoware/map_based_prediction/predictor_vru/deceleration_aware_path_cut_vru.hpp"

#include "autoware/map_based_prediction/path_cut/path_cut_utils.hpp"

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{

namespace
{
bool is_point_near_crosswalk(const lanelet::LaneletMap & map, const lanelet::BasicPoint2d & point)
{
  // Tolerance so a crossing that lands just outside a crosswalk polygon still counts as inside it:
  // the boundary linestring and the crosswalk boundary rarely share exact vertices.
  constexpr double margin = 1.0;
  const lanelet::BoundingBox2d search_box{
    point - lanelet::BasicPoint2d{margin, margin}, point + lanelet::BasicPoint2d{margin, margin}};
  for (const auto & candidate : map.laneletLayer.search(search_box)) {
    const std::string subtype = candidate.attributeOr(lanelet::AttributeName::Subtype, "none");
    if (
      subtype != lanelet::AttributeValueString::Crosswalk &&
      subtype != lanelet::AttributeValueString::Walkway) {
      continue;
    }
    if (
      lanelet::geometry::inside(candidate, point) ||
      boost::geometry::distance(point, candidate.polygon2d()) <= margin) {
      return true;
    }
  }
  return false;
}

template <typename Candidates>
std::optional<size_t> find_boundary_crossing_index(
  const lanelet::BasicLineString2d & path_ls, const Candidates & candidates)
{
  for (size_t seg = 0; seg + 1 < path_ls.size(); ++seg) {
    const lanelet::BasicLineString2d segment(
      lanelet::BasicPoints2d{path_ls[seg], path_ls[seg + 1]});
    const bool crosses = std::any_of(candidates.begin(), candidates.end(), [&](const auto & border) {
      return boost::geometry::intersects(segment, lanelet::utils::to2D(border).basicLineString());
    });
    if (crosses) {
      return seg;
    }
  }
  return std::nullopt;
}

double arc_length_to_index(const lanelet::BasicLineString2d & path_ls, const size_t index)
{
  double distance = 0.0;
  for (size_t seg = 0; seg < index; ++seg) {
    distance += (path_ls[seg + 1] - path_ls[seg]).norm();
  }
  return distance;
}
}  // namespace

void DecelerationAwarePathCutVruModule::build_from_map(
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr,
  const std::vector<std::string> & boundary_types)
{
  lanelet_map_ptr_ = lanelet_map_ptr;
  boundary_layer_ = nullptr;
  if (!lanelet_map_ptr_) {
    return;
  }

  lanelet::LineStrings3d boundaries;
  for (const auto & linestring : lanelet_map_ptr_->lineStringLayer) {
    const std::string type = linestring.attributeOr(lanelet::AttributeName::Type, "none");
    if (std::find(boundary_types.begin(), boundary_types.end(), type) != boundary_types.end()) {
      boundaries.emplace_back(
        std::const_pointer_cast<lanelet::LineStringData>(linestring.constData()));
    }
  }
  if (!boundaries.empty()) {
    boundary_layer_ = lanelet::utils::createMap(boundaries);
  }
}

PredictedPath DecelerationAwarePathCutVruModule::cut_path_at_boundary(
  const PredictedPath & predicted_path, const autoware_perception_msgs::msg::TrackedObject & object,
  const path_cut::MaxDecelerationParams & max_decel_params) const
{
  const auto & poses = predicted_path.path;
  if (!boundary_layer_ || poses.size() < 2) {
    return predicted_path;
  }

  const lanelet::BasicLineString2d path_ls = path_cut::to_basic_line_string(poses);
  const auto candidates =
    boundary_layer_->lineStringLayer.search(lanelet::geometry::boundingBox2d(path_ls));

  const std::optional<size_t> crossing_index = find_boundary_crossing_index(path_ls, candidates);
  if (!crossing_index) {
    return predicted_path;
  }

  const lanelet::BasicPoint2d crossing =
    0.5 * (path_ls[*crossing_index] + path_ls[*crossing_index + 1]);
  if (lanelet_map_ptr_ && is_point_near_crosswalk(*lanelet_map_ptr_, crossing)) {
    return predicted_path;
  }

  const double distance_to_cut = arc_length_to_index(path_ls, *crossing_index);
  const auto & v = object.kinematics.twist_with_covariance.twist.linear;
  const double max_deceleration = path_cut::max_deceleration_for_label(
    max_decel_params, autoware::object_recognition_utils::getHighestProbLabel(object.classification));
  if (!path_cut::can_stop_before_the_line(distance_to_cut, std::hypot(v.x, v.y), max_deceleration)) {
    return predicted_path;
  }
  return path_cut::force_cut_at_index(predicted_path, *crossing_index);
}

}  // namespace autoware::map_based_prediction
