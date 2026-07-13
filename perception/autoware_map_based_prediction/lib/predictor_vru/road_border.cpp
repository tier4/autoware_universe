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

#include "autoware/map_based_prediction/predictor_vru/road_border.hpp"

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
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{

namespace
{
bool point_near_crosswalk(const lanelet::LaneletMap & map, const lanelet::BasicPoint2d & point)
{
  // Tolerance so a crossing that lands just outside a crosswalk polygon still counts as inside it:
  // the road_border linestring and the crosswalk boundary rarely share exact vertices.
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
}  // namespace

void RoadBorderModule::build_from_map(
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr,
  const std::vector<std::string> & boundary_types)
{
  lanelet_map_ptr_ = lanelet_map_ptr;
  road_border_layer_ = nullptr;
  if (!lanelet_map_ptr_) {
    return;
  }

  lanelet::LineStrings3d borders;
  for (const auto & linestring : lanelet_map_ptr_->lineStringLayer) {
    const std::string type = linestring.attributeOr(lanelet::AttributeName::Type, "none");
    if (std::find(boundary_types.begin(), boundary_types.end(), type) != boundary_types.end()) {
      borders.emplace_back(
        std::const_pointer_cast<lanelet::LineStringData>(linestring.constData()));
    }
  }
  if (!borders.empty()) {
    road_border_layer_ = lanelet::utils::createMap(borders);
  }
}

PredictedPath RoadBorderModule::cut_path_at_road_border(
  const PredictedPath & predicted_path,
  const autoware_perception_msgs::msg::TrackedObject & object,
  const path_cut::MaxDecelerationParams & max_decel_params) const
{
  const auto & poses = predicted_path.path;
  if (!road_border_layer_ || poses.size() < 2) {
    return predicted_path;
  }

  lanelet::BasicLineString2d path_ls;
  path_ls.reserve(poses.size());
  for (const auto & pose : poses) {
    path_ls.emplace_back(pose.position.x, pose.position.y);
  }
  const auto candidates =
    road_border_layer_->lineStringLayer.search(lanelet::geometry::boundingBox2d(path_ls));

  double distance_to_cut = 0.0;  // arc length up to pose[seg], where the path is truncated
  for (size_t seg = 0; seg + 1 < poses.size(); ++seg) {
    const lanelet::BasicLineString2d segment(lanelet::BasicPoints2d{path_ls[seg], path_ls[seg + 1]});
    const bool crosses = std::any_of(
      candidates.begin(), candidates.end(), [&](const auto & border) {
        return boost::geometry::intersects(segment, lanelet::utils::to2D(border).basicLineString());
      });

    if (crosses) {
      const lanelet::BasicPoint2d crossing = 0.5 * (path_ls[seg] + path_ls[seg + 1]);
      if (lanelet_map_ptr_ && point_near_crosswalk(*lanelet_map_ptr_, crossing)) {
        return predicted_path;
      }
      const auto & v = object.kinematics.twist_with_covariance.twist.linear;
      const double max_deceleration = path_cut::max_deceleration_for_label(
        max_decel_params, autoware::object_recognition_utils::getHighestProbLabel(
                            object.classification));
      if (!path_cut::can_stop_before_the_line(distance_to_cut, std::hypot(v.x, v.y), max_deceleration)) {
        return predicted_path;
      }
      return path_cut::force_cut_at_index(predicted_path, seg);
    }
    distance_to_cut += (path_ls[seg + 1] - path_ls[seg]).norm();
  }
  return predicted_path;
}

}  // namespace autoware::map_based_prediction
