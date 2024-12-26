// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifndef COLLISIONS_FILTERING_HPP_
#define COLLISIONS_FILTERING_HPP_

#include "parameters.hpp"

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>

#include <boost/geometry/algorithms/correct.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>

namespace autoware::motion_velocity_planner::run_out
{
/// @brief prepare polygons where collisions should be ignored
inline universe_utils::MultiPolygon2d prepare_ignored_polygons(
  const lanelet::LaneletMapPtr & map_ptr, const Parameters & params)
{
  universe_utils::MultiPolygon2d ignored_polygons;
  if (params.objects_ignore_collisions_on_crosswalk) {
    // TODO(Maxime): only retrieve lanelets crossed by the ego trajectory footprint
    for (const auto & ll : map_ptr->laneletLayer) {
      if (
        std::strcmp(
          ll.attributeOr(lanelet::AttributeName::Subtype, ""),
          lanelet::AttributeValueString::Crosswalk) == 0) {
        universe_utils::Polygon2d polygon;
        for (const auto & p : ll.polygon2d().basicPolygon()) {
          polygon.outer().emplace_back(p.x(), p.y());
        }
        boost::geometry::correct(polygon);
        ignored_polygons.push_back(polygon);
      }
    }
  }
  return ignored_polygons;
}
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // COLLISIONS_FILTERING_HPP_
