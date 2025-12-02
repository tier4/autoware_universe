// Copyright 2025 TIER IV, Inc.
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

#include "roundabout_lanelets.hpp"

#include <autoware/behavior_velocity_intersection_module/util.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <boost/geometry/algorithms/intersection.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <queue>
#include <string>
#include <unordered_set>

namespace autoware::behavior_velocity_planner
{

namespace
{
/**
 * @brief Compute the geometric intersection of two lanelets and return as CompoundPolygon3d
 * @param lanelet1 First lanelet
 * @param lanelet2 Second lanelet
 * @return The intersection polygon, or nullopt if no intersection
 */
std::optional<lanelet::CompoundPolygon3d> computeIntersectionPolygon(
  const lanelet::ConstLanelet & lanelet1, const lanelet::ConstLanelet & lanelet2)
{
  // Get 2D basic polygons
  const auto poly1 = lanelet1.polygon2d().basicPolygon();
  const auto poly2 = lanelet2.polygon2d().basicPolygon();

  // Compute intersection using boost::geometry
  std::vector<lanelet::BasicPolygon2d> intersection_result;
  boost::geometry::intersection(poly1, poly2, intersection_result);

  if (intersection_result.empty()) {
    return std::nullopt;
  }

  // Compute average z-value from both lanelets for 3D conversion
  double avg_z = 0.0;
  size_t z_count = 0;
  for (const auto & pt : lanelet1.polygon3d()) {
    avg_z += pt.z();
    z_count++;
  }
  for (const auto & pt : lanelet2.polygon3d()) {
    avg_z += pt.z();
    z_count++;
  }
  avg_z = z_count > 0 ? avg_z / static_cast<double>(z_count) : 0.0;

  // Convert the intersection result to LineString3d
  lanelet::LineString3d result_ls(lanelet::InvalId);
  for (const auto & pt_2d : intersection_result[0]) {
    result_ls.push_back(lanelet::Point3d(lanelet::InvalId, pt_2d.x(), pt_2d.y(), avg_z));
  }

  // CompoundPolygon3d requires a vector of ConstLineString3d
  lanelet::ConstLineStrings3d linestrings;
  linestrings.push_back(result_ls);

  return lanelet::CompoundPolygon3d(linestrings);
}
}  // namespace

void RoundaboutLanelets::update(
  const InterpolatedPathInfo & interpolated_path_info,
  const autoware_utils::LinearRing2d & footprint, const double vehicle_length,
  [[maybe_unused]] lanelet::routing::RoutingGraphPtr routing_graph_ptr)
{
  if (!first_attention_area_) {
    const auto first = util::getFirstPointInsidePolygonsByFootprint(
      attention_non_preceding_area_, interpolated_path_info, footprint, vehicle_length);
    if (first) {
      first_attention_lane_ = attention_non_preceding_.at(first.value().second);
      first_attention_area_ = attention_non_preceding_area_.at(first.value().second);
    }
  }
}

void RoundaboutLanelets::calculateInternalLaneletRangeArea(
  const lanelet::ConstLanelet & entry_lanelet,
  const std::shared_ptr<const lanelet::autoware::Roundabout> & roundabout_reg_elem,
  const lanelet::routing::RoutingGraphPtr & routing_graph_ptr)
{
  internal_lanelet_range_.clear();
  internal_lanelet_range_area_.clear();

  if (!roundabout_reg_elem) {
    return;
  }

  // Get all internal lanelets from the roundabout
  const auto internal_lanelets = roundabout_reg_elem->roundaboutInternalLanelets();
  if (internal_lanelets.empty()) {
    return;
  }

  // Find internal lanelets that conflict with ego's entry lanelet
  // getConflictingLanelets returns lanelets that cross/conflict with the given lanelet
  // This properly identifies internal lanelets crossing from the side (upstream direction)
  const auto conflicting_lanelets =
    lanelet::utils::getConflictingLanelets(routing_graph_ptr, entry_lanelet);

  // Filter to only include internal lanelets from the roundabout
  std::vector<lanelet::ConstLanelet> conflicting_internal_lanelets;
  for (const auto & conflicting_lanelet : conflicting_lanelets) {
    if (roundabout_reg_elem->isInternalLanelet(conflicting_lanelet.id())) {
      conflicting_internal_lanelets.push_back(conflicting_lanelet);
    }
  }

  if (conflicting_internal_lanelets.empty()) {
    return;
  }

  // Collect all internal lanelets from all BFS traversals
  std::unordered_set<lanelet::Id> result_internal_lanelet_ids;
  // Collect previous entry lanelet IDs found during BFS
  std::unordered_set<lanelet::Id> prev_entry_lanelet_ids;

  // For each conflicting internal lanelet, perform an independent BFS with its own visited set
  // When an entry lanelet is found, stop that BFS and move to the next conflicting internal lanelet
  for (const auto & start_lanelet : conflicting_internal_lanelets) {
    // Each conflicting internal lanelet has its own visited set
    std::unordered_set<lanelet::Id> visited;
    visited.insert(entry_lanelet.id());  // Don't include ego's entry lanelet

    std::queue<lanelet::ConstLanelet> queue;
    queue.push(start_lanelet);
    visited.insert(start_lanelet.id());
    result_internal_lanelet_ids.insert(start_lanelet.id());

    bool found_entry_lanelet = false;

    while (!queue.empty() && !found_entry_lanelet) {
      const auto current = queue.front();
      queue.pop();

      // Get previous lanelets
      const auto prevs = routing_graph_ptr->previous(current);
      for (const auto & prev : prevs) {
        if (visited.find(prev.id()) != visited.end()) {
          continue;  // Already visited in this BFS
        }
        visited.insert(prev.id());

        // Check if this is an entry lanelet (previous entry)
        if (roundabout_reg_elem->isEntryLanelet(prev.id())) {
          // Found a previous entry lanelet - record it and STOP this BFS entirely
          prev_entry_lanelet_ids.insert(prev.id());
          found_entry_lanelet = true;
          break;  // Exit the for loop
        }

        // Check if this is an internal lanelet
        if (roundabout_reg_elem->isInternalLanelet(prev.id())) {
          result_internal_lanelet_ids.insert(prev.id());
          queue.push(prev);
        }
      }
    }
    // Move to next conflicting internal lanelet (if any)
  }

  // Collect result internal lanelets
  for (const auto & internal_lanelet : internal_lanelets) {
    if (result_internal_lanelet_ids.find(internal_lanelet.id()) != result_internal_lanelet_ids.end()) {
      internal_lanelet_range_.push_back(internal_lanelet);
      internal_lanelet_range_area_.push_back(internal_lanelet.polygon3d());
    }
  }

  // For previous entry lanelets, compute the intersection with conflicting internal lanelets
  // and add only the intersection polygon to internal_lanelet_range_area_
  const auto all_entry_lanelets = roundabout_reg_elem->roundaboutEntryLanelets();
  for (const auto & prev_entry : all_entry_lanelets) {
    // Skip if not in prev_entry_lanelet_ids or is ego's entry lanelet
    if (
      prev_entry_lanelet_ids.find(prev_entry.id()) == prev_entry_lanelet_ids.end() ||
      prev_entry.id() == entry_lanelet.id()) {
      continue;
    }

    // Get internal lanelets that conflict with this previous entry lanelet
    const auto prev_entry_conflicting =
      lanelet::utils::getConflictingLanelets(routing_graph_ptr, prev_entry);

    for (const auto & conflicting : prev_entry_conflicting) {
      // Only process if it's an internal lanelet from the roundabout
      if (roundabout_reg_elem->isInternalLanelet(conflicting.id())) {
        // Compute the geometric intersection between prev_entry and conflicting internal lanelet
        const auto intersection_polygon = computeIntersectionPolygon(prev_entry, conflicting);
        if (intersection_polygon) {
          internal_lanelet_range_.push_back(prev_entry);
          internal_lanelet_range_area_.push_back(intersection_polygon.value());
        }
      }
    }
  }
}
}  // namespace autoware::behavior_velocity_planner
