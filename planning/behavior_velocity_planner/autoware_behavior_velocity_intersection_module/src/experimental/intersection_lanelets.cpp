// Copyright 2026 Tier IV, Inc.
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

#include "intersection_lanelets.hpp"

#include "autoware/behavior_velocity_intersection_module/experimental/util.hpp"

#include <autoware/trajectory/utils/crop.hpp>

namespace autoware::behavior_velocity_planner::experimental
{

void IntersectionLanelets::update(
  const bool is_prioritized, const Trajectory & path,
  const autoware::experimental::trajectory::Interval & lane_id_interval,
  const autoware_utils::LinearRing2d & footprint, const double vehicle_length)
{
  is_prioritized_ = is_prioritized;

  // find the first conflicting/detection area polygon intersecting the path
  if (!first_conflicting_area_) {
    const auto first = util::getFirstIndexInsidePolygonsByFootprint(
      path, lane_id_interval, conflicting_area_, footprint, vehicle_length);
    if (first) {
      first_conflicting_lane_ = conflicting_.at(first.value().polygon_index);
      first_conflicting_area_ = conflicting_area_.at(first.value().polygon_index);
    }
  }
  if (!first_attention_area_) {
    const auto first = util::getFirstIndexInsidePolygonsByFootprint(
      path, lane_id_interval, attention_non_preceding_area_, footprint, vehicle_length);
    if (first) {
      first_attention_lane_ = attention_non_preceding_.at(first.value().polygon_index);
      first_attention_area_ = attention_non_preceding_area_.at(first.value().polygon_index);
    }
  }
}
}  // namespace autoware::behavior_velocity_planner::experimental
