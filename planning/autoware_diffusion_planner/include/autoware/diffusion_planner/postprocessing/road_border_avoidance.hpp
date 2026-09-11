// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__ROAD_BORDER_AVOIDANCE_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__ROAD_BORDER_AVOIDANCE_HPP_

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cstddef>
#include <vector>

namespace autoware::diffusion_planner::postprocess
{
using autoware_planning_msgs::msg::Trajectory;

struct RoadBorderAvoidanceParams
{
  bool enable{false};
  // Longitudinal and lateral inflation of the ego footprint used for the overlap check.
  double footprint_margin_m{0.2};
  // Road borders farther than this from the current ego position are ignored.
  double search_radius_m{120.0};
  // Lateral shift resolution and cap (total offset from the raw position) per point.
  double shift_step_m{0.1};
  double max_lateral_shift_m{1.5};
  // When true, the lateral offset of an overlapping point is carried over to all
  // subsequent points (the path stays shifted instead of snapping back to the raw
  // output right after the border). When false, only overlapping points are shifted.
  bool propagate_shift{true};
};

struct RoadBorderAvoidanceResult
{
  Trajectory trajectory;
  // Points shifted and now clear of all road borders.
  size_t num_shifted_points{0};
  // Points still overlapping after the maximum shift (shift kept, best effort).
  size_t num_unresolved_points{0};

  [[nodiscard]] bool modified() const { return num_shifted_points + num_unresolved_points > 0; }
};

/**
 * @brief Shifts raw model-output trajectory points laterally away from road borders.
 *
 * For every trajectory point the ego footprint (inflated by footprint_margin_m) is placed
 * at the point's pose and checked for overlap against the road border line strings of the
 * lanelet map (boost::geometry). Overlapping points are moved perpendicular to their
 * heading, away from the nearest border, in shift_step_m increments until the footprint
 * clears all borders or the total offset reaches max_lateral_shift_m. With
 * propagate_shift enabled the resulting offset is carried over to all subsequent points
 * (applied along each point's own lateral direction), so the path stays shifted after
 * passing the border instead of snapping back. Yaw and all non-position fields are left
 * untouched. The adjusted trajectory is meant to be fed to the acados-based trajectory
 * optimization as its tracking reference.
 */
class RoadBorderAvoidance
{
public:
  RoadBorderAvoidance(
    const RoadBorderAvoidanceParams & params,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

  /// Extract and cache the road border line strings (map frame) from the lanelet map.
  void set_map(const lanelet::LaneletMap & lanelet_map);

  /// Directly set the road borders (map frame). Used by set_map() and unit tests.
  void set_road_borders(std::vector<autoware_utils_geometry::LineString2d> road_borders);

  /**
   * @brief Check and adjust one raw trajectory (map frame).
   * @param raw_trajectory Raw model-output trajectory.
   * @param ego_pose Current ego pose, used to pre-filter faraway borders.
   */
  [[nodiscard]] RoadBorderAvoidanceResult adjust(
    const Trajectory & raw_trajectory, const geometry_msgs::msg::Pose & ego_pose) const;

private:
  RoadBorderAvoidanceParams params_;
  // Inflated ego footprint in base_link frame.
  autoware_utils_geometry::LinearRing2d base_footprint_;
  // Road border line strings in map frame.
  std::vector<autoware_utils_geometry::LineString2d> road_borders_;
};

}  // namespace autoware::diffusion_planner::postprocess

#endif  // AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__ROAD_BORDER_AVOIDANCE_HPP_
