// Copyright 2026 Autoware Foundation
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

#ifndef AUTOWARE__DIFFUSION_PLANNER__MPPI_RELATED__AVOIDANCE_TARGET_FILTER_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__MPPI_RELATED__AVOIDANCE_TARGET_FILTER_HPP_

#include <rclcpp/time.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <memory>

namespace autoware::diffusion_planner
{

using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;

/**
 * @brief Host-side bridge that filters tracked objects down to avoidance targets.
 *
 * @details Wraps the avoidance_target_detector's TrackedObjectSelector, ExtendedRouteHandler and
 * accumulated ego trajectory behind a PIMPL so the heavy lanelet / route-handler headers stay
 * confined to the implementation translation unit and never leak into node/core headers.
 */
class AvoidanceTargetFilter
{
public:
  AvoidanceTargetFilter();
  ~AvoidanceTargetFilter();

  AvoidanceTargetFilter(const AvoidanceTargetFilter &) = delete;
  AvoidanceTargetFilter & operator=(const AvoidanceTargetFilter &) = delete;
  AvoidanceTargetFilter(AvoidanceTargetFilter &&) noexcept;
  AvoidanceTargetFilter & operator=(AvoidanceTargetFilter &&) noexcept;

  /**
   * @brief Build (or rebuild on change) the extended route handler from map and route.
   * @details The routing graph is rebuilt only when the map or route identity changes.
   */
  void set_route_context(const LaneletMapBin & map, const LaneletRoute & route);

  /**
   * @brief Append the latest ego pose to the accumulated ego trajectory.
   */
  void update_ego_trajectory(const geometry_msgs::msg::Pose & ego_pose);

  /**
   * @brief Whether a route context is available for filtering.
   */
  [[nodiscard]] bool has_context() const;

  /**
   * @brief Update per-object filters and return avoidance targets plus driving-along vehicles.
   * @param now Current time.
   * @param objects Raw tracked objects.
   * @param reference Reference trajectory used for longitudinal/lateral distance filtering.
   * @return Union of avoidance targets and driving-along vehicles (disjoint sets from the
   *         selector); when no context is available, returns @p objects unchanged.
   */
  [[nodiscard]] TrackedObjects filter(
    const rclcpp::Time & now, const TrackedObjects & objects, const Trajectory & reference);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__MPPI_RELATED__AVOIDANCE_TARGET_FILTER_HPP_
