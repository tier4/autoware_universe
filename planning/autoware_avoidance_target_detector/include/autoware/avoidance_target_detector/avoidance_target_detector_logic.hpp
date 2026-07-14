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

#ifndef AUTOWARE__AVOIDANCE_TARGET_DETECTOR__AVOIDANCE_TARGET_DETECTOR_LOGIC_HPP_
#define AUTOWARE__AVOIDANCE_TARGET_DETECTOR__AVOIDANCE_TARGET_DETECTOR_LOGIC_HPP_

#include "autoware/avoidance_target_detector/boundary.hpp"
#include "autoware/avoidance_target_detector/object_filtering.hpp"

#include <autoware/trajectory/trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/primitives/Polygon.h>

#include <memory>
#include <optional>

namespace autoware::avoidance_target_detector
{

using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using visualization_msgs::msg::MarkerArray;

/** Shared avoidance-target detection logic (no ROS I/O). */
class AvoidanceTargetDetectorLogic
{
public:
  explicit AvoidanceTargetDetectorLogic(bool use_extended_route_bounds = true);

  void set_use_extended_route_bounds(bool use_extended_route_bounds);

  /** Rebuild the extended route handler when map or route changes. */
  void update_map_and_route(const LaneletMapBin & map, const LaneletRoute & route);

  [[nodiscard]] bool is_ready() const;

  struct PredictedOutput
  {
    PredictedObjects avoidance_targets;
    PredictedObjects driving_along_vehicles;
    Path drivable_area;
    std::optional<MarkerArray> near_segment_polygon;
  };

  [[nodiscard]] std::optional<PredictedOutput> process_predicted_objects(
    const rclcpp::Time & current_time, const PredictedObjects & objects,
    const Trajectory & trajectory);

  struct TrackedOutput
  {
    TrackedObjects tracked_avoidance_targets;
    TrackedObjects tracked_driving_along_vehicles;
  };

  [[nodiscard]] std::optional<TrackedOutput> process_tracked_objects(
    const rclcpp::Time & current_time, const TrackedObjects & objects,
    const Trajectory & trajectory);

private:
  void update_ego_trajectory(const TrajectoryPoint & ego_point);

  [[nodiscard]] const RouteBounds & route_bounds() const;

  [[nodiscard]] std::optional<MarkerArray> create_near_segment_polygon_marker(
    const rclcpp::Time & stamp, const Trajectory & trajectory) const;

  bool use_extended_route_bounds_{true};
  std::shared_ptr<ExtendedRouteHandler> extended_route_handler_;
  LaneletMapBin::ConstSharedPtr map_bin_;
  LaneletRoute::ConstSharedPtr route_;
  Trajectory::ConstSharedPtr trajectory_;
  PredictedObjectSelector object_selector_;
  TrackedObjectSelector tracked_object_selector_;
  autoware::experimental::trajectory::Trajectory<TrajectoryPoint> ego_trajectory_;
  bool ego_trajectory_built_{false};
};

}  // namespace autoware::avoidance_target_detector

#endif  // AUTOWARE__AVOIDANCE_TARGET_DETECTOR__AVOIDANCE_TARGET_DETECTOR_LOGIC_HPP_
