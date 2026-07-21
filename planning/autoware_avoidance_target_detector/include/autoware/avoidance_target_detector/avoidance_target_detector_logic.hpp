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
#include <autoware_perception_msgs/msg/detail/predicted_objects__struct.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <memory>
#include <optional>
#include <vector>

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
  void update_map(const LaneletMapBin & map);
  void update_route_if_new(const LaneletRoute & route);

  [[nodiscard]] bool is_ready() const;

  [[nodiscard]] std::optional<PredictedObjects> process_predicted_objects(
    const rclcpp::Time & current_time, const PredictedObjects & objects,
    const Trajectory & trajectory);

  [[nodiscard]] std::optional<TrackedObjects> process_tracked_objects(
    const rclcpp::Time & current_time, const TrackedObjects & objects,
    const Trajectory & trajectory);

  [[nodiscard]] std::vector<lanelet::LineString2d> get_road_borders() const
  {
    return extended_route_handler_->get_road_borders();
  }

  [[nodiscard]] const RouteBounds & get_original_route_bounds() const
  {
    return extended_route_handler_->get_original_route_bounds();
  }

  [[nodiscard]] const RouteBounds & get_extended_route_bounds() const
  {
    return extended_route_handler_->get_extended_route_bounds();
  }

  template <typename PointT>
  [[nodiscard]] std::optional<double> get_velocity_limit(const PointT & point) const
  {
    return extended_route_handler_->get_velocity_limit(point);
  }

private:
  void update_ego_trajectory(const TrajectoryPoint & ego_point);

  [[nodiscard]] const RouteBounds & route_bounds() const;

  [[nodiscard]] std::optional<MarkerArray> create_near_segment_polygon_marker(
    const rclcpp::Time & stamp, const Trajectory & trajectory) const;

  bool use_extended_route_bounds_{true};
  std::shared_ptr<ExtendedRouteHandler> extended_route_handler_;
  PredictedObjectSelector object_selector_;
  TrackedObjectSelector tracked_object_selector_;
  autoware::experimental::trajectory::Trajectory<TrajectoryPoint> ego_trajectory_;
  bool ego_trajectory_built_{false};
};

}  // namespace autoware::avoidance_target_detector

#endif  // AUTOWARE__AVOIDANCE_TARGET_DETECTOR__AVOIDANCE_TARGET_DETECTOR_LOGIC_HPP_
