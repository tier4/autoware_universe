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

#include "autoware/avoidance_target_detector/avoidance_target_detector_logic.hpp"

#include <autoware/marker_utils/marker_conversion.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <vector>

namespace autoware::avoidance_target_detector
{

AvoidanceTargetDetectorLogic::AvoidanceTargetDetectorLogic(const bool use_extended_route_bounds)
: use_extended_route_bounds_(use_extended_route_bounds),
  extended_route_handler_(std::make_shared<ExtendedRouteHandler>())
{
}

void AvoidanceTargetDetectorLogic::set_use_extended_route_bounds(
  const bool use_extended_route_bounds)
{
  use_extended_route_bounds_ = use_extended_route_bounds;
}

void AvoidanceTargetDetectorLogic::update_map(const LaneletMapBin & map_bin)
{
  extended_route_handler_->update_map(map_bin);
}

void AvoidanceTargetDetectorLogic::update_route_if_new(const LaneletRoute & route)
{
  const bool route_updated =
    (route.uuid != extended_route_handler_->getOriginalRouteHandler()->getRouteUuid());
  if (route_updated) {
    extended_route_handler_->update_route(route);
    extended_route_handler_->create_route_map();
  }
}

bool AvoidanceTargetDetectorLogic::is_ready() const
{
  return extended_route_handler_->getOriginalRouteHandler()->isHandlerReady();
}

void AvoidanceTargetDetectorLogic::update_ego_trajectory(const TrajectoryPoint & ego_point)
{
  namespace aw_trajectory = autoware::experimental::trajectory;
  constexpr double k_max_ego_trajectory_length_m = 100.0;
  constexpr std::size_t k_max_ego_trajectory_points = 100;

  std::vector<TrajectoryPoint> points;
  if (ego_trajectory_built_) {
    points = ego_trajectory_.restore();
  }
  points.push_back(ego_point);

  while (points.size() > k_max_ego_trajectory_points) {
    points.erase(points.begin());
  }

  const auto built = aw_trajectory::Trajectory<TrajectoryPoint>::Builder{}.build(points);
  if (!built) {
    return;
  }

  ego_trajectory_ = *built;
  ego_trajectory_built_ = true;

  while (ego_trajectory_.length() > k_max_ego_trajectory_length_m) {
    const double excess = ego_trajectory_.length() - k_max_ego_trajectory_length_m;
    ego_trajectory_.crop(excess, k_max_ego_trajectory_length_m);
  }
}

const RouteBounds & AvoidanceTargetDetectorLogic::route_bounds() const
{
  return use_extended_route_bounds_ ? extended_route_handler_->get_extended_route_bounds()
                                    : extended_route_handler_->get_original_route_bounds();
}

std::optional<MarkerArray> AvoidanceTargetDetectorLogic::create_near_segment_polygon_marker(
  const rclcpp::Time & stamp, const Trajectory & trajectory) const
{
  if (!ego_trajectory_built_ || trajectory.points.empty()) {
    return std::nullopt;
  }

  const auto ego_points = ego_trajectory_.restore();
  if (ego_points.empty()) {
    return std::nullopt;
  }

  const auto near_segment_polygon = extended_route_handler_->get_near_segment_polygon(
    ego_points.front().pose.position, trajectory.points.back().pose.position);
  if (near_segment_polygon.size() < 3) {
    return std::nullopt;
  }

  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;
  return autoware::experimental::marker_utils::create_lanelet_polygon_marker_array(
    near_segment_polygon, stamp, "near_segment_polygon", 0, create_marker_scale(0.4, 0.0, 0.0),
    create_marker_color(1.0, 0.5, 0.0, 0.7), 0.0);
}

std::optional<PredictedObjects> AvoidanceTargetDetectorLogic::process_predicted_objects(
  const rclcpp::Time & current_time, const PredictedObjects & objects,
  const Trajectory & trajectory)
{
  if (!is_ready() || trajectory.points.empty()) {
    return std::nullopt;
  }

  update_ego_trajectory(trajectory.points.front());

  object_selector_.update_objects(
    current_time, objects, trajectory, *extended_route_handler_, ego_trajectory_,
    ego_trajectory_built_);

  return object_selector_.get_all_avoidance_targets(objects, trajectory, route_bounds());
}

std::optional<TrackedObjects> AvoidanceTargetDetectorLogic::process_tracked_objects(
  const rclcpp::Time & current_time, const TrackedObjects & objects, const Trajectory & trajectory)
{
  if (!is_ready() || trajectory.points.empty()) {
    return std::nullopt;
  }

  update_ego_trajectory(trajectory.points.front());

  tracked_object_selector_.update_objects(
    current_time, objects, trajectory, *extended_route_handler_, ego_trajectory_,
    ego_trajectory_built_);

  return tracked_object_selector_.get_all_avoidance_targets(objects, trajectory, route_bounds());
}

}  // namespace autoware::avoidance_target_detector
