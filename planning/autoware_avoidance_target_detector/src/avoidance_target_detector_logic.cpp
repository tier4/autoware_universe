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

namespace autoware::avoidance_target_detector
{

AvoidanceTargetDetectorLogic::AvoidanceTargetDetectorLogic(const bool use_extended_route_bounds)
: use_extended_route_bounds_(use_extended_route_bounds)
{
}

void AvoidanceTargetDetectorLogic::set_use_extended_route_bounds(
  const bool use_extended_route_bounds)
{
  use_extended_route_bounds_ = use_extended_route_bounds;
}

void AvoidanceTargetDetectorLogic::update_map_and_route(
  const LaneletMapBin & map, const LaneletRoute & route)
{
  if (route.segments.empty()) {
    return;
  }

  map_bin_ = std::make_shared<LaneletMapBin>(map);
  route_ = std::make_shared<LaneletRoute>(route);
  extended_route_handler_ = std::make_shared<ExtendedRouteHandler>(*map_bin_, *route_);
  extended_route_handler_->create_map();
  extended_route_handler_->export_debug_map();
}

bool AvoidanceTargetDetectorLogic::is_ready() const
{
  return route_ && extended_route_handler_ &&
         extended_route_handler_->getOriginalRouteHandler()->isHandlerReady();
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

std::optional<AvoidanceTargetDetectorLogic::PredictedOutput>
AvoidanceTargetDetectorLogic::process_predicted_objects(
  const rclcpp::Time & current_time, const PredictedObjects & objects,
  const Trajectory & trajectory)
{
  if (!is_ready() || trajectory.points.empty()) {
    return std::nullopt;
  }

  trajectory_ = std::make_shared<Trajectory>(trajectory);
  update_ego_trajectory(trajectory.points.front());

  object_selector_.update_objects(
    current_time, objects, trajectory, *extended_route_handler_, ego_trajectory_,
    ego_trajectory_built_);

  PredictedOutput output;
  output.drivable_area = to_path_msg(route_bounds(), trajectory);
  output.near_segment_polygon = create_near_segment_polygon_marker(current_time, trajectory);
  output.avoidance_targets =
    object_selector_.get_avoidance_targets(objects, trajectory, route_bounds());

  if (ego_trajectory_built_) {
    output.driving_along_vehicles = object_selector_.get_driving_along_vehicles(
      objects, *extended_route_handler_, ego_trajectory_, trajectory);
  }

  return output;
}

std::optional<AvoidanceTargetDetectorLogic::TrackedOutput>
AvoidanceTargetDetectorLogic::process_tracked_objects(
  const rclcpp::Time & current_time, const TrackedObjects & objects,
  const Trajectory & trajectory)
{
  if (!is_ready() || !trajectory_ || trajectory.points.empty()) {
    return std::nullopt;
  }

  const Trajectory trajectory_msg = *trajectory_;

  tracked_object_selector_.update_objects(
    current_time, objects, trajectory_msg, *extended_route_handler_, ego_trajectory_,
    ego_trajectory_built_);

  TrackedOutput output;
  output.tracked_avoidance_targets =
    tracked_object_selector_.get_avoidance_targets(objects, trajectory_msg, route_bounds());

  if (ego_trajectory_built_) {
    output.tracked_driving_along_vehicles = tracked_object_selector_.get_driving_along_vehicles(
      objects, *extended_route_handler_, ego_trajectory_, trajectory_msg);
  }

  return output;
}

}  // namespace autoware::avoidance_target_detector
