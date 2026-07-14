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

#include "autoware/diffusion_planner/avoidance_target_filter.hpp"

#include "autoware/avoidance_target_detector/boundary.hpp"
#include "autoware/avoidance_target_detector/object_filtering.hpp"

#include <autoware/trajectory/trajectory_point.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{
namespace
{
namespace aw_trajectory = autoware::experimental::trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

constexpr double k_max_ego_trajectory_length_m = 100.0;
constexpr std::size_t k_max_ego_trajectory_points = 100;

/** Cheap identity signature to decide when the routing graph must be rebuilt. */
struct RouteContextSignature
{
  std::array<uint8_t, 16> route_uuid{};
  std::size_t route_segment_count{0};
  std::size_t map_data_size{0};
  bool valid{false};

  bool operator==(const RouteContextSignature & other) const
  {
    return valid && other.valid && route_uuid == other.route_uuid &&
           route_segment_count == other.route_segment_count && map_data_size == other.map_data_size;
  }
  bool operator!=(const RouteContextSignature & other) const { return !(*this == other); }
};
}  // namespace

struct AvoidanceTargetFilter::Impl
{
  autoware::avoidance_target_detector::TrackedObjectSelector selector;
  std::shared_ptr<autoware::avoidance_target_detector::ExtendedRouteHandler> extended_route_handler;
  aw_trajectory::Trajectory<TrajectoryPoint> ego_trajectory;
  bool ego_trajectory_built{false};
  RouteContextSignature signature;
};

AvoidanceTargetFilter::AvoidanceTargetFilter() : impl_{std::make_unique<Impl>()} {}
AvoidanceTargetFilter::~AvoidanceTargetFilter() = default;
AvoidanceTargetFilter::AvoidanceTargetFilter(AvoidanceTargetFilter &&) noexcept = default;
AvoidanceTargetFilter & AvoidanceTargetFilter::operator=(AvoidanceTargetFilter &&) noexcept =
  default;

void AvoidanceTargetFilter::set_route_context(const LaneletMapBin & map, const LaneletRoute & route)
{
  RouteContextSignature signature;
  signature.route_uuid = route.uuid.uuid;
  signature.route_segment_count = route.segments.size();
  signature.map_data_size = map.data.size();
  signature.valid = true;

  if (impl_->extended_route_handler && signature == impl_->signature) {
    return;
  }

  impl_->extended_route_handler =
    std::make_shared<autoware::avoidance_target_detector::ExtendedRouteHandler>(map, route);
  impl_->signature = signature;
}

void AvoidanceTargetFilter::update_ego_trajectory(const geometry_msgs::msg::Pose & ego_pose)
{
  TrajectoryPoint ego_point;
  ego_point.pose = ego_pose;

  std::vector<TrajectoryPoint> points;
  if (impl_->ego_trajectory_built) {
    points = impl_->ego_trajectory.restore();
  }
  points.push_back(ego_point);

  while (points.size() > k_max_ego_trajectory_points) {
    points.erase(points.begin());
  }

  const auto built = aw_trajectory::Trajectory<TrajectoryPoint>::Builder{}.build(points);
  if (!built) {
    return;
  }

  impl_->ego_trajectory = *built;
  impl_->ego_trajectory_built = true;

  while (impl_->ego_trajectory.length() > k_max_ego_trajectory_length_m) {
    const double excess = impl_->ego_trajectory.length() - k_max_ego_trajectory_length_m;
    impl_->ego_trajectory.crop(excess, k_max_ego_trajectory_length_m);
  }
}

bool AvoidanceTargetFilter::has_context() const
{
  return impl_->extended_route_handler &&
         impl_->extended_route_handler->getOriginalRouteHandler()->isHandlerReady();
}

TrackedObjects AvoidanceTargetFilter::filter(
  const rclcpp::Time & now, const TrackedObjects & objects, const Trajectory & reference)
{
  if (!has_context()) {
    return objects;
  }

  const auto & route_bounds = impl_->extended_route_handler->get_extended_route_bounds();

  impl_->selector.update_objects(
    now, objects, reference, *impl_->extended_route_handler, impl_->ego_trajectory,
    impl_->ego_trajectory_built);

  TrackedObjects filtered =
    impl_->selector.get_avoidance_targets(objects, reference, route_bounds);

  // Selector treats these as disjoint from stationary avoidance targets.
  if (impl_->ego_trajectory_built && !reference.points.empty()) {
    const TrackedObjects driving_along = impl_->selector.get_driving_along_vehicles(
      objects, *impl_->extended_route_handler, impl_->ego_trajectory, reference);
    filtered.objects.insert(
      filtered.objects.end(), driving_along.objects.begin(), driving_along.objects.end());
  }

  return filtered;
}

}  // namespace autoware::diffusion_planner
