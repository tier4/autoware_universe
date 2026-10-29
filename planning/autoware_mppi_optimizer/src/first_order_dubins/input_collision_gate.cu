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

#include "autoware/mppi_optimizer/detail/trajectory_utils.hpp"
#include "autoware/mppi_optimizer/input_collision_gate.hpp"
#include "autoware/mppi_optimizer/predicted_objects_obstacles.hpp"

#include <mppi/cost_functions/path_tracking_geometry.cuh>
#include <mppi/cost_functions/sat.cuh>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace autoware::mppi_optimizer
{
namespace
{

constexpr double kMaxFootprintTravelPerSampleM = 0.1;
constexpr double kTwoPi = 6.283185307179586;

struct Pose2d
{
  float x;
  float y;
  float yaw;
};

struct StaticObstacle
{
  float x;
  float y;
  float cos_yaw;
  float sin_yaw;
  float half_length;
  float half_width;
};

struct RoadSegments
{
  std::vector<float> x0, y0, x1, y1;
};

template <class Pose>
Pose2d to_pose2d(const Pose & pose, const detail::LocalFrame & frame)
{
  const auto & q = pose.orientation;
  const double norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (
    !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
    !std::isfinite(norm_squared) || norm_squared <= 0.0) {
    throw std::invalid_argument("MPPI input collision gate received an invalid pose");
  }
  const Pose2d result{
    frame.x(pose.position.x), frame.y(pose.position.y), static_cast<float>(tf2::getYaw(q))};
  if (!std::isfinite(result.x) || !std::isfinite(result.y) || !std::isfinite(result.yaw)) {
    throw std::invalid_argument("MPPI input collision gate received an invalid pose");
  }
  return result;
}

RoadSegments make_road_segments(
  const std::vector<Segment> & segments, const detail::LocalFrame & frame)
{
  RoadSegments result;
  result.x0.reserve(segments.size());
  result.y0.reserve(segments.size());
  result.x1.reserve(segments.size());
  result.y1.reserve(segments.size());
  for (const auto & segment : segments) {
    if (
      !std::isfinite(segment.x0) || !std::isfinite(segment.y0) || !std::isfinite(segment.x1) ||
      !std::isfinite(segment.y1)) {
      throw std::invalid_argument("MPPI input collision gate received an invalid road border");
    }
    result.x0.push_back(frame.x(segment.x0));
    result.y0.push_back(frame.y(segment.y0));
    result.x1.push_back(frame.x(segment.x1));
    result.y1.push_back(frame.y(segment.y1));
  }
  return result;
}

std::vector<StaticObstacle> make_static_obstacles(
  const autoware_perception_msgs::msg::TrackedObjects & objects, const detail::LocalFrame & frame)
{
  std::vector<StaticObstacle> result;
  result.reserve(objects.objects.size());
  for (const auto & object : objects.objects) {
    const auto pose = to_pose2d(object.kinematics.pose_with_covariance.pose, frame);
    float length = kDefaultObstacleLength;
    float width = kDefaultObstacleWidth;
    obstacleFootprintFromShape(object.shape, length, width);
    if (!std::isfinite(length) || !std::isfinite(width)) {
      throw std::invalid_argument("MPPI input collision gate received an invalid object shape");
    }
    result.push_back(
      {pose.x, pose.y, std::cos(pose.yaw), std::sin(pose.yaw), length * 0.5F, width * 0.5F});
  }
  return result;
}

bool intersects_road_border(
  const Pose2d & pose, const RoadSegments & segments,
  const FirstOrderDubinsMppiVehicleParams & vehicle, const float margin)
{
  if (segments.x0.empty()) return false;
  const float half_length = vehicle.ego_length * 0.5F;
  return checkRectSegmentIntersections(
    pose.x, pose.y, pose.yaw, vehicle.ego_axle_to_box_center + half_length,
    half_length - vehicle.ego_axle_to_box_center, vehicle.ego_width * 0.5F,
    vehicle.ego_width * 0.5F, margin, segments.x0.data(), segments.y0.data(), segments.x1.data(),
    segments.y1.data(), static_cast<int>(segments.x0.size()));
}

bool intersects_static_object(
  const Pose2d & pose, const std::vector<StaticObstacle> & objects,
  const FirstOrderDubinsMppiVehicleParams & vehicle, const float margin)
{
  if (objects.empty()) return false;
  const float cos_yaw = std::cos(pose.yaw);
  const float sin_yaw = std::sin(pose.yaw);
  const float center_x = pose.x + vehicle.ego_axle_to_box_center * cos_yaw;
  const float center_y = pose.y + vehicle.ego_axle_to_box_center * sin_yaw;
  for (const auto & object : objects) {
    if (mppi::cost::detail::orientedBoxesOverlap(
          center_x, center_y, cos_yaw, sin_yaw, vehicle.ego_length * 0.5F + margin,
          vehicle.ego_width * 0.5F + margin, object.x, object.y, object.cos_yaw, object.sin_yaw,
          object.half_length, object.half_width)) {
      return true;
    }
  }
  return false;
}

void check_pose(
  const Pose2d & pose, const RoadSegments & segments, const std::vector<StaticObstacle> & objects,
  const FirstOrderDubinsMppiVehicleParams & vehicle, const float obstacle_margin,
  const float border_margin, const InputCollisionConditions conditions,
  InputCollisionResult & result)
{
  if (conditions.road_border && !result.road_border) {
    result.road_border = intersects_road_border(pose, segments, vehicle, border_margin);
  }
  if (conditions.static_object && !result.static_object) {
    result.static_object = intersects_static_object(pose, objects, vehicle, obstacle_margin);
  }
}

std::size_t interpolation_steps(
  const Pose2d & start, const Pose2d & end, const FirstOrderDubinsMppiVehicleParams & vehicle)
{
  const double yaw_change = std::remainder(end.yaw - start.yaw, kTwoPi);
  const double front = vehicle.ego_axle_to_box_center + vehicle.ego_length * 0.5F;
  const double rear = vehicle.ego_axle_to_box_center - vehicle.ego_length * 0.5F;
  const double corner_radius =
    std::hypot(std::max(std::abs(front), std::abs(rear)), vehicle.ego_width * 0.5F);
  const double footprint_travel =
    std::hypot(end.x - start.x, end.y - start.y) + corner_radius * std::abs(yaw_change);
  if (!std::isfinite(footprint_travel) || footprint_travel > 10000.0) {
    throw std::invalid_argument("MPPI input collision gate received an invalid point gap");
  }
  return std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::ceil(footprint_travel / kMaxFootprintTravelPerSampleM)));
}

void validate_geometry(
  const FirstOrderDubinsMppiVehicleParams & vehicle, const float obstacle_margin,
  const float border_margin)
{
  if (
    !std::isfinite(vehicle.ego_length) || vehicle.ego_length <= 0.0F ||
    !std::isfinite(vehicle.ego_width) || vehicle.ego_width <= 0.0F ||
    !std::isfinite(vehicle.ego_axle_to_box_center) || !std::isfinite(obstacle_margin) ||
    obstacle_margin < 0.0F || !std::isfinite(border_margin) || border_margin < 0.0F) {
    throw std::invalid_argument("MPPI input collision gate received invalid vehicle geometry");
  }
}

}  // namespace

InputCollisionResult checkInputTrajectoryCollisions(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const autoware_perception_msgs::msg::TrackedObjects & static_objects,
  const std::vector<Segment> & road_borders, const FirstOrderDubinsMppiVehicleParams & vehicle,
  const float obstacle_margin, const float road_border_margin,
  const InputCollisionConditions conditions)
{
  InputCollisionResult result;
  if ((!conditions.road_border && !conditions.static_object) || trajectory.points.empty()) {
    return result;
  }
  validate_geometry(vehicle, obstacle_margin, road_border_margin);
  detail::LocalFrame frame;
  const auto & first_position = trajectory.points.front().pose.position;
  frame.reset(first_position.x, first_position.y, trajectory.header.frame_id);
  const auto segments =
    conditions.road_border ? make_road_segments(road_borders, frame) : RoadSegments{};
  const auto objects = conditions.static_object ? make_static_obstacles(static_objects, frame)
                                                : std::vector<StaticObstacle>{};
  if (segments.x0.empty() && objects.empty()) return result;
  const std::size_t count =
    std::min(trajectory.points.size(), static_cast<std::size_t>(detail::kMppiHorizon));
  auto previous = to_pose2d(trajectory.points.front().pose, frame);
  check_pose(
    previous, segments, objects, vehicle, obstacle_margin, road_border_margin, conditions, result);
  for (std::size_t index = 1U; index < count && !result.triggered(); ++index) {
    const auto next = to_pose2d(trajectory.points[index].pose, frame);
    const auto steps = interpolation_steps(previous, next, vehicle);
    const float yaw_change = std::remainder(next.yaw - previous.yaw, kTwoPi);
    for (std::size_t step = 1U; step <= steps && !result.triggered(); ++step) {
      const float ratio = static_cast<float>(step) / static_cast<float>(steps);
      const Pose2d pose{
        previous.x + (next.x - previous.x) * ratio, previous.y + (next.y - previous.y) * ratio,
        previous.yaw + yaw_change * ratio};
      check_pose(
        pose, segments, objects, vehicle, obstacle_margin, road_border_margin, conditions, result);
    }
    previous = next;
  }
  return result;
}

}  // namespace autoware::mppi_optimizer
