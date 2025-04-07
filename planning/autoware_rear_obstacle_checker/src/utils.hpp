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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include "autoware/behavior_path_planner_common/marker_utils/colors.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/transform/transforms.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/length.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::rear_obstacle_checker::utils
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace
{
std::uint8_t get_highest_prob_label(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classifications)
{
  std::uint8_t label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
  float highest_prob = 0.0;
  for (const auto & classification : classifications) {
    if (highest_prob < classification.probability) {
      highest_prob = classification.probability;
      label = classification.label;
    }
  }
  return label;
}

bool is_target_behavior(
  const tier4_planning_msgs::msg::PlanningFactor & factor,
  const rear_obstacle_checker_node::Params::MapScene & config)
{
  switch (factor.behavior) {
    case PlanningFactor::SLOW_DOWN:
      return std::any_of(
        config.behavior.begin(), config.behavior.end(),
        [](const auto & condition) { return condition == "slow_down"; });
    case PlanningFactor::STOP:
      return std::any_of(
        config.behavior.begin(), config.behavior.end(),
        [](const auto & condition) { return condition == "stop"; });
    case PlanningFactor::SHIFT_LEFT:
      return std::any_of(
        config.behavior.begin(), config.behavior.end(),
        [](const auto & condition) { return condition == "shift_left"; });
    case PlanningFactor::SHIFT_RIGHT:
      return std::any_of(
        config.behavior.begin(), config.behavior.end(),
        [](const auto & condition) { return condition == "shift_right"; });
    case PlanningFactor::TURN_LEFT:
      return std::any_of(
        config.behavior.begin(), config.behavior.end(),
        [](const auto & condition) { return condition == "turn_left"; });
    case PlanningFactor::TURN_RIGHT:
      return std::any_of(
        config.behavior.begin(), config.behavior.end(),
        [](const auto & condition) { return condition == "turn_right"; });
  }

  return false;
}

bool has_valid_control_points(const tier4_planning_msgs::msg::PlanningFactor & factor)
{
  if (factor.control_points.empty()) {
    return false;
  }

  // TODO(satoshi-ota): don't use distance to prevent unnecessary lane checking.
  if (factor.control_points.front().distance < 0.0) {
    return false;
  }

  switch (factor.behavior) {
    case PlanningFactor::SLOW_DOWN:
      return true;
    case PlanningFactor::STOP:
      return std::any_of(
        factor.control_points.begin(), factor.control_points.end(),
        [](const auto & control_point) { return std::abs(control_point.velocity) < 1e-3; });
    case PlanningFactor::SHIFT_LEFT:
      return std::any_of(
        factor.control_points.begin(), factor.control_points.end(),
        [](const auto & control_point) { return std::abs(control_point.shift_length) > 0.0; });
    case PlanningFactor::SHIFT_RIGHT:
      return std::any_of(
        factor.control_points.begin(), factor.control_points.end(),
        [](const auto & control_point) { return std::abs(control_point.shift_length) > 0.0; });
    case PlanningFactor::TURN_LEFT:
      return true;
    case PlanningFactor::TURN_RIGHT:
      return true;
  }

  return false;
}

lanelet::ConstLanelets get_previous_lanes_recursively(
  const lanelet::ConstLanelet & lane, const double length, const double threshold,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  lanelet::ConstLanelets ret{lane};
  for (const auto & prev_lane : route_handler->getPreviousLanelets(lane)) {
    const double total_length = length + lanelet::utils::getLaneletLength2d(prev_lane);
    if (total_length < threshold) {
      const auto prev_lanes =
        get_previous_lanes_recursively(prev_lane, total_length, threshold, route_handler);
      ret.insert(ret.end(), prev_lanes.begin(), prev_lanes.end());
    }
  }

  return ret;
}

lanelet::BasicPolygons3d get_previous_polygons_recursively(
  const lanelet::ConstLanelets & lanes, const double s1, const double s2,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  lanelet::BasicPolygons3d ret{};

  if (lanes.empty()) {
    return ret;
  }

  for (const auto & prev_lane : route_handler->getPreviousLanelets(lanes.front())) {
    lanelet::ConstLanelets pushed_lanes = lanes;
    pushed_lanes.insert(pushed_lanes.begin(), prev_lane);
    const auto total_length = lanelet::utils::getLaneletLength2d(pushed_lanes);
    if (total_length > s2) {
      const auto polygon =
        lanelet::utils::getPolygonFromArcLength(pushed_lanes, total_length - s2, total_length - s1);
      ret.push_back(polygon.basicPolygon());
    } else {
      const auto polygons = get_previous_polygons_recursively(pushed_lanes, s1, s2, route_handler);
      ret.insert(ret.end(), polygons.begin(), polygons.end());
    }
  }

  return ret;
}

bool is_within_lanes(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & ego_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const auto transform = autoware_utils::pose2transform(ego_pose);
  const auto footprint =
    autoware_utils::transform_vector(vehicle_info.createFootprint(), transform);

  const auto combine_lanelet = lanelet::utils::combineLaneletsShape(lanelets);

  return boost::geometry::within(footprint, combine_lanelet.polygon2d().basicPolygon());
}

std::pair<lanelet::BasicPoint2d, double> get_smallest_enclosing_circle(
  const lanelet::BasicPolygon2d & poly)
{
  // The `eps` is used to avoid precision bugs in circle inclusion checks.
  // If the value of `eps` is too small, this function doesn't work well. More than 1e-10 is
  // recommended.
  const double eps = 1e-5;
  lanelet::BasicPoint2d center(0.0, 0.0);
  double radius_squared = 0.0;

  auto cross = [](const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2) -> double {
    return p1.x() * p2.y() - p1.y() * p2.x();
  };

  auto make_circle_3 = [&](
                         const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2,
                         const lanelet::BasicPoint2d & p3) -> void {
    // reference for circumcenter vector https://en.wikipedia.org/wiki/Circumscribed_circle
    const double a = (p2 - p3).squaredNorm();
    const double b = (p3 - p1).squaredNorm();
    const double c = (p1 - p2).squaredNorm();
    const double s = cross(p2 - p1, p3 - p1);
    if (std::abs(s) < eps) return;
    center = (a * (b + c - a) * p1 + b * (c + a - b) * p2 + c * (a + b - c) * p3) / (4 * s * s);
    radius_squared = (center - p1).squaredNorm() + eps;
  };

  auto make_circle_2 =
    [&](const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2) -> void {
    center = (p1 + p2) * 0.5;
    radius_squared = (center - p1).squaredNorm() + eps;
  };

  auto in_circle = [&](const lanelet::BasicPoint2d & p) -> bool {
    return (center - p).squaredNorm() <= radius_squared;
  };

  // mini disc
  for (size_t i = 1; i < poly.size(); i++) {
    const auto p1 = poly[i];
    if (in_circle(p1)) continue;

    // mini disc with point
    const auto p0 = poly[0];
    make_circle_2(p0, p1);
    for (size_t j = 0; j < i; j++) {
      const auto p2 = poly[j];
      if (in_circle(p2)) continue;

      // mini disc with two points
      make_circle_2(p1, p2);
      for (size_t k = 0; k < j; k++) {
        const auto p3 = poly[k];
        if (in_circle(p3)) continue;

        // mini disc with tree points
        make_circle_3(p1, p2, p3);
      }
    }
  }

  return std::make_pair(center, radius_squared);
}
}  // namespace

lanelet::BasicPolygon3d to_basic_polygon3d(
  const autoware_utils::LinearRing2d & footprint, const double z)
{
  lanelet::BasicPolygon3d polygon;
  for (const auto & point : footprint) {
    Eigen::Vector3d p(point.x(), point.y(), z);
    polygon.push_back(p);
  }
  return polygon;
}

auto is_shift_path(
  const lanelet::ConstLanelets & lanelets,
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info) -> std::pair<bool, bool>
{
  const auto combine_lanelet = lanelet::utils::combineLaneletsShape(lanelets);

  for (const auto & p : points) {
    const auto transform = autoware_utils::pose2transform(autoware_utils::get_pose(p));
    const auto footprint =
      autoware_utils::transform_vector(vehicle_info.createFootprint(), transform);

    const auto is_left_shift = boost::geometry::intersects(
      footprint, lanelet::utils::to2D(combine_lanelet.leftBound()).basicLineString());
    const auto is_right_shift = boost::geometry::intersects(
      footprint, lanelet::utils::to2D(combine_lanelet.rightBound()).basicLineString());

    if (is_left_shift || is_right_shift) {
      return std::make_pair(is_left_shift, is_right_shift);
    }
  }

  return std::make_pair(false, false);
}

bool should_activate(
  const tier4_planning_msgs::msg::PlanningFactor & factor,
  const rear_obstacle_checker_node::Params & parameters)
{
  const auto config = parameters.scene_map.at(factor.module);

  if (config.safety_condition == "safe") {
    if (!factor.safety_factors.is_safe) {
      return false;
    }
  }

  if (config.safety_condition == "unsafe") {
    if (factor.safety_factors.is_safe) {
      return false;
    }
  }

  if (!is_target_behavior(factor, config)) {
    return false;
  }

  if (config.check_control_points) {
    if (!has_valid_control_points(factor)) {
      return false;
    }
  }

  return true;
}

bool should_check_objects(
  const tier4_planning_msgs::msg::PlanningFactor & factor,
  const rear_obstacle_checker_node::Params & parameters)
{
  const auto config = parameters.scene_map.at(factor.module);
  return std::any_of(
    config.target_type.begin(), config.target_type.end(),
    [](const auto & target_type) { return target_type != "pointcloud"; });
}

bool should_check_pointcloud(
  const tier4_planning_msgs::msg::PlanningFactor & factor,
  const rear_obstacle_checker_node::Params & parameters)
{
  const auto config = parameters.scene_map.at(factor.module);
  return std::any_of(
    config.target_type.begin(), config.target_type.end(),
    [](const auto & target_type) { return target_type == "pointcloud"; });
}

bool is_target(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const tier4_planning_msgs::msg::PlanningFactor & factor,
  const rear_obstacle_checker_node::Params & parameters)
{
  using autoware_perception_msgs::msg::ObjectClassification;

  const auto config = parameters.scene_map.at(factor.module);

  const auto label = get_highest_prob_label(object.classification);
  switch (label) {
    case ObjectClassification::UNKNOWN:
      return std::any_of(
        config.target_type.begin(), config.target_type.end(),
        [](const auto & target_type) { return target_type == "unknown"; });
    case ObjectClassification::CAR:
      return std::any_of(
        config.target_type.begin(), config.target_type.end(),
        [](const auto & target_type) { return target_type == "car"; });
    case ObjectClassification::TRUCK:
      return std::any_of(
        config.target_type.begin(), config.target_type.end(),
        [](const auto & target_type) { return target_type == "truck"; });
    case ObjectClassification::TRAILER:
      return std::any_of(
        config.target_type.begin(), config.target_type.end(),
        [](const auto & target_type) { return target_type == "trailer"; });
    case ObjectClassification::BUS:
      return std::any_of(
        config.target_type.begin(), config.target_type.end(),
        [](const auto & target_type) { return target_type == "bus"; });
    case ObjectClassification::MOTORCYCLE:
      return std::any_of(
        config.target_type.begin(), config.target_type.end(),
        [](const auto & target_type) { return target_type == "motorcycle"; });
    case ObjectClassification::PEDESTRIAN:
      return std::any_of(
        config.target_type.begin(), config.target_type.end(),
        [](const auto & target_type) { return target_type == "pedestrian"; });
    case ObjectClassification::BICYCLE:
      return std::any_of(
        config.target_type.begin(), config.target_type.end(),
        [](const auto & target_type) { return target_type == "bicycle"; });
  }

  return false;
}

std::optional<geometry_msgs::msg::Pose> calc_predicted_stop_pose(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & ego_pose, const double current_velocity,
  const double current_acceleration, const rear_obstacle_checker_node::Params & parameters)
{
  constexpr double target_velocity = 0.0;
  const auto & max_deceleration = parameters.common.prediction.ego.max_deceleration;
  const auto & max_positive_jerk = parameters.common.prediction.ego.max_positive_jerk;
  const auto & max_negative_jerk = parameters.common.prediction.ego.max_negative_jerk;

  if (current_velocity < target_velocity + 1e-3) {
    return ego_pose;
  }

  const auto stop_distance = autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
    current_velocity, target_velocity, current_acceleration, max_deceleration, max_positive_jerk,
    max_negative_jerk);

  if (!stop_distance.has_value()) {
    return {};
  }

  return autoware::motion_utils::calcLongitudinalOffsetPose(
    points, ego_pose.position, stop_distance.value());
}

PointCloud get_obstacle_points(const lanelet::BasicPolygons3d & polygons, const PointCloud & points)
{
  PointCloud ret;
  for (const auto & polygon : polygons) {
    const auto circle = get_smallest_enclosing_circle(lanelet::utils::to2D(polygon));
    for (const auto & p : points) {
      const double squared_dist = (circle.first.x() - p.x) * (circle.first.x() - p.x) +
                                  (circle.first.y() - p.y) * (circle.first.y() - p.y);
      if (squared_dist > circle.second) {
        continue;
      }
      if (boost::geometry::within(
            autoware_utils::Point2d{p.x, p.y}, lanelet::utils::to2D(polygon))) {
        ret.push_back(p);
      }
    }
  }
  return ret;
}

PointCloud filter_lost_object_pointcloud(
  const PredictedObjects & objects, const PointCloud & points, const double object_buffer)
{
  PointCloud ret = points;
  for (const auto & object : objects.objects) {
    const auto polygon =
      autoware_utils::expand_polygon(autoware_utils::to_polygon2d(object), object_buffer);
    lanelet::BasicPolygon2d basic_polygon;
    for (const auto & p : polygon.outer()) {
      basic_polygon.emplace_back(p.x(), p.y());
    }
    const auto circle = get_smallest_enclosing_circle(basic_polygon);
    auto itr = ret.begin();
    while (itr != ret.end()) {
      const double squared_dist = (circle.first.x() - itr->x) * (circle.first.x() - itr->x) +
                                  (circle.first.y() - itr->y) * (circle.first.y() - itr->y);
      if (squared_dist > circle.second) {
        itr++;
      } else if (boost::geometry::within(autoware_utils::Point2d{itr->x, itr->y}, polygon)) {
        itr = ret.erase(itr);
      } else {
        itr++;
      }
    }
  }
  return ret;
}

lanelet::ConstLanelets get_current_lanes(
  const geometry_msgs::msg::Pose & ego_pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double forward_distance,
  const double backward_distance)
{
  lanelet::ConstLanelet closest_lanelet;
  if (!route_handler->getClosestLaneletWithinRoute(ego_pose, &closest_lanelet)) {
    return {};
  }

  const auto current_lanes = route_handler->getLaneletSequence(
    closest_lanelet, ego_pose, forward_distance, backward_distance);

  if (is_within_lanes(current_lanes, ego_pose, vehicle_info)) {
    return current_lanes;
  }

  const auto start_pose = route_handler->getOriginalStartPose();
  const auto pull_out_start_lane_opt =
    route_handler->getPullOutStartLane(start_pose, vehicle_info.vehicle_width_m);
  if (!pull_out_start_lane_opt.has_value()) {
    return {};
  }

  const auto road_shoulder_lanes =
    route_handler->getShoulderLaneletSequence(pull_out_start_lane_opt.value(), start_pose);
  if (is_within_lanes(road_shoulder_lanes, ego_pose, vehicle_info)) {
    return road_shoulder_lanes;
  }

  return {};
}

lanelet::ConstLanelets get_adjacent_lanes(
  const lanelet::ConstLanelets & current_lanes, const geometry_msgs::msg::Pose & ego_pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler, const bool is_right,
  const double backward_distance)
{
  const auto ego_coordinate_on_arc = lanelet::utils::getArcCoordinates(current_lanes, ego_pose);

  lanelet::ConstLanelets lanes{};

  const auto exist_in_current_lane = [&current_lanes](const auto id) {
    const auto itr = std::find_if(
      current_lanes.begin(), current_lanes.end(),
      [&id](const auto & lane) { return lane.id() == id; });
    return itr != current_lanes.end();
  };

  const auto exist = [&lanes](const auto id) {
    const auto itr = std::find_if(
      lanes.begin(), lanes.end(), [&id](const auto & lane) { return lane.id() == id; });
    return itr != lanes.end();
  };

  double length = 0.0;
  for (const auto & lane : current_lanes) {
    const auto residual_length = backward_distance - ego_coordinate_on_arc.length + length;
    const auto opt_left_lane = route_handler->getLeftLanelet(lane, true, true);
    if (!is_right) {
      if (opt_left_lane.has_value()) {
        lanes.push_back(opt_left_lane.value());

        for (const auto & prev_lane : get_previous_lanes_recursively(
               opt_left_lane.value(), 0.0, residual_length, route_handler)) {
          if (!exist(prev_lane.id()) && !exist_in_current_lane(prev_lane.id())) {
            lanes.push_back(prev_lane);
          }
        }
      }
    }

    const auto opt_right_lane = route_handler->getRightLanelet(lane, true, true);
    if (is_right) {
      if (opt_right_lane.has_value()) {
        lanes.push_back(opt_right_lane.value());

        for (const auto & prev_lane : get_previous_lanes_recursively(
               opt_right_lane.value(), 0.0, residual_length, route_handler)) {
          if (!exist(prev_lane.id()) && !exist_in_current_lane(prev_lane.id())) {
            lanes.push_back(prev_lane);
          }
        }
      }
    }

    length += lanelet::utils::getLaneletLength2d(lane);
  }

  return lanes;
}

lanelet::BasicPolygon3d generate_detection_polygon(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::msg::Pose & ego_pose,
  const double forward_distance, const double backward_distance)
{
  const auto ego_coordinate_on_arc = lanelet::utils::getArcCoordinates(lanelets, ego_pose).length;
  const auto polygon = lanelet::utils::getPolygonFromArcLength(
    lanelets, ego_coordinate_on_arc - backward_distance, ego_coordinate_on_arc + forward_distance);
  return polygon.basicPolygon();
}

lanelet::BasicPolygons3d get_adjacent_polygons(
  const lanelet::ConstLanelets & current_lanes, const geometry_msgs::msg::Pose & ego_pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler, const bool is_right,
  const double forward_distance, const double backward_distance)
{
  const auto ego_coordinate_on_arc = lanelet::utils::getArcCoordinates(current_lanes, ego_pose);

  lanelet::BasicPolygons3d ret{};

  lanelet::ConstLanelets root{};

  double length = 0.0;
  for (const auto & lane : current_lanes) {
    const auto total_length = length + lanelet::utils::getLaneletLength2d(lane);
    const auto residual_distance = total_length - ego_coordinate_on_arc.length - forward_distance;

    if (!is_right) {
      const auto opt_left_lane = route_handler->getLeftLanelet(lane, true, true);
      if (opt_left_lane.has_value()) {
        root = {opt_left_lane.value()};
      }

      if (opt_left_lane.has_value() && residual_distance > 0.0) {
        const auto polygons = get_previous_polygons_recursively(
          root, residual_distance, residual_distance + forward_distance + backward_distance,
          route_handler);
        ret.insert(ret.end(), polygons.begin(), polygons.end());
        return ret;
      }

      if (!opt_left_lane.has_value()) {
        const auto polygons = get_previous_polygons_recursively(
          root, 0.0, length - ego_coordinate_on_arc.length + backward_distance, route_handler);
        ret.insert(ret.end(), polygons.begin(), polygons.end());
        root.clear();
      }
    }

    if (is_right) {
      const auto opt_right_lane = route_handler->getRightLanelet(lane, true, true);
      if (opt_right_lane.has_value()) {
        root = {opt_right_lane.value()};
      }

      if (opt_right_lane.has_value() && residual_distance > 0.0) {
        const auto polygons = get_previous_polygons_recursively(
          root, residual_distance, residual_distance + forward_distance + backward_distance,
          route_handler);
        ret.insert(ret.end(), polygons.begin(), polygons.end());
        return ret;
      }

      if (!opt_right_lane.has_value()) {
        const auto polygons = get_previous_polygons_recursively(
          root, 0.0, length - ego_coordinate_on_arc.length + backward_distance, route_handler);
        ret.insert(ret.end(), polygons.begin(), polygons.end());
        root.clear();
      }
    }

    length += lanelet::utils::getLaneletLength2d(lane);
  }

  return ret;
}

auto get_previous_polygons_with_lane_recursively(
  const lanelet::ConstLanelets & lanes, const double s1, const double s2,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler) -> DetectionAreas
{
  DetectionAreas ret{};

  if (lanes.empty()) {
    return ret;
  }

  for (const auto & prev_lane : route_handler->getPreviousLanelets(lanes.front())) {
    lanelet::ConstLanelets pushed_lanes = lanes;
    pushed_lanes.insert(pushed_lanes.begin(), prev_lane);
    const auto total_length = lanelet::utils::getLaneletLength2d(pushed_lanes);
    if (total_length > s2) {
      const auto polygon =
        lanelet::utils::getPolygonFromArcLength(pushed_lanes, total_length - s2, total_length - s1);
      ret.emplace_back(polygon.basicPolygon(), pushed_lanes);
    } else {
      const auto polygons =
        get_previous_polygons_with_lane_recursively(pushed_lanes, s1, s2, route_handler);
      ret.insert(ret.end(), polygons.begin(), polygons.end());
    }
  }

  return ret;
}

auto get_pointcloud_object(
  const rclcpp::Time & now, const PointCloud::Ptr & pointcloud_ptr,
  const DetectionAreas & detection_areas,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
  -> std::optional<PointCloudObject>
{
  std::optional<PointCloudObject> opt_object = std::nullopt;
  for (const auto & [polygon, lanes] : detection_areas) {
    const auto pointcloud = get_obstacle_points({polygon}, *pointcloud_ptr);

    const auto path =
      route_handler->getCenterLinePath(lanes, 0.0, std::numeric_limits<double>::max());
    const auto resampled_path = behavior_path_planner::utils::resamplePathWithSpline(path, 0.3);

    for (const auto & point : pointcloud) {
      // TODO(satoshi-ota): remove redundant implementation.
      const double obj_arc_length = autoware::motion_utils::calcSignedArcLength(
        resampled_path.points, autoware_utils::create_point(point.x, point.y, point.z),
        autoware_utils::get_point(path.points.back()));
      const auto point_on_ceter_line = autoware::motion_utils::calcLongitudinalOffsetPoint(
        resampled_path.points, autoware_utils::get_point(resampled_path.points.back()),
        -1.0 * obj_arc_length);

      if (!point_on_ceter_line.has_value()) {
        continue;
      }

      if (!opt_object.has_value()) {
        PointCloudObject object;
        object.last_update_time = now;
        object.position = point_on_ceter_line.value();
        object.base_lane_id = lanes.front().id();
        object.absolute_distance = obj_arc_length;
        object.velocity = 0.0;
        opt_object = object;
      } else if (opt_object.value().absolute_distance > obj_arc_length) {
        opt_object.value().last_update_time = now;
        opt_object.value().position = point_on_ceter_line.value();
        opt_object.value().base_lane_id = lanes.front().id();
        opt_object.value().absolute_distance = obj_arc_length;
        opt_object.value().velocity = 0.0;
      }
    }
  }

  return opt_object;
}

MarkerArray create_polygon_marker_array(
  const lanelet::BasicPolygons3d & polygons, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color)
{
  MarkerArray msg;

  size_t i = 0;
  for (const auto & polygon : polygons) {
    auto marker = autoware_utils::create_default_marker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i++, Marker::LINE_STRIP,
      autoware_utils::create_marker_scale(0.2, 0.0, 0.0), color);

    for (const auto & p : polygon) {
      marker.points.push_back(autoware_utils::create_point(p.x(), p.y(), p.z()));
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

lanelet::ConstLanelet generate_half_lanelet(
  const lanelet::ConstLanelet lanelet, const bool is_right,
  const double ignore_width_from_centerline)
{
  lanelet::Points3d lefts, rights;

  const double offset = !is_right ? ignore_width_from_centerline : -ignore_width_from_centerline;
  const auto offset_centerline = lanelet::utils::getCenterlineWithOffset(lanelet, offset);

  const auto original_left_bound = !is_right ? lanelet.leftBound() : offset_centerline;
  const auto original_right_bound = !is_right ? offset_centerline : lanelet.rightBound();

  for (const auto & pt : original_left_bound) {
    lefts.emplace_back(pt);
  }
  for (const auto & pt : original_right_bound) {
    rights.emplace_back(pt);
  }
  const auto left_bound = lanelet::LineString3d(lanelet::InvalId, std::move(lefts));
  const auto right_bound = lanelet::LineString3d(lanelet::InvalId, std::move(rights));
  auto half_lanelet = lanelet::Lanelet(lanelet::InvalId, left_bound, right_bound);
  return half_lanelet;
}

lanelet::ConstLanelet generate_offset_lanelet(
  const lanelet::ConstLanelet lanelet, const double offset_near, const double offset_far,
  const bool is_right)
{
  lanelet::Points3d lefts, rights;

  const auto bound_near = lanelet::utils::getCenterlineWithOffset(lanelet, offset_near);
  const auto bound_far = lanelet::utils::getCenterlineWithOffset(lanelet, offset_far);

  const auto original_left_bound = is_right ? bound_near : bound_far;
  const auto original_right_bound = is_right ? bound_far : bound_near;

  for (const auto & pt : original_left_bound) {
    lefts.emplace_back(pt);
  }
  for (const auto & pt : original_right_bound) {
    rights.emplace_back(pt);
  }
  const auto left_bound = lanelet::LineString3d(lanelet::InvalId, std::move(lefts));
  const auto right_bound = lanelet::LineString3d(lanelet::InvalId, std::move(rights));
  auto half_lanelet = lanelet::Lanelet(lanelet::InvalId, left_bound, right_bound);
  return half_lanelet;
}

auto generate_detection_area_for_pointcloud(
  const PlanningFactor & factor, const lanelet::ConstLanelets & current_lanes,
  const geometry_msgs::msg::Pose & ego_pose, const double current_velocity,
  const double current_acceleration,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const rear_obstacle_checker_node::Params & parameters) -> lanelet::BasicPolygons3d
{
  const auto config = parameters.scene_map.at(factor.module);

  lanelet::BasicPolygons3d detection_polygons{};

  constexpr double v_object = 10.0;

  const auto delay_ego = parameters.common.prediction.object.reaction_time;
  const auto max_deceleration_ego = parameters.common.prediction.ego.max_deceleration;
  const auto max_deceleration_object = parameters.common.prediction.object.max_deceleration;

  const auto stop_distance_object =
    0.5 * std::pow(v_object, 2.0) / std::abs(max_deceleration_object);
  const auto stop_distance_ego =
    current_velocity * delay_ego + 0.5 * current_acceleration * std::pow(delay_ego, 2.0) +
    0.5 * std::pow(current_velocity + current_acceleration * delay_ego, 2.0) /
      std::abs(max_deceleration_ego);

  const auto forward_distance =
    parameters.common.range.pointcloud.forward + vehicle_info.max_longitudinal_offset_m;
  const auto backward_distance = parameters.common.range.pointcloud.backward -
                                 vehicle_info.min_longitudinal_offset_m +
                                 std::max(0.0, stop_distance_object - stop_distance_ego);

  if (
    factor.behavior == PlanningFactor::SHIFT_LEFT || factor.behavior == PlanningFactor::TURN_LEFT) {
    if (config.adjacent_lane) {
      const auto adjacent_polygons = utils::get_adjacent_polygons(
        current_lanes, ego_pose, route_handler, false, forward_distance, backward_distance);
      detection_polygons.insert(
        detection_polygons.end(), adjacent_polygons.begin(), adjacent_polygons.end());
    }

    if (config.current_lane) {
      const auto half_lanes = [&current_lanes, &vehicle_info]() {
        lanelet::ConstLanelets ret{};
        for (const auto & lane : current_lanes) {
          ret.push_back(
            utils::generate_half_lanelet(lane, false, 0.5 * vehicle_info.vehicle_width_m));
        }
        return ret;
      }();
      detection_polygons.push_back(utils::generate_detection_polygon(
        half_lanes, ego_pose, forward_distance, backward_distance));
    }
  }

  if (
    factor.behavior == PlanningFactor::SHIFT_RIGHT ||
    factor.behavior == PlanningFactor::TURN_RIGHT) {
    if (config.adjacent_lane) {
      const auto adjacent_polygons = utils::get_adjacent_polygons(
        current_lanes, ego_pose, route_handler, true, forward_distance, backward_distance);
      detection_polygons.insert(
        detection_polygons.end(), adjacent_polygons.begin(), adjacent_polygons.end());
    }

    if (config.current_lane) {
      const auto half_lanes = [&current_lanes, &vehicle_info]() {
        lanelet::ConstLanelets ret{};
        for (const auto & lane : current_lanes) {
          ret.push_back(
            utils::generate_half_lanelet(lane, true, 0.5 * vehicle_info.vehicle_width_m));
        }
        return ret;
      }();
      detection_polygons.push_back(utils::generate_detection_polygon(
        half_lanes, ego_pose, forward_distance, backward_distance));
    }
  }

  return detection_polygons;
}

auto generate_detection_area_for_object(
  const PlanningFactor & factor, const lanelet::ConstLanelets & current_lanes,
  const geometry_msgs::msg::Pose & ego_pose,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const rear_obstacle_checker_node::Params & parameters) -> lanelet::ConstLanelets
{
  const auto config = parameters.scene_map.at(factor.module);

  lanelet::ConstLanelets detection_lanes_for_objects{};

  if (
    factor.behavior == PlanningFactor::SHIFT_LEFT || factor.behavior == PlanningFactor::TURN_LEFT) {
    if (config.adjacent_lane) {
      const auto adjacent_lanes = utils::get_adjacent_lanes(
        current_lanes, ego_pose, route_handler, false, parameters.common.range.object.backward);
      detection_lanes_for_objects.insert(
        detection_lanes_for_objects.end(), adjacent_lanes.begin(), adjacent_lanes.end());
    }
    if (config.current_lane) {
      const auto half_lanes = [&current_lanes, &vehicle_info]() {
        lanelet::ConstLanelets ret{};
        for (const auto & lane : current_lanes) {
          ret.push_back(
            utils::generate_half_lanelet(lane, false, 0.5 * vehicle_info.vehicle_width_m));
        }
        return ret;
      }();
      detection_lanes_for_objects.insert(
        detection_lanes_for_objects.end(), half_lanes.begin(), half_lanes.end());
    }
  }

  if (
    factor.behavior == PlanningFactor::SHIFT_RIGHT ||
    factor.behavior == PlanningFactor::TURN_RIGHT) {
    if (config.adjacent_lane) {
      const auto adjacent_lanes = utils::get_adjacent_lanes(
        current_lanes, ego_pose, route_handler, true, parameters.common.range.object.backward);
      detection_lanes_for_objects.insert(
        detection_lanes_for_objects.end(), adjacent_lanes.begin(), adjacent_lanes.end());
    }
    if (config.current_lane) {
      const auto half_lanes = [&current_lanes, &vehicle_info]() {
        lanelet::ConstLanelets ret{};
        for (const auto & lane : current_lanes) {
          ret.push_back(
            utils::generate_half_lanelet(lane, true, 0.5 * vehicle_info.vehicle_width_m));
        }
        return ret;
      }();
      detection_lanes_for_objects.insert(
        detection_lanes_for_objects.end(), half_lanes.begin(), half_lanes.end());
    }
  }

  return detection_lanes_for_objects;
}

MarkerArray showPolygon(
  const behavior_path_planner::utils::path_safety_checker::CollisionCheckDebugMap & obj_debug_vec,
  std::string && ns)
{
  if (obj_debug_vec.empty()) {
    return MarkerArray{};
  }

  int32_t id{0};
  const auto now = rclcpp::Clock{RCL_ROS_TIME}.now();

  constexpr float line_scale_val{0.2};
  const auto line_marker_scale =
    autoware_utils::create_marker_scale(line_scale_val, line_scale_val, line_scale_val);

  auto default_line_marker = [&](const auto & color = marker_utils::colors::green()) {
    return autoware_utils::create_default_marker(
      "map", now, ns, ++id, Marker::LINE_STRIP, line_marker_scale, color);
  };

  constexpr float text_scale_val{1.5};
  const auto text_marker_scale =
    autoware_utils::create_marker_scale(text_scale_val, text_scale_val, text_scale_val);

  auto default_text_marker = [&]() {
    return autoware_utils::create_default_marker(
      "map", now, ns + "_text", ++id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      text_marker_scale, marker_utils::colors::white());
  };

  auto default_cube_marker = [&](
                               const auto & width, const auto & depth,
                               const auto & color = marker_utils::colors::green()) {
    return autoware_utils::create_default_marker(
      "map", now, ns + "_cube", ++id, visualization_msgs::msg::Marker::CUBE,
      autoware_utils::create_marker_scale(width, depth, 1.0), color);
  };

  MarkerArray marker_array;
  marker_array.markers.reserve(
    obj_debug_vec.size() * 5);  // poly ego, text ego, poly obj, text obj, cube obj

  int32_t idx = {0};
  for (const auto & [uuid, info] : obj_debug_vec) {
    const auto color = info.is_safe ? marker_utils::colors::green() : marker_utils::colors::red();
    const auto poly_z = info.current_obj_pose.position.z;  // temporally

    const auto insert_polygon_marker = [&](const auto & polygon) {
      marker_array.markers.emplace_back();
      auto & polygon_marker = marker_array.markers.back();
      polygon_marker = default_line_marker(color);
      polygon_marker.points.reserve(polygon.outer().size());
      for (const auto & p : polygon.outer()) {
        polygon_marker.points.push_back(autoware_utils::create_point(p.x(), p.y(), poly_z));
      }
    };

    insert_polygon_marker(info.extended_ego_polygon);
    insert_polygon_marker(info.extended_obj_polygon);

    const auto str_idx = std::to_string(++idx);
    const auto insert_text_marker = [&](const auto & pose) {
      marker_array.markers.emplace_back();
      auto & text_marker = marker_array.markers.back();
      text_marker = default_text_marker();
      text_marker.text = str_idx;
      text_marker.pose = pose;
    };

    insert_text_marker(info.expected_ego_pose);
    insert_text_marker(info.expected_obj_pose);

    const auto insert_cube_marker = [&](const auto & pose) {
      marker_array.markers.emplace_back();
      auto & cube_marker = marker_array.markers.back();
      cube_marker = default_cube_marker(1.0, 1.0, color);
      cube_marker.pose = pose;
    };
    insert_cube_marker(info.current_obj_pose);
  }
  return marker_array;
}

MarkerArray showPredictedPath(
  const behavior_path_planner::utils::path_safety_checker::CollisionCheckDebugMap & obj_debug_vec,
  std::string && ns)
{
  int32_t id{0};
  const auto current_time{rclcpp::Clock{RCL_ROS_TIME}.now()};
  const auto arrow_marker_scale = autoware_utils::create_marker_scale(1.0, 0.3, 0.3);
  const auto default_arrow_marker = [&](const auto & color) {
    return autoware_utils::create_default_marker(
      "map", current_time, ns, ++id, Marker::ARROW, arrow_marker_scale, color);
  };

  MarkerArray marker_array;
  marker_array.markers.reserve(std::accumulate(
    obj_debug_vec.cbegin(), obj_debug_vec.cend(), 0UL,
    [&](const auto current_sum, const auto & obj_debug) {
      const auto & [uuid, info] = obj_debug;
      return current_sum + info.ego_predicted_path.size() + info.obj_predicted_path.size() + 2;
    }));

  for (const auto & [uuid, info] : obj_debug_vec) {
    const auto insert_marker = [&](const auto & path, const auto & color) {
      for (const auto & pose : path) {
        marker_array.markers.emplace_back();
        auto & marker = marker_array.markers.back();
        marker = default_arrow_marker(color);
        marker.pose = pose.pose;
      }
    };

    insert_marker(info.ego_predicted_path, marker_utils::colors::aqua());
    insert_marker(info.obj_predicted_path, marker_utils::colors::yellow());
    const auto insert_expected_pose_marker = [&](const auto & pose, const auto & color) {
      // instead of checking for distance, inserting a new marker might be more efficient
      marker_array.markers.emplace_back();
      auto & marker = marker_array.markers.back();
      marker = default_arrow_marker(color);
      marker.pose = pose;
      marker.pose.position.z += 0.05;
    };

    insert_expected_pose_marker(info.expected_ego_pose, marker_utils::colors::red());
    insert_expected_pose_marker(info.expected_obj_pose, marker_utils::colors::red());
  }
  return marker_array;
}

MarkerArray showSafetyCheckInfo(
  const behavior_path_planner::utils::path_safety_checker::CollisionCheckDebugMap & obj_debug_vec,
  std::string && ns)
{
  int32_t id{0};
  auto default_text_marker = [&]() {
    return autoware_utils::create_default_marker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, ++id, Marker::TEXT_VIEW_FACING,
      autoware_utils::create_marker_scale(0.5, 0.5, 0.5), marker_utils::colors::aqua());
  };

  MarkerArray marker_array;

  marker_array.markers.reserve(obj_debug_vec.size());

  int idx{0};

  for (const auto & [uuid, info] : obj_debug_vec) {
    auto safety_check_info_text = default_text_marker();
    safety_check_info_text.pose = info.current_obj_pose;

    std::ostringstream ss;

    ss << "Idx: " << ++idx << "\nUnsafe reason: " << info.unsafe_reason
       << "\nRSS dist: " << std::setprecision(4) << info.rss_longitudinal
       << "\nEgo to obj: " << info.inter_vehicle_distance
       << "\nExtended polygon: " << (info.is_front ? "ego" : "object")
       << "\nExtended polygon lateral offset: " << info.lat_offset
       << "\nExtended polygon forward longitudinal offset: " << info.forward_lon_offset
       << "\nExtended polygon backward longitudinal offset: " << info.backward_lon_offset
       << "\nLast checked position: " << (info.is_front ? "obj in front ego" : "obj at back ego")
       << "\nSafe: " << (info.is_safe ? "Yes" : "No");

    safety_check_info_text.text = ss.str();
    marker_array.markers.push_back(safety_check_info_text);
  }
  return marker_array;
}

MarkerArray createPointsMarkerArray(
  const std::vector<geometry_msgs::msg::Point> & points, const std::string & ns)
{
  MarkerArray msg;

  auto marker = autoware_utils::create_default_marker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::POINTS,
    autoware_utils::create_marker_scale(0.3, 0.3, 0.3),
    autoware_utils::create_marker_color(1.0, 0.0, 0.0, 0.999));
  for (const auto & p : points) {
    marker.points.push_back(p);
  }
  msg.markers.push_back(marker);

  return msg;
}

MarkerArray create_pointcloud_object_marker_array(
  const PointCloudObjects & objects, const std::string & ns)
{
  MarkerArray msg;

  size_t i = 0L;
  for (const auto & object : objects) {
    {
      auto marker = autoware_utils::create_default_marker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i++, Marker::SPHERE,
        autoware_utils::create_marker_scale(1.0, 1.0, 1.0),
        autoware_utils::create_marker_color(1.0, 0.0, 0.42, 0.999));
      marker.pose.position = object.position;
      msg.markers.push_back(marker);
    }

    {
      auto marker = autoware_utils::create_default_marker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns + "_text", i++,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
        autoware_utils::create_marker_scale(0.5, 0.5, 0.5),
        autoware_utils::create_marker_color(0.16, 1.0, 0.69, 0.999));

      std::ostringstream ss;
      ss << "RelativeDistance:" << object.relative_distance << "\nVelocity:" << object.velocity;

      marker.text = ss.str();
      marker.pose.position = object.position;
      marker.pose.position.z += 1.0;
      msg.markers.push_back(marker);
    }
  }

  return msg;
}
}  // namespace autoware::rear_obstacle_checker::utils

#endif  // UTILS_HPP_
