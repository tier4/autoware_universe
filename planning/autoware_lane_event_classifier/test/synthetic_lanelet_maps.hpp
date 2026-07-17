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

// Shared synthetic lanelet maps and input builders for the unit tests.

#ifndef SYNTHETIC_LANELET_MAPS_HPP_
#define SYNTHETIC_LANELET_MAPS_HPP_

#include <autoware_lane_event_classifier/detail/geometry_utils.hpp>
#include <autoware_lane_event_classifier/types.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <vector>

namespace autoware::lane_event_classifier::test_maps
{

inline void mark_road(lanelet::Lanelet & lane)
{
  lane.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  lane.attributes()[lanelet::AttributeName::Location] = lanelet::AttributeValueString::Urban;
}

// Single straight lanelet: x=[0,10], y=[-2,2] (width 4 m, centred on Y=0).
inline lanelet::LaneletMapPtr make_single_lane_map(lanelet::Id & lane_id_out)
{
  lanelet::Point3d lp1(lanelet::utils::getId(), 0.0, 2.0, 0.0);
  lanelet::Point3d lp2(lanelet::utils::getId(), 10.0, 2.0, 0.0);
  lanelet::Point3d rp1(lanelet::utils::getId(), 0.0, -2.0, 0.0);
  lanelet::Point3d rp2(lanelet::utils::getId(), 10.0, -2.0, 0.0);
  lanelet::LineString3d left_bound(lanelet::utils::getId(), {lp1, lp2});
  lanelet::LineString3d right_bound(lanelet::utils::getId(), {rp1, rp2});
  lanelet::Lanelet lane(lanelet::utils::getId(), left_bound, right_bound);
  lane_id_out = lane.id();
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lane);
  return map;
}

// Two lanelets connected end-to-end (lane_a x=[0,10] -> lane_b x=[10,20]), sharing the boundary
// points at x=10 so the routing graph reports lane_b as a next lane of lane_a.
inline lanelet::LaneletMapPtr make_next_lane_map(lanelet::Id & id_a, lanelet::Id & id_b)
{
  lanelet::Point3d l0(lanelet::utils::getId(), 0.0, 2.0, 0.0);
  lanelet::Point3d r0(lanelet::utils::getId(), 0.0, -2.0, 0.0);
  lanelet::Point3d l_mid(lanelet::utils::getId(), 10.0, 2.0, 0.0);
  lanelet::Point3d r_mid(lanelet::utils::getId(), 10.0, -2.0, 0.0);
  lanelet::Point3d l1(lanelet::utils::getId(), 20.0, 2.0, 0.0);
  lanelet::Point3d r1(lanelet::utils::getId(), 20.0, -2.0, 0.0);

  lanelet::LineString3d left_a(lanelet::utils::getId(), {l0, l_mid});
  lanelet::LineString3d right_a(lanelet::utils::getId(), {r0, r_mid});
  lanelet::Lanelet lane_a(lanelet::utils::getId(), left_a, right_a);
  mark_road(lane_a);

  lanelet::LineString3d left_b(lanelet::utils::getId(), {l_mid, l1});
  lanelet::LineString3d right_b(lanelet::utils::getId(), {r_mid, r1});
  lanelet::Lanelet lane_b(lanelet::utils::getId(), left_b, right_b);
  mark_road(lane_b);

  id_a = lane_a.id();
  id_b = lane_b.id();
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lane_a);
  map->add(lane_b);
  return map;
}

// Two side-by-side lanelets: lane_a y=[-2,2], lane_b y=[2,6], both x=[0,10].
inline lanelet::LaneletMapPtr make_parallel_map(lanelet::Id & id_a, lanelet::Id & id_b)
{
  auto make_lane = [](double y_right, double y_left) {
    lanelet::Point3d lp1(lanelet::utils::getId(), 0.0, y_left, 0.0);
    lanelet::Point3d lp2(lanelet::utils::getId(), 10.0, y_left, 0.0);
    lanelet::Point3d rp1(lanelet::utils::getId(), 0.0, y_right, 0.0);
    lanelet::Point3d rp2(lanelet::utils::getId(), 10.0, y_right, 0.0);
    lanelet::LineString3d lb(lanelet::utils::getId(), {lp1, lp2});
    lanelet::LineString3d rb(lanelet::utils::getId(), {rp1, rp2});
    return lanelet::Lanelet(lanelet::utils::getId(), lb, rb);
  };
  auto lane_a = make_lane(-2.0, 2.0);
  auto lane_b = make_lane(2.0, 6.0);
  id_a = lane_a.id();
  id_b = lane_b.id();
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lane_a);
  map->add(lane_b);
  return map;
}

// Single straight lanelet (x=[0,10], y=[-2,2]) tagged as a turn-direction lane.
inline lanelet::LaneletMapPtr make_turn_lane_map(lanelet::Id & lane_id_out)
{
  auto map = make_single_lane_map(lane_id_out);
  auto lane = map->laneletLayer.get(lane_id_out);
  lane.attributes()["turn_direction"] = "left";
  return map;
}

// Single straight lanelet whose LEFT bound (y=2) is a virtual linestring, RIGHT bound (y=-2) solid.
inline lanelet::LaneletMapPtr make_virtual_left_bound_map(lanelet::Id & lane_id_out)
{
  lanelet::Point3d lp1(lanelet::utils::getId(), 0.0, 2.0, 0.0);
  lanelet::Point3d lp2(lanelet::utils::getId(), 10.0, 2.0, 0.0);
  lanelet::Point3d rp1(lanelet::utils::getId(), 0.0, -2.0, 0.0);
  lanelet::Point3d rp2(lanelet::utils::getId(), 10.0, -2.0, 0.0);
  lanelet::LineString3d left_bound(lanelet::utils::getId(), {lp1, lp2});
  left_bound.attributes()["type"] = "virtual";
  lanelet::LineString3d right_bound(lanelet::utils::getId(), {rp1, rp2});
  right_bound.attributes()["type"] = "line_thin";
  lanelet::Lanelet lane(lanelet::utils::getId(), left_bound, right_bound);
  lane_id_out = lane.id();
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lane);
  return map;
}

// A road lane (y=[-2,2]) with a road-shoulder lanelet (y=[2,4]) beside it, sharing the y=2 bound.
inline lanelet::LaneletMapPtr make_road_and_shoulder_map(
  lanelet::Id & road_id, lanelet::Id & shoulder_id)
{
  lanelet::Point3d road_rp1(lanelet::utils::getId(), 0.0, -2.0, 0.0);
  lanelet::Point3d road_rp2(lanelet::utils::getId(), 10.0, -2.0, 0.0);
  lanelet::Point3d shared1(lanelet::utils::getId(), 0.0, 2.0, 0.0);
  lanelet::Point3d shared2(lanelet::utils::getId(), 10.0, 2.0, 0.0);
  lanelet::Point3d shoulder_lp1(lanelet::utils::getId(), 0.0, 4.0, 0.0);
  lanelet::Point3d shoulder_lp2(lanelet::utils::getId(), 10.0, 4.0, 0.0);

  lanelet::LineString3d road_right(lanelet::utils::getId(), {road_rp1, road_rp2});
  lanelet::LineString3d shared_bound(lanelet::utils::getId(), {shared1, shared2});
  lanelet::LineString3d shoulder_left(lanelet::utils::getId(), {shoulder_lp1, shoulder_lp2});

  lanelet::Lanelet road(lanelet::utils::getId(), shared_bound, road_right);
  mark_road(road);
  lanelet::Lanelet shoulder(lanelet::utils::getId(), shoulder_left, shared_bound);
  shoulder.attributes()[lanelet::AttributeName::Subtype] = "road_shoulder";

  road_id = road.id();
  shoulder_id = shoulder.id();
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(road);
  map->add(shoulder);
  return map;
}

// stamp_sec / stamp_nanosec passed as integers to avoid float-to-nanosec rounding.
inline LaneEventInput make_input(
  const std::vector<lanelet::Id> & route_lane_ids, double x, double y, int32_t stamp_sec,
  uint32_t stamp_nanosec)
{
  LaneEventInput input;

  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->pose.pose.position.x = x;
  odom->pose.pose.position.y = y;
  odom->header.stamp.sec = stamp_sec;
  odom->header.stamp.nanosec = stamp_nanosec;
  input.odometry_ptr = odom;

  auto route = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>();
  route->segments.reserve(route_lane_ids.size());
  for (const auto id : route_lane_ids) {
    autoware_planning_msgs::msg::LaneletSegment seg;
    seg.preferred_primitive.id = id;
    route->segments.push_back(seg);
  }
  input.route_ptr = route;
  input.footprint = compute_footprint(input);  // vehicle_info null → single centre point
  return input;
}

// A static (or optionally moving) bounding-box object at (x, y). Identity orientation keeps the box
// axis-aligned. Used by the lane-crossing tests to place an obstacle to avoid.
inline autoware_perception_msgs::msg::PredictedObject make_object(
  double x, double y, double size_x = 2.0, double size_y = 2.0, double speed_mps = 0.0)
{
  autoware_perception_msgs::msg::PredictedObject object;
  auto & pose = object.kinematics.initial_pose_with_covariance.pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  object.kinematics.initial_twist_with_covariance.twist.linear.x = speed_mps;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = size_x;
  object.shape.dimensions.y = size_y;
  object.shape.dimensions.z = 1.5;
  return object;
}

inline autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr make_objects(
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects)
{
  auto message = std::make_shared<autoware_perception_msgs::msg::PredictedObjects>();
  message->objects = objects;
  return message;
}

// Builds an input carrying an explicit planned trajectory, footprint, turn indicator, and optional
// perceived objects — used by the trajectory-driven lane-change and lane-crossing tests. The ego
// pose is the odom position; timing comes from the stamp (advance it between cycles to drive the
// debounce timers).
inline LaneEventInput make_trajectory_input(
  const std::vector<lanelet::Id> & route_lane_ids, const lanelet::BasicPoint2d & ego,
  int32_t stamp_sec, uint32_t stamp_nanosec,
  const std::vector<lanelet::BasicPoint2d> & trajectory_points,
  const std::vector<lanelet::BasicPoint2d> & footprint,
  uint8_t turn_indicator = autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE,
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr = nullptr)
{
  LaneEventInput input;

  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->pose.pose.position.x = ego.x();
  odom->pose.pose.position.y = ego.y();
  odom->header.stamp.sec = stamp_sec;
  odom->header.stamp.nanosec = stamp_nanosec;
  input.odometry_ptr = odom;

  auto route = std::make_shared<autoware_planning_msgs::msg::LaneletRoute>();
  route->segments.reserve(route_lane_ids.size());
  for (const auto id : route_lane_ids) {
    autoware_planning_msgs::msg::LaneletSegment seg;
    seg.preferred_primitive.id = id;
    route->segments.push_back(seg);
  }
  input.route_ptr = route;

  auto trajectory = std::make_shared<autoware_planning_msgs::msg::Trajectory>();
  trajectory->header.stamp.sec = stamp_sec;
  trajectory->header.stamp.nanosec = stamp_nanosec;
  trajectory->points.reserve(trajectory_points.size());
  for (const auto & point : trajectory_points) {
    autoware_planning_msgs::msg::TrajectoryPoint trajectory_point;
    trajectory_point.pose.position.x = point.x();
    trajectory_point.pose.position.y = point.y();
    trajectory->points.push_back(trajectory_point);
  }
  input.trajectory_ptr = trajectory;

  input.footprint = footprint;
  input.turn_indicator = turn_indicator;
  input.objects_ptr = objects_ptr;
  return input;
}

inline constexpr uint32_t ms(uint32_t milliseconds)
{
  return milliseconds * 1'000'000u;
}

}  // namespace autoware::lane_event_classifier::test_maps

#endif  // SYNTHETIC_LANELET_MAPS_HPP_
