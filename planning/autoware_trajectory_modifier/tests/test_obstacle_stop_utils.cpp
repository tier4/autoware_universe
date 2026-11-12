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

#include "autoware/trajectory_modifier/trajectory_modifier_utils/obstacle_stop_utils.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{
using autoware::trajectory_modifier::utils::obstacle_stop::compute_cruise_velocity;
using autoware::trajectory_modifier::utils::obstacle_stop::evaluate_object_state;
using autoware::trajectory_modifier::utils::obstacle_stop::get_object_stopping_distance;
using autoware::trajectory_modifier::utils::obstacle_stop::get_safe_distance;
using autoware::trajectory_modifier::utils::obstacle_stop::ObjectDecelMap;
using autoware::trajectory_modifier::utils::obstacle_stop::ObjectRssState;
using autoware::trajectory_modifier::utils::obstacle_stop::ObjectType;
using autoware::trajectory_modifier::utils::obstacle_stop::select_cruise_target;
using autoware::trajectory_modifier::utils::obstacle_stop::TargetObject;
using autoware::trajectory_modifier::utils::obstacle_stop::TargetObjects;
using autoware::trajectory_modifier::utils::obstacle_stop::TrajectoryPoints;
using autoware::trajectory_modifier::utils::obstacle_stop::TrajectoryShape;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::TrajectoryPoint;

constexpr double stopped_vel_th = 0.3;
constexpr double car_decel = 2.5;
constexpr double ego_front_offset = 4.0;
constexpr double tolerance = 1e-6;

TrajectoryPoints create_straight_trajectory(const double length, const double velocity)
{
  TrajectoryPoints trajectory;
  for (double x = 0.0; x <= length + 1e-6; x += 1.0) {
    TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = static_cast<float>(velocity);
    trajectory.push_back(point);
  }
  return trajectory;
}

TrajectoryShape create_shape(const double trajectory_length, const double ego_arc_length)
{
  TrajectoryShape shape;
  shape.trajectory_length = trajectory_length;
  shape.forward_traj_length = trajectory_length - ego_arc_length;
  shape.ego_front_offset = ego_front_offset;
  return shape;
}

// 4 m x 2 m box centered at (x, 0); the nearest vertex along +x is at x - 2.
PredictedObject create_car(const double x, const double velocity, const bool oncoming = false)
{
  PredictedObject object;
  auto & pose = object.kinematics.initial_pose_with_covariance.pose;
  pose.position.x = x;
  pose.orientation.z = oncoming ? 1.0 : 0.0;
  pose.orientation.w = oncoming ? 0.0 : 1.0;
  object.kinematics.initial_twist_with_covariance.twist.linear.x = velocity;
  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  object.shape.dimensions.z = 1.5;
  ObjectClassification classification;
  classification.label = ObjectClassification::CAR;
  classification.probability = 1.0;
  object.classification.push_back(classification);
  return object;
}

const ObjectDecelMap decel_map{{ObjectType::CAR, car_decel}};

// The stored overlap is the whole box: a fully overlapping car contacts at its rear face.
ObjectRssState evaluate(
  const PredictedObject & object, const TrajectoryPoints & trajectory,
  const TrajectoryShape & shape)
{
  TargetObject target(object);
  const auto x = object.kinematics.initial_pose_with_covariance.pose.position.x;
  auto & ring = target.intersection_polygon.outer();
  ring.emplace_back(x - 2.0, -1.0);
  ring.emplace_back(x + 2.0, -1.0);
  ring.emplace_back(x + 2.0, 1.0);
  ring.emplace_back(x - 2.0, 1.0);
  ring.emplace_back(x - 2.0, -1.0);
  return evaluate_object_state(target, trajectory, shape, decel_map, stopped_vel_th);
}
}  // namespace

TEST(GetObjectStoppingDistance, ForwardMovingObject)
{
  EXPECT_NEAR(
    get_object_stopping_distance(ObjectType::CAR, 10.0, decel_map, stopped_vel_th),
    10.0 * 10.0 / (2.0 * car_decel), tolerance);
}

TEST(GetObjectStoppingDistance, ZeroWhenStoppedOrOncoming)
{
  EXPECT_DOUBLE_EQ(
    get_object_stopping_distance(ObjectType::CAR, 0.1, decel_map, stopped_vel_th), 0.0);
  EXPECT_DOUBLE_EQ(
    get_object_stopping_distance(ObjectType::CAR, -5.0, decel_map, stopped_vel_th), 0.0);
}

TEST(GetObjectStoppingDistance, ZeroWhenTypeHasNoDeceleration)
{
  EXPECT_DOUBLE_EQ(
    get_object_stopping_distance(ObjectType::TRUCK, 10.0, decel_map, stopped_vel_th), 0.0);
}

TEST(EvaluateObjectState, MovingCarAhead)
{
  const auto trajectory = create_straight_trajectory(50.0, 10.0);
  const auto shape = create_shape(50.0, 0.0);

  const auto s = evaluate(create_car(20.0, 5.0), trajectory, shape);

  EXPECT_EQ(s.type, ObjectType::CAR);
  EXPECT_NEAR(s.arc_length, 18.0, tolerance);
  EXPECT_NEAR(s.gap, 18.0 - ego_front_offset, tolerance);
  EXPECT_NEAR(s.lon_vel, 5.0, tolerance);
  EXPECT_NEAR(s.stopping_distance, 5.0 * 5.0 / (2.0 * car_decel), tolerance);
  EXPECT_TRUE(s.is_dynamic);
}

TEST(EvaluateObjectState, StoppedCarIsStatic)
{
  const auto trajectory = create_straight_trajectory(50.0, 10.0);
  const auto shape = create_shape(50.0, 0.0);

  const auto s = evaluate(create_car(20.0, 0.0), trajectory, shape);

  EXPECT_NEAR(s.lon_vel, 0.0, tolerance);
  EXPECT_DOUBLE_EQ(s.stopping_distance, 0.0);
  EXPECT_FALSE(s.is_dynamic);
}

TEST(EvaluateObjectState, OncomingCarHasNegativeVelocity)
{
  const auto trajectory = create_straight_trajectory(50.0, 10.0);
  const auto shape = create_shape(50.0, 0.0);

  const auto s = evaluate(create_car(20.0, 5.0, true), trajectory, shape);

  EXPECT_NEAR(s.lon_vel, -5.0, tolerance);
  EXPECT_DOUBLE_EQ(s.stopping_distance, 0.0);
  EXPECT_TRUE(s.is_dynamic);
}

TEST(EvaluateObjectState, GapIsMeasuredFromEgoFront)
{
  const auto trajectory = create_straight_trajectory(50.0, 10.0);
  const auto shape = create_shape(50.0, 10.0);

  const auto ahead = evaluate(create_car(20.0, 5.0), trajectory, shape);
  const auto beside_ego = evaluate(create_car(14.0, 5.0), trajectory, shape);

  EXPECT_NEAR(ahead.gap, 18.0 - (10.0 + ego_front_offset), tolerance);
  EXPECT_NEAR(beside_ego.arc_length, 12.0, tolerance);
  EXPECT_DOUBLE_EQ(beside_ego.gap, 0.0);
}

TEST(ComputeCruiseVelocity, InverseOfSafeDistance)
{
  constexpr double ego_decel = 1.0;
  constexpr double reaction_time = 0.5;
  constexpr double safety_margin = 8.0;
  for (const double gap : {10.0, 20.0, 40.0}) {
    for (const double object_stopping_distance : {0.0, 5.0, 20.0}) {
      const auto v = compute_cruise_velocity(
        gap, ego_decel, object_stopping_distance, reaction_time, safety_margin);
      ASSERT_GT(v, 0.0);
      EXPECT_NEAR(
        get_safe_distance(v, ego_decel, object_stopping_distance, reaction_time, safety_margin),
        gap, 1e-9);
    }
  }
}

TEST(ComputeCruiseVelocity, ZeroWhenGapWithinStandstillDistance)
{
  EXPECT_DOUBLE_EQ(compute_cruise_velocity(8.0, 1.0, 0.0, 0.5, 8.0), 0.0);
  EXPECT_DOUBLE_EQ(compute_cruise_velocity(5.0, 1.0, 2.0, 0.5, 8.0), 0.0);
  EXPECT_DOUBLE_EQ(compute_cruise_velocity(0.0, 1.0, 0.0, 0.5, 8.0), 0.0);
}

TEST(ComputeCruiseVelocity, ZeroReactionTime)
{
  // v^2 / (2 * a) = gap - margin + d_obj
  EXPECT_NEAR(compute_cruise_velocity(20.0, 2.0, 4.0, 0.0, 8.0), std::sqrt(2.0 * 2.0 * 16.0), 1e-9);
}

TEST(ComputeCruiseVelocity, IncreasesWithGapAndObjectStoppingDistance)
{
  const auto base = compute_cruise_velocity(20.0, 1.0, 5.0, 0.5, 8.0);
  EXPECT_GT(compute_cruise_velocity(30.0, 1.0, 5.0, 0.5, 8.0), base);
  EXPECT_GT(compute_cruise_velocity(20.0, 1.0, 10.0, 0.5, 8.0), base);
  EXPECT_LT(compute_cruise_velocity(20.0, 1.0, 5.0, 1.0, 8.0), base);
  EXPECT_LT(compute_cruise_velocity(20.0, 0.5, 5.0, 0.5, 8.0), base);
}

namespace
{
TargetObject make_target(const double arc_length, const double lon_vel)
{
  TargetObject target(create_car(0.0, lon_vel));
  target.rss_state.arc_length = arc_length;
  target.rss_state.lon_vel = lon_vel;
  target.rss_state.is_dynamic = std::abs(lon_vel) > stopped_vel_th;
  return target;
}
}  // namespace

TEST(SelectCruiseTarget, NoTargetWithoutForwardMovingObject)
{
  EXPECT_FALSE(select_cruise_target({}).has_value());
  EXPECT_FALSE(select_cruise_target({make_target(20.0, 0.0), make_target(30.0, -5.0)}));
  EXPECT_FALSE(select_cruise_target({make_target(20.0, 0.2)}));
}

TEST(SelectCruiseTarget, NearestForwardMovingObject)
{
  const auto target = select_cruise_target(
    {make_target(40.0, 5.0), make_target(10.0, 0.0), make_target(15.0, -5.0),
     make_target(25.0, 3.0)});
  ASSERT_TRUE(target.has_value());
  EXPECT_NEAR(target->rss_state.arc_length, 25.0, tolerance);
  EXPECT_DOUBLE_EQ(target->rss_state.lon_vel, 3.0);
}
