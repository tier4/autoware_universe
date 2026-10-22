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

#include "autoware/mppi_optimizer/input_collision_gate.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

namespace autoware::mppi_optimizer
{
namespace
{

Trajectory make_trajectory(const std::vector<double> & x_positions)
{
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  for (const double x : x_positions) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.orientation.w = 1.0;
    trajectory.points.push_back(point);
  }
  return trajectory;
}

TrackedObjects make_static_object(const double x)
{
  TrackedObjects objects;
  objects.header.frame_id = "map";
  autoware_perception_msgs::msg::TrackedObject object;
  object.kinematics.pose_with_covariance.pose.position.x = x;
  object.kinematics.pose_with_covariance.pose.orientation.w = 1.0;
  object.shape.dimensions.x = 0.4;
  object.shape.dimensions.y = 0.4;
  objects.objects.push_back(object);
  return objects;
}

FirstOrderDubinsMppiVehicleParams make_vehicle()
{
  FirstOrderDubinsMppiVehicleParams vehicle;
  vehicle.ego_length = 1.0F;
  vehicle.ego_width = 1.0F;
  vehicle.ego_axle_to_box_center = 0.0F;
  return vehicle;
}

TEST(InputCollisionGate, DetectsRoadBorderBetweenTrajectoryPoints)
{
  const auto trajectory = make_trajectory({0.0, 3.0});
  const std::vector<Segment> borders{{1.5F, -2.0F, 1.5F, 2.0F}};
  const auto collision = checkInputTrajectoryCollisions(
    trajectory, TrackedObjects{}, borders, make_vehicle(), 0.0F, 0.0F, {true, false});
  EXPECT_TRUE(collision.road_border);
  EXPECT_FALSE(collision.static_object);
}

TEST(InputCollisionGate, DetectsCentimeterScaleBorderAtLargeMapCoordinates)
{
  constexpr double map_x = 88800.12345;
  auto trajectory = make_trajectory({map_x, map_x + 0.02});
  for (auto & point : trajectory.points) {
    point.pose.position.y = 42800.54321;
  }
  const std::vector<Segment> borders{{map_x + 0.01, 42800.0, map_x + 0.01, 42801.0}};
  const auto collision = checkInputTrajectoryCollisions(
    trajectory, TrackedObjects{}, borders, make_vehicle(), 0.0F, 0.0F, {true, false});
  EXPECT_TRUE(collision.road_border);

  const double just_outside = trajectory.points.front().pose.position.y + 0.501;
  const std::vector<Segment> clear_border{{map_x - 1.0, just_outside, map_x + 1.0, just_outside}};
  const auto clear = checkInputTrajectoryCollisions(
    trajectory, TrackedObjects{}, clear_border, make_vehicle(), 0.0F, 0.0F, {true, false});
  EXPECT_FALSE(clear.road_border);
}

TEST(InputCollisionGate, DetectsStaticObjectBetweenTrajectoryPoints)
{
  const auto trajectory = make_trajectory({0.0, 3.0});
  const auto collision = checkInputTrajectoryCollisions(
    trajectory, make_static_object(1.5), {}, make_vehicle(), 0.0F, 0.0F, {false, true});
  EXPECT_FALSE(collision.road_border);
  EXPECT_TRUE(collision.static_object);
}

TEST(InputCollisionGate, RespectsIndependentConditionToggles)
{
  const auto trajectory = make_trajectory({0.0, 3.0});
  const auto objects = make_static_object(1.5);
  const std::vector<Segment> borders{{1.5F, -2.0F, 1.5F, 2.0F}};
  EXPECT_FALSE(checkInputTrajectoryCollisions(
                 trajectory, objects, borders, make_vehicle(), 0.0F, 0.0F, {false, false})
                 .triggered());
  EXPECT_TRUE(checkInputTrajectoryCollisions(
                trajectory, objects, borders, make_vehicle(), 0.0F, 0.0F, {true, false})
                .road_border);
  EXPECT_TRUE(checkInputTrajectoryCollisions(
                trajectory, objects, borders, make_vehicle(), 0.0F, 0.0F, {false, true})
                .static_object);
}

TEST(InputCollisionGate, UsesConfiguredCollisionMargins)
{
  const auto trajectory = make_trajectory({0.0, 0.0});
  const auto objects = make_static_object(1.2);
  const std::vector<Segment> borders{{1.0F, -2.0F, 1.0F, 2.0F}};
  EXPECT_FALSE(checkInputTrajectoryCollisions(
                 trajectory, objects, borders, make_vehicle(), 0.0F, 0.0F, {true, true})
                 .triggered());
  EXPECT_TRUE(checkInputTrajectoryCollisions(
                trajectory, objects, borders, make_vehicle(), 0.0F, 0.6F, {true, false})
                .road_border);
  EXPECT_TRUE(checkInputTrajectoryCollisions(
                trajectory, objects, borders, make_vehicle(), 0.6F, 0.0F, {false, true})
                .static_object);
}

TEST(InputCollisionGate, IgnoresCollisionOutsideModifiablePrefix)
{
  std::vector<double> x_positions;
  for (std::size_t index = 0; index <= 80U; ++index) {
    x_positions.push_back(2.0 * static_cast<double>(index));
  }
  const std::vector<Segment> borders{{160.0F, -2.0F, 160.0F, 2.0F}};
  const auto collision = checkInputTrajectoryCollisions(
    make_trajectory(x_positions), TrackedObjects{}, borders, make_vehicle(), 0.0F, 0.0F,
    {true, false});
  EXPECT_FALSE(collision.triggered());
}

}  // namespace
}  // namespace autoware::mppi_optimizer
