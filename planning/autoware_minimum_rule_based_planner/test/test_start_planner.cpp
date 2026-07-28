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

#include "start_planner.hpp"

#include <rclcpp/logger.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
namespace
{

using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

TrajectoryPoint make_point(const double x, const double y)
{
  TrajectoryPoint pt;
  pt.pose.position.x = x;
  pt.pose.position.y = y;
  pt.pose.orientation.w = 1.0;
  pt.longitudinal_velocity_mps = 5.0f;
  return pt;
}

// Straight reference trajectory along +x at y = 0, 1 m spacing.
Trajectory make_reference_trajectory(const size_t num_points = 60)
{
  Trajectory trajectory;
  for (size_t i = 0; i < num_points; ++i) {
    trajectory.points.push_back(make_point(static_cast<double>(i), 0.0));
  }
  return trajectory;
}

// Candidate that goes straight at y = y_start until x = keep_length, merges linearly onto
// y = 0 at x = merge_end, then follows the reference until x = 50.
Trajectory make_candidate(const double y_start, const double keep_length, const double merge_end)
{
  Trajectory trajectory;
  for (double x = 0.0; x <= 50.0; x += 0.5) {
    double y = 0.0;
    if (x < keep_length) {
      y = y_start;
    } else if (x < merge_end) {
      y = y_start * (merge_end - x) / (merge_end - keep_length);
    }
    trajectory.points.push_back(make_point(x, y));
  }
  return trajectory;
}

geometry_msgs::msg::Pose make_pose(const double x, const double y)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

PredictedObjects::ConstSharedPtr make_objects(
  const std::vector<std::pair<double, double>> & positions, const double speed = 0.0)
{
  auto objects = std::make_shared<PredictedObjects>();
  for (const auto & [x, y] : positions) {
    PredictedObject object;
    object.kinematics.initial_pose_with_covariance.pose = make_pose(x, y);
    object.kinematics.initial_twist_with_covariance.twist.linear.x = speed;
    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = 1.0;
    object.shape.dimensions.y = 1.0;
    object.shape.dimensions.z = 1.0;
    objects->objects.push_back(object);
  }
  return objects;
}

VehicleInfo make_vehicle_info()
{
  VehicleInfo vehicle_info{};
  vehicle_info.max_longitudinal_offset_m = 1.0;
  vehicle_info.rear_overhang_m = 1.0;
  vehicle_info.vehicle_width_m = 1.8;  // footprint edge at |y_offset| = 0.9
  return vehicle_info;
}

StartPlannerParams make_params()
{
  StartPlannerParams params;
  params.enable = true;
  params.ego_stopped_velocity = 0.5;
  params.activation_lateral_offset = 0.5;
  params.finish_lateral_offset = 0.2;
  params.minimum_lateral_accel = 0.5;
  params.maximum_lateral_accel = 1.0;
  params.lateral_accel_sampling_num = 2;  // samples 0.5, 0.75, 1.0
  params.collision_check_margins = {2.0, 1.0};
  params.collision_check_extra_length = 5.0;
  params.object_velocity_threshold = 1.0;
  params.object_search_radius = 30.0;
  // large floor so that the collision check always covers the whole merge section in tests
  params.minimum_shift_distance = 30.0;
  params.min_speed_for_curvature = 2.77;
  return params;
}

StartPlanner make_planner()
{
  return StartPlanner(rclcpp::get_logger("test_start_planner"), make_vehicle_info());
}

//! generator returning the same diagonal candidate for every lateral acceleration
ShiftCandidateGenerator constant_generator(const double y_start)
{
  return [y_start](double /*lateral_accel*/) -> std::optional<Trajectory> {
    return make_candidate(y_start, 0.0, 10.0);
  };
}

}  // namespace

// The planner must not interfere with normal driving: pull out only concerns the
// departure from standstill with a lateral offset.
TEST(StartPlannerTest, NotApplicableWhenMoving)
{
  auto planner = make_planner();
  size_t generator_calls = 0;
  const auto generator = [&](double) -> std::optional<Trajectory> {
    ++generator_calls;
    return make_candidate(3.0, 0.0, 10.0);
  };

  const auto result = planner.plan(
    make_reference_trajectory(), make_pose(0.0, 3.0), /*ego_velocity=*/3.0, nullptr, generator,
    make_params());

  EXPECT_EQ(result.status, StartPlannerResult::Status::NOT_APPLICABLE);
  EXPECT_EQ(generator_calls, 0u);
  EXPECT_FALSE(planner.is_pull_out_active());
}

TEST(StartPlannerTest, NotApplicableWhenAlreadyOnPath)
{
  auto planner = make_planner();
  const auto result = planner.plan(
    make_reference_trajectory(), make_pose(0.0, 0.1), /*ego_velocity=*/0.0, nullptr,
    constant_generator(0.1), make_params());

  EXPECT_EQ(result.status, StartPlannerResult::Status::NOT_APPLICABLE);
  EXPECT_FALSE(planner.is_pull_out_active());
}

// First-fit: with no obstacle, the gentlest (first sampled) candidate must be adopted with the
// largest (safest) margin, and the turn signal must point toward the shift direction.
TEST(StartPlannerTest, AdoptsFirstCandidateWhenClear)
{
  auto planner = make_planner();
  std::vector<double> requested_accels;
  const auto generator = [&](double lateral_accel) -> std::optional<Trajectory> {
    requested_accels.push_back(lateral_accel);
    return make_candidate(3.0, 0.0, 10.0);
  };

  const auto result = planner.plan(
    make_reference_trajectory(), make_pose(0.0, 3.0), /*ego_velocity=*/0.0, nullptr, generator,
    make_params());

  ASSERT_EQ(result.status, StartPlannerResult::Status::PLANNED);
  ASSERT_TRUE(result.trajectory.has_value());
  EXPECT_DOUBLE_EQ(result.selected_lateral_accel, 0.5);  // first sample
  EXPECT_DOUBLE_EQ(result.selected_margin, 2.0);         // largest margin
  ASSERT_EQ(requested_accels.size(), 1u);                // first-fit: no further sampling
  // ego is left of the reference path -> shifting right
  EXPECT_EQ(result.turn_indicators_command, TurnIndicatorsCommand::ENABLE_RIGHT);
  EXPECT_TRUE(planner.is_pull_out_active());
}

// A candidate whose shift section collides with a static object must be rejected, and the next
// sampled candidate that avoids the object must be adopted instead.
TEST(StartPlannerTest, RejectsCollidingCandidateAndAdoptsNextSample)
{
  auto planner = make_planner();
  const auto generator = [&](double lateral_accel) -> std::optional<Trajectory> {
    if (lateral_accel < 0.6) {
      // gentle candidate: keeps y = 3 until x = 20 -> passes right next to the object
      return make_candidate(3.0, 20.0, 30.0);
    }
    // sharp candidate: merges onto y = 0 by x = 10 -> stays away from the object
    return make_candidate(3.0, 0.0, 10.0);
  };
  // object at (10, 4): overlaps the gentle candidate footprint (edge at y = 3.9),
  // 2.6 m away from the sharp candidate footprint (edge at y = 0.9)
  const auto objects = make_objects({{10.0, 4.0}});

  const auto result = planner.plan(
    make_reference_trajectory(), make_pose(0.0, 3.0), /*ego_velocity=*/0.0, objects, generator,
    make_params());

  ASSERT_EQ(result.status, StartPlannerResult::Status::PLANNED);
  EXPECT_DOUBLE_EQ(result.selected_lateral_accel, 0.75);  // second sample
  EXPECT_DOUBLE_EQ(result.selected_margin, 2.0);
}

// When no candidate passes with the large margin, the margin must be relaxed stepwise instead of
// giving up (v4 start_planner's staged margin search).
TEST(StartPlannerTest, RelaxesMarginWhenLargeMarginFails)
{
  auto planner = make_planner();
  // all candidates merge onto y = 0 by x = 10 (footprint edge at y = 0.9 afterwards)
  const auto generator = constant_generator(3.0);
  // object at (20, 2.5): lateral gap to the merged footprint = 2.5 - 0.5 - 0.9 = 1.1 m
  // -> collides with margin 2.0, passes with margin 1.0
  const auto objects = make_objects({{20.0, 2.5}});

  const auto result = planner.plan(
    make_reference_trajectory(), make_pose(0.0, 3.0), /*ego_velocity=*/0.0, objects, generator,
    make_params());

  ASSERT_EQ(result.status, StartPlannerResult::Status::PLANNED);
  EXPECT_DOUBLE_EQ(result.selected_margin, 1.0);
  EXPECT_DOUBLE_EQ(result.selected_lateral_accel, 0.5);
}

// When every candidate collides at every margin, the departure must be suppressed (BLOCKED)
// rather than driving into the obstacle, while the intent (turn signal) is kept.
TEST(StartPlannerTest, BlockedWhenAllCandidatesCollide)
{
  auto planner = make_planner();
  const auto generator = constant_generator(3.0);
  // object sitting on the merge section of every candidate
  const auto objects = make_objects({{5.0, 1.5}});

  const auto result = planner.plan(
    make_reference_trajectory(), make_pose(0.0, 3.0), /*ego_velocity=*/0.0, objects, generator,
    make_params());

  EXPECT_EQ(result.status, StartPlannerResult::Status::BLOCKED);
  EXPECT_FALSE(result.trajectory.has_value());
  EXPECT_EQ(result.turn_indicators_command, TurnIndicatorsCommand::ENABLE_RIGHT);
  EXPECT_TRUE(planner.is_pull_out_active());
}

// Only static objects matter for the pull out path: a fast object at the same position must not
// block the departure (dynamic objects are handled downstream by the obstacle stop modifier).
TEST(StartPlannerTest, IgnoresMovingObjects)
{
  auto planner = make_planner();
  const auto generator = constant_generator(3.0);
  const auto objects = make_objects({{5.0, 1.5}}, /*speed=*/5.0);

  const auto result = planner.plan(
    make_reference_trajectory(), make_pose(0.0, 3.0), /*ego_velocity=*/0.0, objects, generator,
    make_params());

  EXPECT_EQ(result.status, StartPlannerResult::Status::PLANNED);
}

TEST(StartPlannerTest, TurnSignalLeftWhenRightOfPath)
{
  auto planner = make_planner();
  const auto result = planner.plan(
    make_reference_trajectory(), make_pose(0.0, -3.0), /*ego_velocity=*/0.0, nullptr,
    constant_generator(-3.0), make_params());

  ASSERT_EQ(result.status, StartPlannerResult::Status::PLANNED);
  EXPECT_EQ(result.turn_indicators_command, TurnIndicatorsCommand::ENABLE_LEFT);
}

// Once activated, the pull out must stay active while ego is moving along the shift (the
// activation stopped-velocity condition must not deactivate it), and must finish only when the
// lateral offset converges onto the reference path.
TEST(StartPlannerTest, StaysActiveWhileMovingAndFinishesOnConvergence)
{
  auto planner = make_planner();
  const auto generator = constant_generator(3.0);
  const auto params = make_params();
  const auto reference = make_reference_trajectory();

  // activation at standstill
  auto result = planner.plan(reference, make_pose(0.0, 3.0), 0.0, nullptr, generator, params);
  ASSERT_EQ(result.status, StartPlannerResult::Status::PLANNED);

  // still active while moving with a remaining offset
  result = planner.plan(reference, make_pose(3.0, 2.0), 2.0, nullptr, generator, params);
  EXPECT_EQ(result.status, StartPlannerResult::Status::PLANNED);
  EXPECT_TRUE(planner.is_pull_out_active());

  // finished once the offset converges below finish_lateral_offset
  result = planner.plan(reference, make_pose(12.0, 0.05), 3.0, nullptr, generator, params);
  EXPECT_EQ(result.status, StartPlannerResult::Status::NOT_APPLICABLE);
  EXPECT_FALSE(planner.is_pull_out_active());
}

// utils-level checks

TEST(StartPlannerUtilsTest, StaticObjectFilter)
{
  const auto objects_msg = [] {
    auto slow = make_objects({{5.0, 0.0}}, 0.5);
    auto fast = make_objects({{6.0, 0.0}}, 3.0);
    auto far = make_objects({{100.0, 0.0}}, 0.0);
    PredictedObjects merged;
    merged.objects.push_back(slow->objects.front());
    merged.objects.push_back(fast->objects.front());
    merged.objects.push_back(far->objects.front());
    return merged;
  }();

  geometry_msgs::msg::Point ego_position;
  const auto polygons = start_planner_utils::get_static_object_polygons(
    objects_msg, ego_position, /*velocity_threshold=*/1.0, /*search_radius=*/30.0);

  EXPECT_EQ(polygons.size(), 1u);  // only the slow & near object remains
}

TEST(StartPlannerUtilsTest, CollisionOnlyWithinCheckLength)
{
  const auto candidate = make_candidate(0.0, 0.0, 0.0);  // straight along y = 0
  const auto objects = make_objects({{30.0, 0.0}});
  const auto polygons = start_planner_utils::get_static_object_polygons(
    *objects, geometry_msgs::msg::Point{}, 1.0, 100.0);
  const auto vehicle_info = make_vehicle_info();

  // object at x = 30 is beyond a 10 m check section but inside a 40 m one
  EXPECT_FALSE(
    start_planner_utils::has_collision(candidate.points, 10.0, vehicle_info, polygons, 0.5));
  EXPECT_TRUE(
    start_planner_utils::has_collision(candidate.points, 40.0, vehicle_info, polygons, 0.5));
}

}  // namespace autoware::minimum_rule_based_planner
