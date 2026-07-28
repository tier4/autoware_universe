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

#include "goal_planner.hpp"

#include <rclcpp/logger.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
namespace
{

using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

// Straight reference trajectory along +x at y = 0, 1 m spacing (length 59 m).
Trajectory make_reference_trajectory(const size_t num_points = 60)
{
  Trajectory trajectory;
  for (size_t i = 0; i < num_points; ++i) {
    TrajectoryPoint pt;
    pt.pose.position.x = static_cast<double>(i);
    pt.pose.orientation.w = 1.0;
    pt.longitudinal_velocity_mps = 5.0f;
    trajectory.points.push_back(pt);
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

GoalPlannerParams make_params()
{
  GoalPlannerParams params;
  params.enable = true;
  params.activation_lateral_offset = 0.5;
  params.minimum_lateral_accel = 0.5;
  params.maximum_lateral_accel = 1.0;
  params.lateral_accel_sampling_num = 2;  // samples 0.5, 0.75, 1.0
  params.collision_check_margins = {2.0, 1.0};
  params.object_velocity_threshold = 1.0;
  params.object_search_radius = 50.0;
  params.turn_signal_distance = 30.0;
  params.minimum_shift_distance = 5.0;
  params.expected_parking_speed = 2.8;
  return params;
}

GoalPlanner make_planner()
{
  return GoalPlanner(rclcpp::get_logger("test_goal_planner"), make_vehicle_info());
}

}  // namespace

// A goal (nearly) on the path needs no pull over: the normal arrival (early stop fallback,
// distance ~0) applies.
TEST(GoalPlannerTest, NotApplicableWhenGoalOnPath)
{
  const auto planner = make_planner();
  const auto result = planner.plan(
    make_reference_trajectory(), make_pose(50.0, 0.1), make_pose(0.0, 0.0), nullptr, make_params());
  EXPECT_EQ(result.status, GoalPlannerResult::Status::NOT_APPLICABLE);
}

// A goal projecting beyond the trajectory end (route continues past the planning horizon) is
// not this planner's business: it must not morph the tail toward a far goal.
TEST(GoalPlannerTest, NotApplicableWhenGoalBeyondTrajectoryEnd)
{
  const auto planner = make_planner();
  const auto result = planner.plan(
    make_reference_trajectory(), make_pose(70.0, -3.0), make_pose(0.0, 0.0), nullptr,
    make_params());
  EXPECT_EQ(result.status, GoalPlannerResult::Status::NOT_APPLICABLE);
}

// First-fit: with no obstacle the gentlest (first sampled) candidate must be adopted with the
// largest (safest) margin, and the candidate must end exactly at the goal pose.
TEST(GoalPlannerTest, AdoptsFirstCandidateWhenClear)
{
  const auto planner = make_planner();
  const auto goal = make_pose(50.0, -3.0);
  const auto result =
    planner.plan(make_reference_trajectory(), goal, make_pose(10.0, 0.0), nullptr, make_params());

  ASSERT_EQ(result.status, GoalPlannerResult::Status::PLANNED);
  ASSERT_TRUE(result.trajectory.has_value());
  EXPECT_DOUBLE_EQ(result.selected_lateral_accel, 0.5);  // first sample
  EXPECT_DOUBLE_EQ(result.selected_margin, 2.0);         // largest margin
  const auto & last = result.trajectory->points.back().pose.position;
  EXPECT_NEAR(last.x, goal.position.x, 1e-6);
  EXPECT_NEAR(last.y, goal.position.y, 1e-6);
  // goal is right of the path and ego is within turn_signal_distance of the shift start
  EXPECT_EQ(result.turn_indicators_command, TurnIndicatorsCommand::ENABLE_RIGHT);
}

// The turn signal must stay off while ego is still far from the shift start.
TEST(GoalPlannerTest, TurnSignalOffWhenFarFromShiftStart)
{
  const auto planner = make_planner();
  // shift start = 50 - L(0.5) = 50 - 16.5 = 33.5; signal from 3.5 m onward -> ego at 0 is early
  const auto result = planner.plan(
    make_reference_trajectory(), make_pose(50.0, -3.0), make_pose(0.0, 0.0), nullptr,
    make_params());
  ASSERT_EQ(result.status, GoalPlannerResult::Status::PLANNED);
  EXPECT_EQ(result.turn_indicators_command, TurnIndicatorsCommand::NO_COMMAND);
}

// in_pull_over_approach tells the caller to stop re-connecting the trajectory to ego: it must be
// false while the pull over is merely planned ahead (the goal enters the 300 m trajectory long
// before the maneuver starts, and the shift to ego is still wanted there) and true once ego
// reaches the approach window.
TEST(GoalPlannerTest, PullOverApproachFlagFollowsTheShiftStart)
{
  const auto planner = make_planner();
  const auto goal = make_pose(50.0, -3.0);

  const auto far =
    planner.plan(make_reference_trajectory(), goal, make_pose(0.0, 0.0), nullptr, make_params());
  ASSERT_EQ(far.status, GoalPlannerResult::Status::PLANNED);
  EXPECT_FALSE(far.in_pull_over_approach);

  const auto near =
    planner.plan(make_reference_trajectory(), goal, make_pose(10.0, 0.0), nullptr, make_params());
  ASSERT_EQ(near.status, GoalPlannerResult::Status::PLANNED);
  EXPECT_TRUE(near.in_pull_over_approach);
}

TEST(GoalPlannerTest, TurnSignalLeftForLeftGoal)
{
  const auto planner = make_planner();
  const auto result = planner.plan(
    make_reference_trajectory(), make_pose(50.0, 3.0), make_pose(40.0, 0.0), nullptr,
    make_params());
  ASSERT_EQ(result.status, GoalPlannerResult::Status::PLANNED);
  EXPECT_EQ(result.turn_indicators_command, TurnIndicatorsCommand::ENABLE_LEFT);
}

// When no candidate passes with the large margin, the margin must be relaxed stepwise
// (v4 goal_planner's soft/hard margin search).
TEST(GoalPlannerTest, RelaxesMarginWhenLargeMarginFails)
{
  const auto planner = make_planner();
  // object below the goal: gap to the parked footprint edge = 6.0 - 0.5 - 3.9 = 1.6 m
  // -> every candidate collides with margin 2.0, passes with margin 1.0
  const auto objects = make_objects({{50.0, -6.0}});
  const auto result = planner.plan(
    make_reference_trajectory(), make_pose(50.0, -3.0), make_pose(10.0, 0.0), objects,
    make_params());

  ASSERT_EQ(result.status, GoalPlannerResult::Status::PLANNED);
  EXPECT_DOUBLE_EQ(result.selected_margin, 1.0);
  EXPECT_DOUBLE_EQ(result.selected_lateral_accel, 0.5);
}

// A goal occupied by a static object must not be approached: fall back to the early stop
// (stop short of the goal on the lane) instead of driving into the obstacle.
TEST(GoalPlannerTest, BlockedWhenGoalOccupied)
{
  const auto planner = make_planner();
  const auto objects = make_objects({{50.0, -3.0}});
  const auto result = planner.plan(
    make_reference_trajectory(), make_pose(50.0, -3.0), make_pose(10.0, 0.0), objects,
    make_params());

  EXPECT_EQ(result.status, GoalPlannerResult::Status::BLOCKED);
  EXPECT_FALSE(result.trajectory.has_value());
}

// Only static objects matter: a fast object at the goal must not block the pull over
// (dynamic objects are handled downstream by the obstacle stop modifier).
TEST(GoalPlannerTest, IgnoresMovingObjects)
{
  const auto planner = make_planner();
  const auto objects = make_objects({{50.0, -3.0}}, /*speed=*/5.0);
  const auto result = planner.plan(
    make_reference_trajectory(), make_pose(50.0, -3.0), make_pose(10.0, 0.0), objects,
    make_params());
  EXPECT_EQ(result.status, GoalPlannerResult::Status::PLANNED);
}

// utils-level checks

// The morphed tail must leave the reference smoothly and end exactly at the goal: head points
// untouched, no large point gaps, lateral offset bounded by the goal offset.
TEST(GoalPlannerUtilsTest, MakePullOverCandidateGeometry)
{
  const auto trajectory = make_reference_trajectory();
  const auto goal = make_pose(50.0, -3.0);
  const auto candidate =
    goal_planner_utils::make_pull_over_candidate(trajectory, goal, /*shift_length=*/15.0);

  ASSERT_TRUE(candidate.has_value());
  const auto & pts = candidate->points;
  ASSERT_GE(pts.size(), 2u);

  // ends exactly at the goal
  EXPECT_NEAR(pts.back().pose.position.x, 50.0, 1e-6);
  EXPECT_NEAR(pts.back().pose.position.y, -3.0, 1e-6);

  for (size_t i = 0; i < pts.size(); ++i) {
    const auto & p = pts.at(i).pose.position;
    // head (before the shift start at x = 35) unchanged on the reference line
    if (p.x < 34.9) {
      EXPECT_NEAR(p.y, 0.0, 1e-9) << "at x=" << p.x;
    }
    // lateral offset never exceeds the goal offset
    EXPECT_GE(p.y, -3.0 - 1e-6);
    EXPECT_LE(p.y, 1e-6);
    if (i > 0) {
      const auto & q = pts.at(i - 1).pose.position;
      EXPECT_LT(std::hypot(p.x - q.x, p.y - q.y), 2.0) << "gap at index " << i;
      EXPECT_GT(p.x, q.x - 1e-6) << "backward step at index " << i;
    }
  }
}

// Regression test for the unstable end orientation: depending on where the goal projects onto
// the point grid, the last morph point could end up nearly coincident with the appended goal
// point. Such a degenerate final segment makes the end yaw noisy across cycles (downstream
// smoothing derives yaw from segment direction) and can trip the trajectory deviation check on
// arrival. The final segment must always keep a minimum length.
TEST(GoalPlannerUtilsTest, FinalSegmentNeverDegenerate)
{
  const auto trajectory = make_reference_trajectory();
  // sweep the goal longitudinal position across the 1 m point grid
  for (const double goal_x : {50.0, 50.05, 50.2, 50.49, 50.5, 50.9, 51.0}) {
    const auto goal = make_pose(goal_x, -3.0);
    const auto candidate =
      goal_planner_utils::make_pull_over_candidate(trajectory, goal, /*shift_length=*/15.0);
    ASSERT_TRUE(candidate.has_value()) << "goal_x=" << goal_x;
    const auto & pts = candidate->points;
    ASSERT_GE(pts.size(), 2u);

    // ends exactly at the goal
    EXPECT_NEAR(pts.back().pose.position.x, goal_x, 1e-6);
    EXPECT_NEAR(pts.back().pose.position.y, -3.0, 1e-6);

    // the final segment keeps a minimum length so its direction (-> end yaw) is well-defined
    const auto & last = pts.back().pose.position;
    const auto & prev = pts.at(pts.size() - 2).pose.position;
    EXPECT_GE(std::hypot(last.x - prev.x, last.y - prev.y), 0.3 - 1e-6) << "goal_x=" << goal_x;
  }
}

// pin_trajectory_end is applied again after the EB smoother (which recomputes orientations and
// resamples on its own grid): it must remove regenerated near-duplicate tail points and restore
// the exact end pose.
TEST(GoalPlannerUtilsTest, PinTrajectoryEnd)
{
  auto trajectory = make_reference_trajectory();  // points at x = 0..59
  // simulate a smoother output whose tail nearly coincides with the end pose
  auto end_pose = make_pose(59.1, 0.0);
  end_pose.orientation.z = std::sin(0.05);  // distinct yaw to verify restoration
  end_pose.orientation.w = std::cos(0.05);

  goal_planner_utils::pin_trajectory_end(trajectory, end_pose, 0.3);

  const auto & last = trajectory.points.back();
  EXPECT_NEAR(last.pose.position.x, 59.1, 1e-9);
  EXPECT_NEAR(last.pose.orientation.z, std::sin(0.05), 1e-9);
  // the point at x = 59 (0.1 m from the end pose) must have been dropped
  const auto & prev = trajectory.points.at(trajectory.points.size() - 2);
  EXPECT_GE(
    std::hypot(
      last.pose.position.x - prev.pose.position.x, last.pose.position.y - prev.pose.position.y),
    0.3 - 1e-9);
}

// Regression test for the terminal kink on curved arrivals (rotary): the per-point lateral
// offset does not land exactly on the goal on a curve, and snapping the end onto the goal used
// to bend the final segment by >0.19 rad (the yaw deviation threshold), stopping the vehicle
// 1-2 m before the goal. The blended tail must approach along the goal heading without kinks.
TEST(GoalPlannerUtilsTest, NoTerminalKinkOnCurvedArrival)
{
  // left-curving arc, radius 15 m (rotary-like), 0.5 m spacing, arc length 40 m
  constexpr double radius = 15.0;
  Trajectory trajectory;
  for (int i = 0; i <= 80; ++i) {
    const double arc = 0.5 * i;
    const double theta = arc / radius;
    TrajectoryPoint pt;
    pt.pose.position.x = radius * std::sin(theta);
    pt.pose.position.y = radius * (1.0 - std::cos(theta));
    pt.pose.orientation.z = std::sin(theta / 2.0);
    pt.pose.orientation.w = std::cos(theta / 2.0);
    pt.longitudinal_velocity_mps = 5.0f;
    trajectory.points.push_back(pt);
  }

  // goal 3 m to the right (outside of the curve) of the point at arc = 30 m, heading tangent
  const double theta_goal = 30.0 / radius;
  geometry_msgs::msg::Pose goal;
  goal.position.x = radius * std::sin(theta_goal) + 3.0 * std::sin(theta_goal);
  goal.position.y = radius * (1.0 - std::cos(theta_goal)) - 3.0 * std::cos(theta_goal);
  goal.orientation.z = std::sin(theta_goal / 2.0);
  goal.orientation.w = std::cos(theta_goal / 2.0);

  const auto candidate =
    goal_planner_utils::make_pull_over_candidate(trajectory, goal, /*shift_length=*/15.0);
  ASSERT_TRUE(candidate.has_value());
  const auto & pts = candidate->points;
  ASSERT_GE(pts.size(), 3u);

  // ends exactly at the goal
  EXPECT_NEAR(pts.back().pose.position.x, goal.position.x, 1e-6);
  EXPECT_NEAR(pts.back().pose.position.y, goal.position.y, 1e-6);

  // the final segment direction matches the goal heading (no snap kink)
  const auto & last = pts.back().pose.position;
  const auto & prev = pts.at(pts.size() - 2).pose.position;
  const double final_dir = std::atan2(last.y - prev.y, last.x - prev.x);
  EXPECT_LT(std::abs(final_dir - theta_goal), 0.05);

  // no kink anywhere in the tail: adjacent segment direction changes stay small
  double prev_dir = final_dir;
  for (size_t i = pts.size() - 2; i > pts.size() - 10; --i) {
    const auto & a = pts.at(i - 1).pose.position;
    const auto & b = pts.at(i).pose.position;
    const double dir = std::atan2(b.y - a.y, b.x - a.x);
    EXPECT_LT(std::abs(dir - prev_dir), 0.1) << "kink between indices " << i - 1 << "-" << i;
    prev_dir = dir;
  }
}

TEST(GoalPlannerUtilsTest, CropTrajectoryEnd)
{
  const auto trajectory = make_reference_trajectory();  // length 59 m
  const auto cropped = goal_planner_utils::crop_trajectory_end(trajectory, 10.0);

  ASSERT_GE(cropped.points.size(), 2u);
  EXPECT_NEAR(cropped.points.back().pose.position.x, 49.0, 1e-6);

  // zero crop returns the trajectory unchanged
  const auto uncropped = goal_planner_utils::crop_trajectory_end(trajectory, 0.0);
  EXPECT_EQ(uncropped.points.size(), trajectory.points.size());
}

}  // namespace autoware::minimum_rule_based_planner
