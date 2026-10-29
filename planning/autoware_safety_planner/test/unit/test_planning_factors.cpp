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

#include "trajectory_planner/planning_factors.hpp"

#include <autoware_utils_uuid/uuid_helper.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

constexpr double FRONT_M = 4.0;
constexpr double HORIZON_S = 8.0;
constexpr double EGO_S = 10.0;
constexpr double PATH_LENGTH_M = 100.0;

using PredictedObject = autoware_perception_msgs::msg::PredictedObject;

//! Straight path along +x, so arc length s is the x coordinate
PathPointTrajectory make_straight_path()
{
  std::vector<PathPointWithLaneId> points;
  for (int i = 0; i <= 10; ++i) {
    auto & point = points.emplace_back();
    point.point.pose.position.x = PATH_LENGTH_M * i / 10.0;
    point.point.pose.orientation.w = 1.0;
  }
  return *PathPointTrajectory::Builder{}.build(points);
}

//! Owns the input PlannerContext references. context() stays valid while this object does.
struct Scene
{
  SafetyPlannerInput input;
  PathPointTrajectory path;

  Scene(const double ego_s, PredictedObjects::ConstSharedPtr objects)
  {
    input.vehicle_info.max_longitudinal_offset_m = FRONT_M;
    input.odometry.pose.pose.position.x = ego_s;
    input.odometry.pose.pose.orientation.w = 1.0;
    input.predicted_objects = std::move(objects);
    path = make_straight_path();
  }

  PlannerContext context() const { return PlannerContext(input, path); }
};

StopBarEntry make_bar(
  const double s_stop, const std::size_t raw_index, const double t0 = 0.0, const double t1 = INF)
{
  StopBarEntry bar;
  bar.s_stop = s_stop;
  bar.raw_index = raw_index;
  bar.time.t0 = t0;
  bar.time.t1 = t1;
  return bar;
}

Constraint make_constraint(std::string plugin_name, std::string detail, std::string target_id = {})
{
  Constraint constraint;
  constraint.source.plugin_name = std::move(plugin_name);
  constraint.source.detail = std::move(detail);
  constraint.source.target_id = std::move(target_id);
  return constraint;
}

void expect_control_point_at(const Pose & pose, const double s)
{
  EXPECT_NEAR(pose.position.x, s, 1e-6);
  EXPECT_NEAR(pose.position.y, 0.0, 1e-6);
  EXPECT_NEAR(pose.position.z, 0.0, 1e-6);
  EXPECT_NEAR(pose.orientation.w, 1.0, 1e-6);
  EXPECT_NEAR(pose.orientation.z, 0.0, 1e-6);
}

//! No closed bar: safe, and the stop sits at the end of the reference path
void expect_safe_at_path_end(const PlanningFactorData & data)
{
  EXPECT_TRUE(data.safety_factors.is_safe);
  EXPECT_TRUE(data.safety_factors.factors.empty());
  EXPECT_TRUE(data.safety_factors.detail.empty());
  EXPECT_TRUE(data.detail.empty());
  EXPECT_NEAR(data.distance, PATH_LENGTH_M - EGO_S, 1e-6);
  expect_control_point_at(data.control_point_pose, PATH_LENGTH_M);
}

UUID make_uuid(const uint8_t value)
{
  UUID id;
  id.uuid.fill(value);
  return id;
}

PredictedObjects::ConstSharedPtr make_objects(const UUID & named_id, const UUID & other_id)
{
  auto objects = std::make_shared<PredictedObjects>();

  PredictedObject other;
  other.object_id = other_id;
  other.kinematics.initial_pose_with_covariance.pose.position.x = 1.0;
  objects->objects.push_back(other);

  PredictedObject named;
  named.object_id = named_id;
  named.kinematics.initial_pose_with_covariance.pose.position.x = 40.0;
  named.kinematics.initial_pose_with_covariance.pose.position.y = 1.5;
  named.kinematics.initial_pose_with_covariance.pose.position.z = 0.25;
  objects->objects.push_back(named);
  return objects;
}

}  // namespace

// Nothing to stop for: the factor is the end of the path, and it is safe
TEST(CreatePlanningFactorData, NoStopBarLeavesTheFactorAtThePathEnd)
{
  const Scene scene(EGO_S, nullptr);
  const CompiledConstraints compiled;

  const auto data = create_planning_factor_data(scene.context(), compiled, HORIZON_S);
  expect_safe_at_path_end(data);
}

// A bar already open, one that closes only after the horizon, one whose raw_index misses the
// constraint list, and one whose rear-axle stop is not short of the path end
TEST(CreatePlanningFactorData, BarsThatDoNotStopInsideTheHorizonAreIgnored)
{
  const Scene scene(EGO_S, nullptr);
  CompiledConstraints compiled;
  compiled.raw_constraints.push_back(make_constraint("obstacle_stop", "stop_line"));
  compiled.stop_bars.push_back(make_bar(20.0, /*raw_index=*/1));
  compiled.stop_bars.push_back(make_bar(30.0, 0, /*t0=*/0.0, /*t1=*/-0.1));
  compiled.stop_bars.push_back(make_bar(40.0, 0, /*t0=*/HORIZON_S + 0.1, INF));
  compiled.stop_bars.push_back(make_bar(PATH_LENGTH_M + FRONT_M, 0));

  const auto data = create_planning_factor_data(scene.context(), compiled, HORIZON_S);
  expect_safe_at_path_end(data);
}

// The gate is where the footprint front must stop, so the rear axle stops one front overhang
// earlier
TEST(CreatePlanningFactorData, DistanceIsFromTheRearAxleToTheGate)
{
  const Scene scene(EGO_S, nullptr);
  CompiledConstraints compiled;
  compiled.raw_constraints.push_back(make_constraint("obstacle_stop", "dynamic_obstacle"));
  compiled.stop_bars.push_back(make_bar(30.0, 0));

  const auto data = create_planning_factor_data(scene.context(), compiled, HORIZON_S);

  EXPECT_NEAR(data.distance, 30.0 - FRONT_M - EGO_S, 1e-6);
  expect_control_point_at(data.control_point_pose, 30.0 - FRONT_M);
  EXPECT_EQ(data.detail, "obstacle_stop: dynamic_obstacle");
  EXPECT_TRUE(data.safety_factors.is_safe);
  EXPECT_TRUE(data.safety_factors.factors.empty());
}

// The nearest rear-axle stop wins. A bar that closes at t = horizon, and one that opens at t = 0,
// are both still inside the window; a closer bar outside it is not.
TEST(CreatePlanningFactorData, NearestClosedBarInsideTheHorizonWins)
{
  const Scene scene(EGO_S, nullptr);
  CompiledConstraints compiled;
  compiled.raw_constraints.push_back(make_constraint("ignored", "closer_but_open"));
  compiled.raw_constraints.push_back(make_constraint("obstacle_stop", "stop_line"));
  compiled.raw_constraints.push_back(make_constraint("ignored", "closes_at_horizon"));
  compiled.raw_constraints.push_back(make_constraint("ignored", "clearly_inside"));

  compiled.stop_bars.push_back(make_bar(20.0, 0, /*t0=*/0.0, /*t1=*/-1.0));
  compiled.stop_bars.push_back(make_bar(50.0, 1, /*t0=*/0.0, /*t1=*/0.0));
  compiled.stop_bars.push_back(make_bar(70.0, 2, /*t0=*/HORIZON_S, INF));
  compiled.stop_bars.push_back(make_bar(90.0, 3));

  const auto data = create_planning_factor_data(scene.context(), compiled, HORIZON_S);

  EXPECT_NEAR(data.distance, 50.0 - FRONT_M - EGO_S, 1e-6);
  expect_control_point_at(data.control_point_pose, 50.0 - FRONT_M);
  EXPECT_EQ(data.detail, "obstacle_stop: stop_line");
}

// A bar already behind the rear axle is still the nearest, so it explains the stop, but the
// reported distance cannot be negative and the control point stays on the ego
TEST(CreatePlanningFactorData, BarBehindTheEgoClampsTheDistanceAtZero)
{
  constexpr double ego_s = 20.0;
  const Scene scene(ego_s, nullptr);
  CompiledConstraints compiled;
  compiled.raw_constraints.push_back(make_constraint("gate", "passed_stop_line"));
  compiled.raw_constraints.push_back(make_constraint("gate", "ahead_stop_line"));
  compiled.stop_bars.push_back(make_bar(22.0, 0));
  compiled.stop_bars.push_back(make_bar(60.0, 1));

  const auto data = create_planning_factor_data(scene.context(), compiled, HORIZON_S);

  EXPECT_NEAR(data.distance, 0.0, 1e-6);
  expect_control_point_at(data.control_point_pose, ego_s);
  EXPECT_EQ(data.detail, "gate: passed_stop_line");
}

// The object named by target_id becomes one unsafe OBJECT factor at its current position
TEST(CreatePlanningFactorData, TargetObjectIsReportedAsAnUnsafeFactor)
{
  const auto named_id = make_uuid(0xab);
  const auto objects = make_objects(named_id, make_uuid(0x11));
  const Scene scene(EGO_S, objects);
  CompiledConstraints compiled;
  compiled.raw_constraints.push_back(make_constraint(
    "obstacle_stop", "dynamic_obstacle", autoware_utils_uuid::to_hex_string(named_id)));
  compiled.stop_bars.push_back(make_bar(40.0, 0));

  const auto data = create_planning_factor_data(scene.context(), compiled, HORIZON_S);

  EXPECT_EQ(data.detail, "obstacle_stop: dynamic_obstacle");
  EXPECT_FALSE(data.safety_factors.is_safe);
  EXPECT_EQ(data.safety_factors.detail, "dynamic_obstacle");
  ASSERT_EQ(data.safety_factors.factors.size(), 1u);
  const auto & factor = data.safety_factors.factors.front();
  EXPECT_EQ(factor.type, SafetyFactor::OBJECT);
  EXPECT_FALSE(factor.is_safe);
  EXPECT_EQ(factor.object_id.uuid, named_id.uuid);
  ASSERT_EQ(factor.points.size(), 1u);
  EXPECT_NEAR(factor.points.front().x, 40.0, 1e-6);
  EXPECT_NEAR(factor.points.front().y, 1.5, 1e-6);
  EXPECT_NEAR(factor.points.front().z, 0.25, 1e-6);
}

// The stop is still explained, but without the named object the factor stays safe
TEST(CreatePlanningFactorData, MissingObjectLeavesTheFactorSafe)
{
  const auto objects = make_objects(make_uuid(0xab), make_uuid(0x11));
  const Scene scene(EGO_S, objects);

  const auto with_unknown_target = [&scene]() {
    CompiledConstraints compiled;
    compiled.raw_constraints.push_back(
      make_constraint("obstacle_stop", "dynamic_obstacle", "not-in-the-predictions"));
    compiled.stop_bars.push_back(make_bar(40.0, 0));
    return create_planning_factor_data(scene.context(), compiled, HORIZON_S);
  }();
  EXPECT_EQ(with_unknown_target.detail, "obstacle_stop: dynamic_obstacle");
  EXPECT_TRUE(with_unknown_target.safety_factors.is_safe);
  EXPECT_TRUE(with_unknown_target.safety_factors.detail.empty());
  EXPECT_TRUE(with_unknown_target.safety_factors.factors.empty());

  const Scene without_predictions(EGO_S, nullptr);
  CompiledConstraints compiled;
  compiled.raw_constraints.push_back(
    make_constraint("obstacle_stop", "dynamic_obstacle", "0123456789abcdef0123456789abcdef"));
  compiled.stop_bars.push_back(make_bar(40.0, 0));
  const auto data = create_planning_factor_data(without_predictions.context(), compiled, HORIZON_S);
  EXPECT_EQ(data.detail, "obstacle_stop: dynamic_obstacle");
  EXPECT_TRUE(data.safety_factors.is_safe);
  EXPECT_TRUE(data.safety_factors.factors.empty());
}

// plugin_name and detail are joined when both are set, and either one is used on its own
TEST(CreatePlanningFactorData, DetailUsesWhicheverSourceFieldIsSet)
{
  const auto factor_of = [](const std::string & plugin_name, const std::string & detail) {
    const Scene scene(EGO_S, nullptr);
    CompiledConstraints compiled;
    compiled.raw_constraints.push_back(make_constraint(plugin_name, detail));
    compiled.stop_bars.push_back(make_bar(40.0, 0));
    return create_planning_factor_data(scene.context(), compiled, HORIZON_S);
  };

  EXPECT_EQ(factor_of("", "stop_line").detail, "stop_line");
  EXPECT_EQ(factor_of("lanelet_speed_limit", "").detail, "lanelet_speed_limit");
  EXPECT_EQ(
    factor_of("obstacle_stop", "dynamic_obstacle").detail, "obstacle_stop: dynamic_obstacle");
  EXPECT_TRUE(factor_of("", "").detail.empty());
}

}  // namespace autoware::safety_planner
