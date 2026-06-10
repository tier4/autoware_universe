// Copyright 2026 TIER IV, inc.
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

// Unit tests for the conservative (stop) on-lane path generator.

#include "map_based_prediction/data_structure.hpp"
#include "map_based_prediction/path_generator.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace autoware::map_based_prediction
{
namespace
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjectKinematics;

// A CAR moving along +x at v0 [m/s], positioned at the origin facing +x.
TrackedObject makeMovingCar(const double v0)
{
  ObjectClassification classification;
  classification.probability = 1.0;
  classification.label = ObjectClassification::CAR;

  TrackedObjectKinematics kinematics;
  kinematics.pose_with_covariance.pose.position.x = 0.0;
  kinematics.pose_with_covariance.pose.position.y = 0.0;
  kinematics.pose_with_covariance.pose.orientation =
    autoware_utils::create_quaternion_from_yaw(0.0);
  kinematics.twist_with_covariance.twist.linear.x = v0;
  kinematics.orientation_availability = TrackedObjectKinematics::AVAILABLE;

  TrackedObject object;
  object.existence_probability = 1.0;
  object.classification.push_back(classification);
  object.kinematics = kinematics;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  object.shape.dimensions.z = 1.5;
  return object;
}

// A straight reference path along +x, [0, length] m at 1 m spacing.
PosePath makeStraightRefPath(const double length)
{
  PosePath ref_path;
  for (double x = 0.0; x <= length + 1e-6; x += 1.0) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = 0.0;
    pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
    ref_path.push_back(pose);
  }
  return ref_path;
}

double finalAdvanceX(const PredictedPath & path)
{
  return path.path.back().position.x - path.path.front().position.x;
}

constexpr double kSamplingInterval = 0.5;
constexpr double kHorizon = 10.0;

TEST(StoppingPath, EmptyForShortRefPath)
{
  const PathGenerator gen(kSamplingInterval);
  const auto object = makeMovingCar(5.0);
  PosePath ref_path;  // empty
  const auto path = gen.generateStoppingPathForOnLaneVehicle(object, ref_path, kHorizon, 2.0, 10.0);
  EXPECT_TRUE(path.path.empty());
}

TEST(StoppingPath, MonotonicNonDecreasingLongitudinal)
{
  const PathGenerator gen(kSamplingInterval);
  const auto object = makeMovingCar(5.0);
  const auto ref_path = makeStraightRefPath(60.0);
  const auto path = gen.generateStoppingPathForOnLaneVehicle(object, ref_path, kHorizon, 2.0, 10.0);

  ASSERT_FALSE(path.path.empty());
  // The object never moves backward along the lane.
  for (size_t i = 1; i < path.path.size(); ++i) {
    EXPECT_GE(path.path.at(i).position.x + 1e-6, path.path.at(i - 1).position.x);
  }
}

TEST(StoppingPath, NonExtendRunsToHorizonReach)
{
  // Without extend, the generator does NOT clamp the hard-stop path at the stop line
  // (the caller clips it geometrically). It runs at constant speed up to the horizon
  // reach (v0 * horizon), so it advances well past the un-applied stop line.
  const PathGenerator gen(kSamplingInterval);
  const auto object = makeMovingCar(5.0);
  const auto ref_path = makeStraightRefPath(60.0);
  const double stop_distance = 10.0;
  const auto path =
    gen.generateStoppingPathForOnLaneVehicle(object, ref_path, kHorizon, 2.0, stop_distance);

  ASSERT_FALSE(path.path.empty());
  EXPECT_GT(finalAdvanceX(path), stop_distance + 5.0);
  EXPECT_NEAR(finalAdvanceX(path), 5.0 * kHorizon, 1.0);
}

TEST(StoppingPath, ExtendForcesReachToStopLine)
{
  // With extend_to_stop_line, the path is force-swept to the stop line (plus a small
  // overshoot margin) instead of the full constant-speed reach, so the caller can
  // clip exactly at the line even for an object that would otherwise overshoot it.
  const PathGenerator gen(kSamplingInterval);
  const auto object = makeMovingCar(5.0);
  const auto ref_path = makeStraightRefPath(60.0);
  const double stop_distance = 10.0;
  const auto path = gen.generateStoppingPathForOnLaneVehicle(
    object, ref_path, kHorizon, 2.0, stop_distance, 0.0, /*extend_to_stop_line=*/true);

  ASSERT_FALSE(path.path.empty());
  // Reaches the stop line plus the small overshoot margin (~5 m), not the 50 m reach.
  EXPECT_GE(finalAdvanceX(path), stop_distance);
  EXPECT_LT(finalAdvanceX(path), 20.0);
}

TEST(StoppingPath, FirstPointIsObjectPose)
{
  const PathGenerator gen(kSamplingInterval);
  const auto object = makeMovingCar(5.0);
  const auto ref_path = makeStraightRefPath(50.0);
  const auto path = gen.generateStoppingPathForOnLaneVehicle(object, ref_path, kHorizon, 2.0, 10.0);

  ASSERT_FALSE(path.path.empty());
  EXPECT_NEAR(path.path.front().position.x, 0.0, 1e-6);
  EXPECT_NEAR(path.path.front().position.y, 0.0, 1e-6);
}

}  // namespace
}  // namespace autoware::map_based_prediction
