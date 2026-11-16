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

#include "test_utils.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::predicted_trajectory_validator::collision_validator
{
using namespace test_utils;  // NOLINT(build/namespaces)

TEST(CollisionEvaluatorTest, StaticPointsAheadHaveTtcOfDistanceOverVelocity)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  const auto odom = make_odometry(kEgoVelocity);

  const std::vector<PointCluster> clusters = {make_cluster(20.0, 21.0, -0.5, 0.5)};
  const auto candidates =
    evaluate(default_param(), predicted, odom, objects_at(predicted), clusters);

  ASSERT_FALSE(candidates.empty());
  EXPECT_EQ(points_of(candidates).size(), candidates.size());
  double first_collision_time = std::numeric_limits<double>::infinity();
  for (const auto & point : clusters.front().points) {
    if (const auto t = point_collision_time(predicted, point)) {
      first_collision_time = std::min(first_collision_time, *t);
    }
  }
  EXPECT_NEAR(first_collision_time, expected_static_ttc(20.0 - kFrontOffset), 1e-6);
}

TEST(CollisionEvaluatorTest, LateralPointsHaveNoCandidate)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  const std::vector<PointCluster> clusters = {make_cluster(20.0, 21.0, 3.0, 4.0)};
  EXPECT_TRUE(
    evaluate(default_param(), predicted, make_odometry(), objects_at(predicted), clusters).empty());
}

TEST(CollisionEvaluatorTest, EgoFootprintFollowsTheLeftAndRightOverhangs)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  const auto odom = make_odometry(kEgoVelocity);
  const auto vehicle_info = vehicle_info_utils::createVehicleInfo(
    0.383, 0.235, 2.79, 1.64, 1.0, 1.1, 1.0, 0.128, 2.5, 0.70);
  const auto evaluate_cluster = [&](const PointCluster & cluster) {
    const std::optional<PointCloudClusters> point_clusters =
      PointCloudClusters{{cluster}, rclcpp::Time(kCloudStamp)};
    return CollisionEvaluator(default_param())
      .evaluate(
        predicted, odom, vehicle_info,
        std::make_shared<const PredictedObjects>(objects_at(predicted)), point_clusters);
  };

  EXPECT_FALSE(evaluate_cluster(make_cluster(20.0, 21.0, 1.5, 1.75)).empty());
  EXPECT_TRUE(evaluate_cluster(make_cluster(20.0, 21.0, -1.75, -1.5)).empty());
}

TEST(CollisionEvaluatorTest, PointsBeyondHorizonHaveNoCandidate)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  // 5 s * 10 m/s + 3.79 m = 53.79 m is the farthest point swept.
  const std::vector<PointCluster> clusters = {make_cluster(60.0, 61.0, -0.5, 0.5)};
  EXPECT_TRUE(
    evaluate(default_param(), predicted, make_odometry(), objects_at(predicted), clusters).empty());
}

TEST(CollisionEvaluatorTest, FrontPointHasCosNearOne)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  const Point2d point{20.0, 0.0};

  const auto t = point_collision_time(predicted, point);
  ASSERT_TRUE(t.has_value());
  const auto k = static_cast<size_t>(std::round(*t / kDt));
  EXPECT_NEAR(compute_cos_collision(predicted.points[k].pose, true, point), 1.0, 1e-3);
}

TEST(CollisionEvaluatorTest, PointAheadOnTheCurveHasCosNearOneAtCollisionTime)
{
  // Left turn of radius 20 m: after 60 deg (t = 2.09 s) the ego is at (17.3, 10.0) heading 60
  // deg. A point slightly ahead on the arc is currently at bearing ~31 deg (cos 0.86) but is
  // straight ahead at the collision time.
  const auto predicted = make_arc_trajectory(100.0, kEgoVelocity, 20.0, 5.0);
  const double theta = 60.0 * M_PI / 180.0 + 2.0 / 20.0;  // 2 m further along the arc
  const Point2d point{20.0 * std::sin(theta), 20.0 * (1.0 - std::cos(theta))};

  const auto t = point_collision_time(predicted, point);
  ASSERT_TRUE(t.has_value());
  const auto k = static_cast<size_t>(std::round(*t / kDt));
  EXPECT_GT(compute_cos_collision(predicted.points[k].pose, true, point), 0.97);
  const double bearing_now = std::atan2(point.y(), point.x());
  EXPECT_LT(std::cos(bearing_now), 0.9);
}

TEST(CollisionEvaluatorTest, CrossingObjectTtcMatchesAnalyticCrossingTime)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  const auto odom = make_odometry(kEgoVelocity);

  // Object at (25, -10) moving +y at 5 m/s. Ego x-range covers [24.5, 25.5] for t in
  // [2.07, 2.66]; the object y-range covers [-0.948, 0.948] for t in [1.71, 2.29].
  // The first overlap is at t = 2.07 -> first window [2.0, 2.1] -> ttc = 2.1.
  const double object_vy = 5.0;
  auto object = make_object(25.0, -10.0, M_PI / 2.0, object_vy, ObjectClassification::CAR);
  add_straight_path(object, object_vy, M_PI / 2.0);
  const auto objects = objects_at(predicted, {object});

  const auto candidates = evaluate(default_param(), predicted, odom, objects, {});

  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_FALSE(is_point(candidates.front()));
  const auto t = object_collision_time(predicted, path_trajectory(object, 0.0));
  ASSERT_TRUE(t.has_value());
  EXPECT_NEAR(*t, 2.1, 1e-6);
  const auto cos_collision = collision_cos_of(default_param(), predicted, odom, objects);
  ASSERT_TRUE(cos_collision.has_value());
  EXPECT_GT(*cos_collision, 0.9);  // hits the front of the ego
}

TEST(CollisionEvaluatorTest, OppositeLaneObjectHasNoTtc)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);

  // Oncoming object in the adjacent lane (y = 3.5), approaching but never crossing.
  auto object = make_object(40.0, 3.5, M_PI, 10.0, ObjectClassification::CAR, 4.0, 1.8);
  add_straight_path(object, 10.0, M_PI);

  EXPECT_TRUE(
    evaluate(default_param(), predicted, make_odometry(), objects_at(predicted, {object}), {})
      .empty());
  EXPECT_FALSE(object_collision_time(predicted, path_trajectory(object, 0.0)).has_value());
}

TEST(CollisionEvaluatorTest, ObjectStampOffsetIsCompensated)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  const double object_vy = 5.0;

  for (const double offset : {-0.2, 0.0, 0.2}) {
    // The object observed `offset` seconds after the trajectory stamp has already moved by
    // offset * vy, so the physical crossing time stays the same.
    PredictedObjects objects;
    objects.header.stamp = to_msg_time(100.0 + offset);
    auto object = make_object(
      25.0, -10.0 + object_vy * offset, M_PI / 2.0, object_vy, ObjectClassification::CAR);
    add_straight_path(object, object_vy, M_PI / 2.0);
    objects.objects.push_back(object);

    const auto candidates = evaluate(default_param(), predicted, make_odometry(), objects, {});
    ASSERT_EQ(candidates.size(), 1U) << "offset " << offset;
    const auto t = object_collision_time(predicted, path_trajectory(object, offset));
    ASSERT_TRUE(t.has_value()) << "offset " << offset;
    EXPECT_NEAR(*t, 2.1, 1e-6) << "offset " << offset;
  }
}

TEST(CollisionEvaluatorTest, ObjectsOlderThanStampGapAreNotEvaluated)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);

  PredictedObjects objects;
  objects.header.stamp = to_msg_time(100.0 - 0.6);
  objects.objects.push_back(make_object(20.0, 0.0, 0.0, 0.0, ObjectClassification::CAR));

  EXPECT_TRUE(evaluate(default_param(), predicted, make_odometry(), objects, {}).empty());
}

TEST(CollisionEvaluatorTest, ObjectTouchingTheCurrentFootprintIsExcluded)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  // Overlaps the ego footprint (x in [-1.1, 3.79], |y| <= 0.948) at the current time.
  const auto objects = objects_at(
    predicted, {make_object(2.0, 1.2, 0.0, 0.0, ObjectClassification::CAR),
                make_object(20.5, 0.0, 0.0, 0.0, ObjectClassification::CAR)});

  const auto candidates = evaluate(default_param(), predicted, make_odometry(), objects, {});

  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_NEAR(candidates.front().distance_now, 20.0 - kFrontOffset, 1e-6);
}

TEST(CollisionEvaluatorTest, DisabledLabelsAndEvaluationLimitAreApplied)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  const auto objects = objects_at(
    predicted, {make_object(30.5, 0.0, 0.0, 0.0, ObjectClassification::CAR),
                make_object(20.5, 0.0, 0.0, 0.0, ObjectClassification::CAR),
                make_object(10.5, 0.0, 0.0, 0.0, ObjectClassification::BUS),
                make_object(40.5, 0.0, 0.0, 0.0, ObjectClassification::CAR)});

  auto param = default_param();
  param.enabled_labels = {ObjectClassification::CAR};
  param.n_eval_object = 2;

  const auto candidates = evaluate(param, predicted, make_odometry(), objects, {});

  // BUS is disabled, and only the two closest CARs remain.
  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_NEAR(candidates[0].distance_now, 20.0 - kFrontOffset, 1e-6);
  EXPECT_NEAR(candidates[1].distance_now, 30.0 - kFrontOffset, 1e-6);
}

TEST(CollisionEvaluatorTest, HorizonIsClippedToTrajectoryEnd)
{
  // 2 s trajectory: the sweep ends at 23.79 m even though max_time is 5 s.
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 2.0);
  const std::vector<PointCluster> clusters = {
    make_single_point_cluster(20.0, 0.0), make_single_point_cluster(30.0, 0.0)};

  const auto candidates =
    evaluate(default_param(), predicted, make_odometry(), objects_at(predicted), clusters);

  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_NEAR(candidates.front().polygon_now.outer().front().x(), 20.0, 1e-9);
}

TEST(CollisionEvaluatorTest, StoppedEgoTrajectoryGivesNoTtc)
{
  const auto predicted = make_straight_trajectory(100.0, 0.0, 5.0);
  const auto odom = make_odometry();

  auto follower = make_object(-15.0, 0.0, 0.0, 5.0, ObjectClassification::CAR, 4.0, 1.8);
  add_straight_path(follower, 5.0, 0.0);
  const auto objects = objects_at(predicted, {follower});
  const std::vector<PointCluster> clusters = {make_single_point_cluster(3.5, 0.0)};

  EXPECT_TRUE(evaluate(default_param(), predicted, odom, objects, clusters).empty());

  auto no_stop = default_param();
  no_stop.stop_velocity = 0.0;
  const auto unmasked = evaluate(no_stop, predicted, odom, objects, clusters);
  ASSERT_EQ(unmasked.size(), 1U);
  EXPECT_TRUE(is_point(unmasked.front()));
  EXPECT_GE(unmasked.front().brake_level, 1);
}

TEST(CollisionEvaluatorTest, CollisionStartsWhenTheEgoTrajectoryStartsMoving)
{
  const auto predicted = make_starting_trajectory();
  const auto candidates = evaluate(
    default_param(), predicted, make_odometry(), objects_at(predicted),
    {make_single_point_cluster(kFrontOffset + 1.2, 0.0)});
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_FALSE(candidates.front().near);
  EXPECT_GE(candidates.front().brake_level, 1);
}

TEST(CollisionEvaluatorTest, BrakeLevelOfAStaticPointFollowsTheStoppingDistance)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  const auto odom = make_odometry(kEgoVelocity);
  const auto evaluate_point = [&](const Point2d & point) {
    return evaluate(
      default_param(), predicted, odom, objects_at(predicted),
      {make_single_point_cluster(point.x(), point.y())});
  };

  for (const auto & [gap, expected_level] :
       {std::pair{12.0, 3}, std::pair{16.0, 2}, std::pair{25.0, 1}}) {
    const auto candidates = evaluate_point(Point2d{kFrontOffset + gap, 0.0});
    ASSERT_EQ(candidates.size(), 1U) << "gap " << gap;
    EXPECT_EQ(candidates.front().brake_level, expected_level) << "gap " << gap;
  }

  const Point2d avoidable{kFrontOffset + 35.0, 0.0};
  EXPECT_TRUE(point_collision_time(predicted, avoidable).has_value());
  EXPECT_TRUE(evaluate_point(avoidable).empty());
}

TEST(CollisionEvaluatorTest, CrossingObjectAvoidedByEarlierBrakingHasLevelOne)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  auto object = make_object(25.0, -10.0, M_PI / 2.0, 5.0, ObjectClassification::CAR);
  add_straight_path(object, 5.0, M_PI / 2.0);

  const auto candidates = evaluate(
    default_param(), predicted, make_odometry(kEgoVelocity), objects_at(predicted, {object}), {});
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().brake_level, 1);
}

TEST(CollisionEvaluatorTest, FastOncomingObjectInTheEgoLaneHitsBeforeTheEgoStops)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  auto object = make_object(40.0, 0.0, M_PI, 10.0, ObjectClassification::CAR, 4.0, 1.8);
  add_straight_path(object, 10.0, M_PI);

  const auto candidates = evaluate(
    default_param(), predicted, make_odometry(kEgoVelocity), objects_at(predicted, {object}), {});
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().brake_level, 3);
}

TEST(CollisionEvaluatorTest, SlowOncomingObjectHittingTheStoppedEgoIsNotCounted)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  auto param = default_param();
  param.reaction_times = {0.5, 0.3, 0.1};
  auto object = make_object(34.0, 0.0, M_PI, 6.0, ObjectClassification::CAR, 4.0, 1.8);
  add_straight_path(object, 6.0, M_PI);

  EXPECT_TRUE(object_collision_time(predicted, path_trajectory(object, 0.0)).has_value());
  EXPECT_TRUE(
    evaluate(param, predicted, make_odometry(kEgoVelocity), objects_at(predicted, {object}), {})
      .empty());
}

TEST(CollisionEvaluatorTest, ObjectsBelowTheCosThresholdHaveNoBrakeLevel)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  const auto odom = make_odometry(kEgoVelocity);
  auto object = make_object(25.0, -10.0, M_PI / 2.0, 5.0, ObjectClassification::CAR);
  add_straight_path(object, 5.0, M_PI / 2.0);
  const auto objects = objects_at(predicted, {object});

  const auto run = [&](const double cos_threshold) {
    auto param = default_param();
    param.cos_threshold = cos_threshold;
    return evaluate(param, predicted, odom, objects, {});
  };
  const auto passing = run(-1.0);
  ASSERT_EQ(passing.size(), 1U);
  EXPECT_GE(passing.front().brake_level, 1);
  const auto cos_collision = collision_cos_of(default_param(), predicted, odom, objects);
  ASSERT_TRUE(cos_collision.has_value());
  EXPECT_LT(*cos_collision, 1.0);
  EXPECT_TRUE(run(1.0).empty());
}

TEST(CollisionEvaluatorTest, StaticPointsGetABrakeLevelBelowTheCosThreshold)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  const std::vector<PointCluster> clusters = {make_single_point_cluster(20.0, 0.5)};

  auto param = default_param();
  param.cos_threshold = 1.0;
  const auto candidates =
    evaluate(param, predicted, make_odometry(kEgoVelocity), objects_at(predicted), clusters);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().brake_level, 2);
  const auto & point = clusters.front().points.front();
  const auto t = point_collision_time(predicted, point);
  ASSERT_TRUE(t.has_value());
  const auto k = static_cast<size_t>(std::round(*t / kDt));
  EXPECT_LT(compute_cos_collision(predicted.points[k].pose, true, point), 1.0);
}

TEST(CollisionEvaluatorTest, ProximityIsRaisedOnlyWithinTheDistance)
{
  const auto predicted = make_straight_trajectory(100.0, kEgoVelocity, 5.0);
  const auto objects = objects_at(
    predicted, {make_object(1.0, 1.648, 0.0, 0.0, ObjectClassification::CAR),
                make_object(1.0, -1.948, 0.0, 0.0, ObjectClassification::CAR)});
  PointCluster cluster;
  cluster.points = {{1.0, 1.148}, {1.0, -1.448}};
  cluster.centroid = Point2d{1.0, -0.15};

  const auto candidates =
    evaluate(default_param(), predicted, make_odometry(kEgoVelocity), objects, {cluster});

  const auto object_candidates = objects_of(candidates);
  ASSERT_EQ(object_candidates.size(), 1U);
  EXPECT_NEAR(object_candidates.front().distance_now, 0.2, 1e-6);
  EXPECT_TRUE(object_candidates.front().near);
  const auto point_candidates = points_of(candidates);
  ASSERT_EQ(point_candidates.size(), 1U);
  EXPECT_NEAR(point_candidates.front().polygon_now.outer().front().y(), 1.148, 1e-9);
  EXPECT_TRUE(point_candidates.front().near);
  for (const auto & candidate : candidates) {
    EXPECT_EQ(candidate.brake_level, 0);
  }
}

struct ProximityCase
{
  std::string name;
  Trajectory predicted;
  size_t expected_candidates;
};

class ProximityByEgoMotionTest : public ::testing::TestWithParam<ProximityCase>
{
};

TEST_P(ProximityByEgoMotionTest, ProximityIsRaisedOnlyWhenTheEgoMoves)
{
  const auto & predicted = GetParam().predicted;
  const auto objects =
    objects_at(predicted, {make_object(1.0, 1.648, 0.0, 0.0, ObjectClassification::CAR)});

  const auto candidates = evaluate(
    default_param(), predicted, make_odometry(), objects, {make_single_point_cluster(1.0, 1.148)});

  ASSERT_EQ(candidates.size(), GetParam().expected_candidates);
  for (const auto & candidate : candidates) {
    EXPECT_TRUE(candidate.near);
  }
}

INSTANTIATE_TEST_SUITE_P(
  CollisionEvaluatorTest, ProximityByEgoMotionTest,
  ::testing::Values(
    ProximityCase{"WholeTrajectoryStopped", make_straight_trajectory(100.0, 0.0, 5.0), 0U},
    ProximityCase{"StoppedEgoStartsMoving", make_starting_trajectory(), 2U}),
  [](const ::testing::TestParamInfo<ProximityCase> & info) { return info.param.name; });
}  // namespace autoware::predicted_trajectory_validator::collision_validator
