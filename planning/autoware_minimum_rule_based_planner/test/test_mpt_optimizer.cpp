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

#include "mpt_optimizer.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
namespace
{
//! Straight trajectory along +x, one point per metre, all at `velocity` and at height `z`.
TrajectoryPoints make_straight_trajectory(
  const size_t size, const float velocity, const double z = 0.0)
{
  TrajectoryPoints points;
  for (size_t i = 0; i < size; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(i);
    point.pose.position.z = z;
    point.pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
    point.longitudinal_velocity_mps = velocity;
    points.push_back(point);
  }
  return points;
}

Odometry make_odometry(const double x, const double velocity)
{
  Odometry odometry;
  odometry.pose.pose.position.x = x;
  odometry.pose.pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
  odometry.twist.twist.linear.x = velocity;
  return odometry;
}

//! j6_gen2 geometry, so the derived curvature limit is the one the vehicle actually has.
VehicleInfo make_vehicle_info()
{
  return autoware::vehicle_info_utils::createVehicleInfo(
    0.383, 0.235, 4.76012, 1.63, 0.95099, 1.52579, 0.39920, 0.39920, 2.5, 0.640);
}

MptOptimizer make_optimizer(const EngageParams & engage = EngageParams{})
{
  // The generated Params struct carries a default for every field, so this is the node's
  // configuration as shipped.
  const Params params;
  return MptOptimizer{
    make_mpt_optimizer_params(params, make_vehicle_info()), engage,
    rclcpp::get_logger("test_mpt_optimizer"), std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)};
}
}  // namespace

/**
 * The velocities the pipeline hands over are caps, not a profile: the NLP must be free to choose a
 * lower speed (for the lateral acceleration limit, say) but never a higher one.
 */
TEST(MptOptimizer, VelocitiesBecomePerPointCaps)
{
  auto points = make_straight_trajectory(10, 8.0F);
  points[4].longitudinal_velocity_mps = 3.0F;

  const auto input = make_optimization_input(points, make_odometry(0.0, 8.0), 0.0, std::nullopt);

  ASSERT_EQ(input.velocity_limits.size(), points.size());
  EXPECT_DOUBLE_EQ(input.velocity_limits[3], 8.0);
  EXPECT_DOUBLE_EQ(input.velocity_limits[4], 3.0);
}

/**
 * A zero velocity written by a modifier or the map-based stop planner is a *stop point*, not a
 * cheap speed cap: the NLP has to know its station so it can plan the deceleration towards it and
 * hold zero beyond it. Losing this is the difference between stopping at a stop line and driving
 * through it, so it must not degrade to "just another cap".
 */
TEST(MptOptimizer, FirstZeroVelocityBecomesTheStopStation)
{
  auto points = make_straight_trajectory(10, 5.0F);
  for (size_t i = 6; i < points.size(); ++i) {
    points[i].longitudinal_velocity_mps = 0.0F;
  }

  const auto input = make_optimization_input(points, make_odometry(0.0, 5.0), 0.0, std::nullopt);

  ASSERT_TRUE(input.stop_arc_length.has_value());
  // The points sit one metre apart, so the sixth one is six metres along the path.
  EXPECT_DOUBLE_EQ(*input.stop_arc_length, 6.0);
}

/**
 * The path is drawn from 5 m behind the ego, and the NLP anchors its horizon at the ego pose, so
 * the part already driven can only make the fixed number of stages coarser.
 */
TEST(MptOptimizer, CropsThePathBehindTheEgo)
{
  const auto points = make_straight_trajectory(10, 5.0F);

  const auto cropped = crop_behind_ego(points, make_odometry(5.0, 5.0).pose.pose);

  ASSERT_EQ(cropped.size(), 5U);
  EXPECT_DOUBLE_EQ(cropped.front().pose.position.x, 5.0);
  EXPECT_DOUBLE_EQ(cropped.back().pose.position.x, 9.0);
}

/**
 * The stop station is an arc length along the input, and the library measures the horizon from the
 * ego: both have to count from the same point. Handing over the uncropped path leaves the two
 * origins 5 m apart and the deceleration would be planned for a stop line that is not where the
 * planner put it.
 */
TEST(MptOptimizer, StopStationIsMeasuredFromTheEgo)
{
  auto points = make_straight_trajectory(10, 5.0F);
  for (size_t i = 8; i < points.size(); ++i) {
    points[i].longitudinal_velocity_mps = 0.0F;
  }
  const auto odometry = make_odometry(5.0, 5.0);

  const auto input = make_optimization_input(
    crop_behind_ego(points, odometry.pose.pose), odometry, 0.0, std::nullopt);

  ASSERT_TRUE(input.stop_arc_length.has_value());
  // The stop sits at x = 8 m and the ego at x = 5 m, so it is 3 m ahead - not 8.
  EXPECT_DOUBLE_EQ(*input.stop_arc_length, 3.0);
}

TEST(MptOptimizer, NoStopPointWhenEveryVelocityIsPositive)
{
  const auto input = make_optimization_input(
    make_straight_trajectory(10, 5.0F), make_odometry(0.0, 5.0), 0.0, std::nullopt);

  EXPECT_FALSE(input.stop_arc_length.has_value());
  // The end of the path is still a stop, exactly as the velocity smoother forces it to be.
  EXPECT_TRUE(input.stop_at_end);
}

/**
 * The NLP pins its initial state to the ego, not to the head of the path: the path starts behind
 * the vehicle, and the library anchors the horizon at the ego pose.
 */
TEST(MptOptimizer, EgoStateIsCarriedOver)
{
  const auto input = make_optimization_input(
    make_straight_trajectory(10, 5.0F), make_odometry(2.5, 4.0), -0.7, 0.02);

  EXPECT_DOUBLE_EQ(input.ego.pose.x, 2.5);
  EXPECT_DOUBLE_EQ(input.ego.velocity, 4.0);
  EXPECT_DOUBLE_EQ(input.ego.acceleration, -0.7);
  ASSERT_TRUE(input.ego.curvature.has_value());
  EXPECT_DOUBLE_EQ(*input.ego.curvature, 0.02);
}

TEST(MptOptimizer, OutputCarriesPoseVelocityAndAcceleration)
{
  std::vector<mpt::OptimizedPoint> optimized(1);
  optimized[0].x = 1.0;
  optimized[0].y = 2.0;
  optimized[0].z = 3.0;
  optimized[0].yaw = 0.5;
  optimized[0].velocity = 4.0;
  optimized[0].acceleration = -0.5;
  optimized[0].time_from_start = 1.25;

  const auto points = to_trajectory_points(optimized);

  ASSERT_EQ(points.size(), 1U);
  EXPECT_EQ(rclcpp::Duration(points[0].time_from_start).seconds(), 1.25);
  EXPECT_DOUBLE_EQ(points[0].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(points[0].pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(points[0].pose.position.z, 3.0);
  EXPECT_NEAR(tf2::getYaw(points[0].pose.orientation), 0.5, 1.0e-9);
  EXPECT_FLOAT_EQ(points[0].longitudinal_velocity_mps, 4.0F);
  EXPECT_FLOAT_EQ(points[0].acceleration_mps2, -0.5F);
}

/**
 * End-to-end through the real solver, with the node's default parameters and the real vehicle
 * geometry: the point is that the wiring produces something drivable, not that the NLP is correct
 * (the library has its own tests for that).
 */
TEST(MptOptimizer, SolvesAStraightPathWithinTheSpeedCap)
{
  auto optimizer = make_optimizer();
  const auto points = make_straight_trajectory(100, 8.0F);

  const auto optimized = optimizer.optimize(points, make_odometry(0.0, 8.0), 0.0, 0.0);

  ASSERT_TRUE(optimized.has_value());
  ASSERT_GT(optimized->size(), 2U);
  // The horizon is anchored at the ego.
  EXPECT_NEAR(optimized->front().pose.position.x, 0.0, 0.1);
  // The terminal speed is soft by design (a path too short to decelerate must not turn into an
  // infeasible problem), so the NLP only pulls it towards zero - the exact zero at the end of the
  // published trajectory comes from the post-resample step in the node.
  EXPECT_LT(optimized->back().longitudinal_velocity_mps, 0.5F);
  for (const auto & point : *optimized) {
    EXPECT_GE(point.longitudinal_velocity_mps, 0.0F);
    EXPECT_LE(point.longitudinal_velocity_mps, 8.0F + 1.0e-3F);
  }
}

/**
 * The height has to survive the round trip. Nothing downstream checks it, but the post-resample
 * step derives the pitch of every published pose from consecutive z values, so a wrong z is not a
 * cosmetic error: it turns into a pose that points out of the road plane, and the controller then
 * follows it. This is the regression test for the trajectory that flew off in lsim.
 */
TEST(MptOptimizer, HeightIsPreserved)
{
  auto optimizer = make_optimizer();
  // A sloped road, like the one this was found on (4.99 m rising to 6.0 m over 120 m). A flat road
  // would not distinguish "carried through" from "taken from the ego".
  constexpr double ROAD_HEIGHT = 5.0;
  constexpr double SLOPE = 0.0084;
  auto points = make_straight_trajectory(100, 8.0F, ROAD_HEIGHT);
  for (size_t i = 0; i < points.size(); ++i) {
    points[i].pose.position.z = ROAD_HEIGHT + SLOPE * static_cast<double>(i);
  }
  // The path starts behind the ego, as it does in the node (path_length.backward = 5 m).
  auto odometry = make_odometry(5.0, 8.0);
  odometry.pose.pose.position.z = ROAD_HEIGHT + SLOPE * 5.0;

  const auto optimized = optimizer.optimize(points, odometry, 0.0, 0.0);

  ASSERT_TRUE(optimized.has_value());
  for (const auto & point : *optimized) {
    // The height at this station, which is what "carried through" has to mean.
    const double expected = ROAD_HEIGHT + SLOPE * (point.pose.position.x);
    EXPECT_NEAR(point.pose.position.z, expected, 0.05);
  }
}

/**
 * The initial curvature is a *hard* constraint, so a wrong one cannot be smoothed away: the
 * trajectory has to drive the arc it was given and then come back, which shows up as a bulge off
 * the path. Estimating it from the input path around the ego does not work - over the ~1.5 m the
 * estimate spans, the few centimetres of lateral offset an ego always has turn into a curvature of
 * the wrong magnitude and often the wrong sign. Measured on the road this produced a 1.5 m bulge on
 * a straight lane, which is why the caller passes tan(steer) / wheel_base instead.
 */
TEST(MptOptimizer, StaysOnAStraightPathWhenTheEgoIsOffsetFromIt)
{
  auto optimizer = make_optimizer();
  const auto points = make_straight_trajectory(100, 8.0F);
  // 5 cm off the path and 5 m along it, as measured in lsim.
  auto odometry = make_odometry(5.0, 8.0);
  odometry.pose.pose.position.y = 0.05;

  const auto optimized = optimizer.optimize(points, odometry, 0.0, /*ego_curvature=*/0.0);

  ASSERT_TRUE(optimized.has_value());
  double worst = 0.0;
  for (const auto & point : *optimized) {
    worst = std::max(worst, std::abs(point.pose.position.y));
  }
  EXPECT_LT(worst, 0.2) << "the trajectory bulged " << worst << " m off a straight lane";
}

/**
 * From a standstill the NLP's profile starts at exactly zero, because the initial speed is pinned
 * to the ego's. The longitudinal controller only leaves its stopped state once the speed at the ego
 * exceeds its engage threshold, so without a floor the vehicle plans a departure every cycle and
 * never takes it - which is exactly what it did in lsim.
 */
TEST(MptOptimizer, EngagesFromStandstill)
{
  auto optimizer = make_optimizer();
  const auto points = make_straight_trajectory(100, 8.0F);

  const auto optimized = optimizer.optimize(points, make_odometry(0.0, 0.0), 0.0, 0.0);

  ASSERT_TRUE(optimized.has_value());
  EXPECT_GE(optimized->front().longitudinal_velocity_mps, 0.25F);
}

/**
 * ...but not into a stop point that is right in front of the vehicle. Engaging there is how a
 * vehicle creeps into whatever it was told to stop for.
 */
TEST(MptOptimizer, DoesNotEngageIntoAStopPointRightAhead)
{
  EngageParams engage;
  engage.stop_dist_to_prohibit_engage = 2.0;  // the stop below sits inside this
  auto optimizer = make_optimizer(engage);
  auto points = make_straight_trajectory(100, 8.0F);
  for (size_t i = 1; i < points.size(); ++i) {
    points[i].longitudinal_velocity_mps = 0.0F;  // stop 1 m ahead
  }

  const auto optimized = optimizer.optimize(points, make_odometry(0.0, 0.0), 0.0, 0.0);

  ASSERT_TRUE(optimized.has_value());
  for (const auto & point : *optimized) {
    EXPECT_LT(point.longitudinal_velocity_mps, 0.25F);
  }
}

/**
 * And the engage speed must not be painted over the stop point itself: raising every point is what
 * erases the modifiers' stop and makes the vehicle creep through it.
 */
TEST(MptOptimizer, EngageDoesNotOverwriteAStopFurtherAhead)
{
  auto optimizer = make_optimizer();
  auto points = make_straight_trajectory(100, 8.0F);
  constexpr size_t STOP_INDEX = 20;  // 20 m ahead, well beyond stop_dist_to_prohibit_engage
  for (size_t i = STOP_INDEX; i < points.size(); ++i) {
    points[i].longitudinal_velocity_mps = 0.0F;
  }

  const auto optimized = optimizer.optimize(points, make_odometry(0.0, 0.0), 0.0, 0.0);

  ASSERT_TRUE(optimized.has_value());
  EXPECT_GE(optimized->front().longitudinal_velocity_mps, 0.25F) << "should still depart";
  // Everything from the stop point on has to stay stopped.
  for (const auto & point : *optimized) {
    if (point.pose.position.x > static_cast<double>(STOP_INDEX) + 1.0) {
      EXPECT_LT(point.longitudinal_velocity_mps, 0.25F);
    }
  }
}

/**
 * The optimiser is one stage of a backup trajectory generator, so an input it cannot handle has to
 * come back as "use the fallback", never as a throw or a half-built trajectory.
 */
TEST(MptOptimizer, RejectsInputItCannotRepresent)
{
  auto optimizer = make_optimizer();

  EXPECT_FALSE(optimizer.optimize({}, make_odometry(0.0, 0.0), 0.0, 0.0).has_value());

  auto reversing = make_straight_trajectory(10, -1.0F);
  EXPECT_FALSE(optimizer.optimize(reversing, make_odometry(0.0, -1.0), 0.0, 0.0).has_value());
}

}  // namespace autoware::minimum_rule_based_planner
