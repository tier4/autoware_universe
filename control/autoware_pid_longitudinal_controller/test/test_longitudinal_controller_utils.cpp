// Copyright 2021 Tier IV, Inc.
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

#include "autoware/interpolation/spherical_linear_interpolation.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/pid_longitudinal_controller/longitudinal_controller_utils.hpp"
#include "gtest/gtest.h"

#include <tf2/LinearMath/Quaternion.hpp>

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>
#include <initializer_list>
#include <limits>
#include <vector>

namespace longitudinal_utils =
  ::autoware::motion::control::pid_longitudinal_controller::longitudinal_utils;

namespace
{
struct TemporalSample
{
  double time;
  double x;
  float velocity;
  float acceleration{0.0F};
};

autoware_planning_msgs::msg::Trajectory makeTemporalTrajectory(
  const std::initializer_list<TemporalSample> samples)
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  for (const auto & sample : samples) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(sample.time);
    point.pose.position.x = sample.x;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = sample.velocity;
    point.acceleration_mps2 = sample.acceleration;
    trajectory.points.push_back(point);
  }
  return trajectory;
}
}  // namespace

TEST(TestLongitudinalControllerUtils, temporalLaunchPreservesZeroPrefix)
{
  const auto trajectory = makeTemporalTrajectory(
    {{0.1, 0.0, 0.0F}, {0.2, 0.0, 0.0F}, {0.3, 0.005, 0.1F, 0.3F}, {0.8, 0.2, 0.3F}});
  const auto stop = longitudinal_utils::findTemporalStop(trajectory, 0.1, 0.25);
  EXPECT_TRUE(stop.departure_requested);
  EXPECT_FALSE(stop.stop_idx.has_value());
  // A moving horizon only 20 cm long must leave both STOPPED and STOPPING and must not
  // immediately enter STOPPING again under the current 1.0 / 1.5 / 0.49 m thresholds.
  EXPECT_TRUE(stop.allowsDeparture(0.2, 1.0));
  EXPECT_TRUE(stop.allowsDeparture(0.2, 1.5));
  EXPECT_FALSE(stop.requiresStopping(0.2, 0.49));

  const auto target = longitudinal_utils::calcTemporalLookaheadPoint(trajectory, 0.25, 0.0, stop);
  EXPECT_NEAR(target.longitudinal_velocity_mps, 0.05, 1.0e-6);
  EXPECT_GT(target.acceleration_mps2, 0.0F);
  EXPECT_FLOAT_EQ(trajectory.points.front().longitudinal_velocity_mps, 0.0F);
  EXPECT_FLOAT_EQ(trajectory.points.front().acceleration_mps2, 0.0F);
}

TEST(TestLongitudinalControllerUtils, temporalScheduledHoldCannotBeReleasedByLookahead)
{
  const auto trajectory = makeTemporalTrajectory(
    {{0.1, 0.0, 0.0F}, {2.0, 0.0, 0.0F}, {2.1, 0.005, 0.1F, 0.3F}, {3.0, 0.2, 0.3F}});
  const auto stop = longitudinal_utils::findTemporalStop(trajectory, 0.1, 0.25);
  ASSERT_TRUE(stop.stop_idx.has_value());
  EXPECT_EQ(*stop.stop_idx, 0U);
  EXPECT_FALSE(stop.departure_requested);
  EXPECT_FALSE(stop.allowsDeparture(10.0, 1.0));
  EXPECT_TRUE(stop.requiresStopping(10.0, 0.49));
  const auto future = longitudinal_utils::calcTemporalLookaheadPoint(trajectory, 0.25, 5.0, stop);
  EXPECT_FLOAT_EQ(future.longitudinal_velocity_mps, 0.0F);
  EXPECT_FLOAT_EQ(future.acceleration_mps2, 0.0F);

  // Release when the scheduled movement enters the actuator-delay target, not the longer
  // velocity-feedback lookahead window.
  const auto release = longitudinal_utils::findTemporalStop(trajectory, 1.95, 2.1);
  EXPECT_TRUE(release.departure_requested);
  EXPECT_TRUE(release.allowsDeparture(0.2, 1.0));
}

TEST(TestLongitudinalControllerUtils, temporalLaunchAccelerationPrecedesPhysicalVelocity)
{
  const auto trajectory = makeTemporalTrajectory(
    {{0.1, 0.0, 0.0F}, {0.2, 0.0, 0.0F, 0.4F}, {0.3, 0.005, 0.1F, 0.3F}, {0.8, 0.2, 0.3F}});
  const auto stop = longitudinal_utils::findTemporalStop(trajectory, 0.0, 0.15);
  EXPECT_TRUE(stop.allowsDeparture(0.2, 1.0));
  EXPECT_FALSE(stop.requiresStopping(0.2, 0.49));
  EXPECT_TRUE(stop.forward);
  const auto target = longitudinal_utils::calcTemporalLookaheadPoint(trajectory, 0.15, 0.0, stop);
  EXPECT_FLOAT_EQ(target.longitudinal_velocity_mps, 0.0F);
  EXPECT_NEAR(target.acceleration_mps2, 0.2, 1.0e-6);

  // A spurious positive acceleration without any planned movement must not release the hold.
  auto stationary = trajectory;
  for (auto & point : stationary.points) point.longitudinal_velocity_mps = 0.0F;
  const auto hold = longitudinal_utils::findTemporalStop(stationary, 0.0, 0.15);
  EXPECT_FALSE(hold.departure_requested);
}

TEST(TestLongitudinalControllerUtils, temporalLaunchRetainsLaterStopAndDepartureHysteresis)
{
  const auto trajectory = makeTemporalTrajectory(
    {{0.1, 0.0, 0.0F}, {0.2, 0.0, 0.0F}, {0.3, 0.1, 0.2F}, {1.0, 0.5, 0.0F}, {2.0, 1.0, 0.5F}});
  const auto stop = longitudinal_utils::findTemporalStop(trajectory, 0.1, 0.25);
  ASSERT_TRUE(stop.stop_idx.has_value());
  EXPECT_EQ(*stop.stop_idx, 3U);
  EXPECT_TRUE(stop.departure_requested);
  // Unlike a moving endpoint, an explicit stop continues to enforce the configured clearance.
  EXPECT_FALSE(stop.allowsDeparture(0.5, 1.0));
  EXPECT_FALSE(stop.allowsDeparture(1.0, 1.0));
  EXPECT_TRUE(stop.allowsDeparture(1.2, 1.0));
  EXPECT_FALSE(stop.allowsDeparture(1.2, 1.5));
  EXPECT_TRUE(stop.requiresStopping(0.4, 0.49));

  const auto future = longitudinal_utils::calcTemporalLookaheadPoint(trajectory, 0.25, 3.0, stop);
  EXPECT_DOUBLE_EQ(rclcpp::Duration(future.time_from_start).seconds(), 1.0);
  EXPECT_FLOAT_EQ(future.longitudinal_velocity_mps, 0.0F);
  EXPECT_DOUBLE_EQ(future.pose.position.x, 0.5);

  // A delay target beyond the stop must not skip to the later restart either.
  const auto after_stop = longitudinal_utils::findTemporalStop(trajectory, 0.9, 1.05);
  EXPECT_FALSE(after_stop.departure_requested);
  const auto target =
    longitudinal_utils::calcTemporalLookaheadPoint(trajectory, 1.05, 0.0, after_stop);
  EXPECT_FLOAT_EQ(target.longitudinal_velocity_mps, 0.0F);
}

TEST(TestLongitudinalControllerUtils, temporalLookaheadUsesTimeAtRepeatedPositions)
{
  // Geometry provides no useful lookahead distance here; temporal interpolation still does.
  const auto trajectory = makeTemporalTrajectory(
    {{0.1, 0.0, 0.0F},
     {0.2, 0.0, 0.0F},
     {0.3, 0.0, 0.1F, 0.2F},
     {0.5, 0.0, 0.3F, 0.4F},
     {2.0, 0.2, 0.5F}});
  const auto stop = longitudinal_utils::findTemporalStop(trajectory, 0.1, 0.25);
  const auto future = longitudinal_utils::calcTemporalLookaheadPoint(trajectory, 0.25, 0.15, stop);
  EXPECT_NEAR(rclcpp::Duration(future.time_from_start).seconds(), 0.4, 1.0e-8);
  EXPECT_NEAR(future.longitudinal_velocity_mps, 0.2, 1.0e-6);
  EXPECT_NEAR(future.acceleration_mps2, 0.3, 1.0e-6);
}

TEST(TestLongitudinalControllerUtils, temporalAllStoppedAndTinyNoiseRemainStopped)
{
  const auto trajectory =
    makeTemporalTrajectory({{0.1, 0.0, 0.0F}, {0.2, 0.0, 0.0001F}, {0.3, 0.0, 0.0F}});
  const auto stop = longitudinal_utils::findTemporalStop(trajectory, 0.1, 0.25);
  EXPECT_FALSE(stop.departure_requested);
  EXPECT_FALSE(stop.allowsDeparture(2.0, 1.0));
  EXPECT_TRUE(stop.requiresStopping(2.0, 0.49));
}

TEST(TestLongitudinalControllerUtils, temporalMovingEndpointExpiresWithoutBecomingAStopEarly)
{
  const auto trajectory = makeTemporalTrajectory({{0.1, 0.0, 0.1F}, {0.3, 0.02, 0.2F}});
  const auto active = longitudinal_utils::findTemporalStop(trajectory, 0.2, 0.35);
  EXPECT_TRUE(active.allowsDeparture(0.02, 1.0));
  const auto future = longitudinal_utils::calcTemporalLookaheadPoint(trajectory, 0.35, 1.0, active);
  EXPECT_NEAR(rclcpp::Duration(future.time_from_start).seconds(), 0.3, 1.0e-8);
  EXPECT_FLOAT_EQ(future.longitudinal_velocity_mps, 0.2F);

  const auto expired = longitudinal_utils::findTemporalStop(trajectory, 0.4, 0.55);
  EXPECT_FALSE(expired.allowsDeparture(10.0, 1.0));
  EXPECT_TRUE(expired.requiresStopping(10.0, 0.49));
  const auto not_started = longitudinal_utils::findTemporalStop(trajectory, -0.2, -0.05);
  EXPECT_FALSE(not_started.departure_requested);
}

TEST(TestLongitudinalControllerUtils, temporalReverseLaunchAndUnscheduledDirectionChange)
{
  const auto reverse = makeTemporalTrajectory(
    {{0.1, 0.0, 0.0F}, {0.2, 0.0, 0.0F}, {0.3, -0.005, -0.1F}, {0.8, -0.2, -0.3F}});
  const auto stop = longitudinal_utils::findTemporalStop(reverse, 0.1, 0.25);
  EXPECT_FALSE(stop.forward);
  EXPECT_TRUE(stop.allowsDeparture(0.2, 1.0));
  EXPECT_FALSE(stop.requiresStopping(0.2, 0.49));

  const auto reversal =
    makeTemporalTrajectory({{0.1, 0.0, 0.1F}, {0.2, 0.1, 0.1F}, {0.3, 0.2, -0.1F}});
  const auto blocked = longitudinal_utils::findTemporalStop(reversal, 0.15, 0.25);
  ASSERT_TRUE(blocked.stop_idx.has_value());
  EXPECT_EQ(*blocked.stop_idx, 1U);
  EXPECT_FALSE(blocked.departure_requested);
  const auto target = longitudinal_utils::calcTemporalLookaheadPoint(reversal, 0.25, 0.0, blocked);
  EXPECT_FLOAT_EQ(target.longitudinal_velocity_mps, 0.0F);
}

TEST(TestLongitudinalControllerUtils, temporalStopDistancePreservesOvershootSign)
{
  const auto trajectory = makeTemporalTrajectory(
    {{0.0, 0.0, 0.5F}, {1.0, 0.5, 0.0F}, {2.0, 1.0, 0.5F}, {3.0, 2.0, 0.5F}});
  const auto stop = longitudinal_utils::findTemporalStop(trajectory, 1.2, 1.35);
  ASSERT_TRUE(stop.stop_idx.has_value());
  geometry_msgs::msg::Pose ego;
  ego.position.x = 1.5;
  ego.orientation.w = 1.0;
  const double distance =
    longitudinal_utils::calcStopDistance(ego, trajectory, 3.0, 0.7, *stop.stop_idx);
  EXPECT_NEAR(distance, -1.0, 1.0e-6);
  EXPECT_FALSE(stop.allowsDeparture(distance, 1.0));
  EXPECT_TRUE(stop.requiresStopping(distance, 0.49));
}

TEST(TestLongitudinalControllerUtils, temporalInvalidInputsCannotRequestDeparture)
{
  auto trajectory = makeTemporalTrajectory({{0.1, 0.0, 0.0F}, {0.2, 0.1, 0.2F}, {0.3, 0.2, 0.3F}});
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(longitudinal_utils::findTemporalStop(trajectory, nan, 0.25).departure_requested);
  EXPECT_FALSE(longitudinal_utils::findTemporalStop(trajectory, 0.1, nan).departure_requested);
  EXPECT_FALSE(longitudinal_utils::findTemporalStop(trajectory, 0.2, 0.1).departure_requested);
  trajectory.points[1].time_from_start = trajectory.points[0].time_from_start;
  EXPECT_FALSE(longitudinal_utils::findTemporalStop(trajectory, 0.1, 0.25).departure_requested);
  trajectory.points[1].time_from_start = rclcpp::Duration::from_seconds(0.2);
  trajectory.points[1].longitudinal_velocity_mps = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(longitudinal_utils::findTemporalStop(trajectory, 0.1, 0.25).departure_requested);
  trajectory.points.clear();
  EXPECT_FALSE(longitudinal_utils::findTemporalStop(trajectory, 0.1, 0.25).departure_requested);
}

TEST(TestLongitudinalControllerUtils, isValidTrajectory)
{
  using autoware_planning_msgs::msg::Trajectory;
  using autoware_planning_msgs::msg::TrajectoryPoint;
  Trajectory traj;
  TrajectoryPoint point;
  EXPECT_FALSE(longitudinal_utils::isValidTrajectory(traj));
  traj.points.push_back(point);
  EXPECT_TRUE(longitudinal_utils::isValidTrajectory(traj));
  point.pose.position.x = std::numeric_limits<decltype(point.pose.position.x)>::infinity();
  traj.points.push_back(point);
  EXPECT_FALSE(longitudinal_utils::isValidTrajectory(traj));
}

TEST(TestLongitudinalControllerUtils, calcStopDistance)
{
  using autoware_planning_msgs::msg::Trajectory;
  using autoware_planning_msgs::msg::TrajectoryPoint;
  using geometry_msgs::msg::Pose;
  Pose current_pose;
  current_pose.position.x = 0.0;
  current_pose.position.y = 0.0;
  current_pose.position.z = 0.0;
  Trajectory traj;
  double max_dist = 3.0;
  double max_yaw = 0.7;
  // empty trajectory : exception
  EXPECT_THROW(
    longitudinal_utils::calcStopDistance(current_pose, traj, max_dist, max_yaw),
    std::invalid_argument);
  // one point trajectory : exception
  TrajectoryPoint point;
  point.pose.position.x = 0.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 0.0;
  traj.points.push_back(point);
  EXPECT_DOUBLE_EQ(
    longitudinal_utils::calcStopDistance(current_pose, traj, max_dist, max_yaw), 0.0);
  traj.points.clear();
  // non stopping trajectory: stop distance = trajectory length
  point.pose.position.x = 0.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 1.0;
  traj.points.push_back(point);
  point.pose.position.x = 1.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 1.0;
  traj.points.push_back(point);
  point.pose.position.x = 2.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 1.0;
  traj.points.push_back(point);
  EXPECT_EQ(longitudinal_utils::calcStopDistance(current_pose, traj, max_dist, max_yaw), 2.0);
  // stopping trajectory: stop distance = length until stopping point
  point.pose.position.x = 3.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 0.0;
  traj.points.push_back(point);
  point.pose.position.x = 4.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 1.0;
  traj.points.push_back(point);
  point.pose.position.x = 5.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 0.0;
  traj.points.push_back(point);
  EXPECT_EQ(longitudinal_utils::calcStopDistance(current_pose, traj, max_dist, max_yaw), 3.0);
}

TEST(TestLongitudinalControllerUtils, getPitchByPose)
{
  tf2::Quaternion quaternion_tf;
  quaternion_tf.setRPY(0.0, 0.0, 0.0);
  EXPECT_EQ(longitudinal_utils::getPitchByPose(tf2::toMsg(quaternion_tf)), 0.0);
  quaternion_tf.setRPY(0.0, 1.0, 0.0);
  EXPECT_EQ(longitudinal_utils::getPitchByPose(tf2::toMsg(quaternion_tf)), 1.0);
}

TEST(TestLongitudinalControllerUtils, getPitchByTraj)
{
  using autoware_planning_msgs::msg::Trajectory;
  using autoware_planning_msgs::msg::TrajectoryPoint;
  const double wheel_base = 0.9;
  /**
   * Trajectory:
   * 1    X
   *            X
   * 0 X     X
   *   0  1  2  3
   */
  Trajectory traj;
  TrajectoryPoint point;
  point.pose.position.x = 0.0;
  point.pose.position.y = 0.0;
  point.pose.position.z = 0.0;
  traj.points.push_back(point);
  // non stopping trajectory: stop distance = trajectory length
  point.pose.position.x = 0.6;
  point.pose.position.y = 0.0;
  point.pose.position.z = 0.8;
  traj.points.push_back(point);
  point.pose.position.x = 1.2;
  point.pose.position.y = 0.0;
  point.pose.position.z = 0.0;
  traj.points.push_back(point);
  point.pose.position.x = 1.8;
  point.pose.position.y = 0.0;
  point.pose.position.z = 0.8;
  traj.points.push_back(point);
  size_t closest_idx = 0;
  EXPECT_DOUBLE_EQ(
    longitudinal_utils::getPitchByTraj(traj, closest_idx, wheel_base), std::atan2(0.8, 0.6));
  closest_idx = 1;
  EXPECT_DOUBLE_EQ(
    longitudinal_utils::getPitchByTraj(traj, closest_idx, wheel_base), std::atan2(-0.8, 0.6));
  closest_idx = 2;
  EXPECT_DOUBLE_EQ(
    longitudinal_utils::getPitchByTraj(traj, closest_idx, wheel_base), std::atan2(0.8, 0.6));
  closest_idx = 3;
  EXPECT_DOUBLE_EQ(
    longitudinal_utils::getPitchByTraj(traj, closest_idx, wheel_base), std::atan2(0.8, 0.6));
}

TEST(TestLongitudinalControllerUtils, calcPoseAfterTimeDelay)
{
  using geometry_msgs::msg::Pose;
  const double abs_err = 1e-7;
  Pose current_pose;
  current_pose.position.x = 0.0;
  current_pose.position.y = 0.0;
  current_pose.position.z = 0.0;
  tf2::Quaternion quaternion_tf;
  quaternion_tf.setRPY(0.0, 0.0, 0.0);
  current_pose.orientation = tf2::toMsg(quaternion_tf);

  // With a delay acceleration and/or a velocity of 0.0 there is no change of position
  double delay_time = 0.0;
  double current_vel = 0.0;
  double current_acc = 0.0;
  Pose delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x, abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  delay_time = 1.0;
  current_vel = 0.0;
  current_acc = 0.0;
  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x, abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  delay_time = 0.0;
  current_vel = 1.0;
  current_acc = 0.0;
  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x, abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  // With both delay and velocity: change of position
  delay_time = 1.0;
  current_vel = 1.0;
  current_acc = 0.0;

  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x + current_vel * delay_time, abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  // With all, acceleration, delay and velocity: change of position
  delay_time = 1.0;
  current_vel = 1.0;
  current_acc = 1.0;

  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(
    delayed_pose.position.x,
    current_pose.position.x + current_vel * delay_time +
      0.5 * current_acc * delay_time * delay_time,
    abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  // Vary the yaw
  quaternion_tf.setRPY(0.0, 0.0, M_PI);
  current_pose.orientation = tf2::toMsg(quaternion_tf);
  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(
    delayed_pose.position.x,
    current_pose.position.x - current_vel * delay_time -
      0.5 * current_acc * delay_time * delay_time,
    abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  quaternion_tf.setRPY(0.0, 0.0, M_PI_2);
  current_pose.orientation = tf2::toMsg(quaternion_tf);
  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x, abs_err);
  EXPECT_NEAR(
    delayed_pose.position.y,
    current_pose.position.y + current_vel * delay_time +
      0.5 * current_acc * delay_time * delay_time,
    abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  quaternion_tf.setRPY(0.0, 0.0, -M_PI_2);
  current_pose.orientation = tf2::toMsg(quaternion_tf);
  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x, abs_err);
  EXPECT_NEAR(
    delayed_pose.position.y,
    current_pose.position.y - current_vel * delay_time -
      0.5 * current_acc * delay_time * delay_time,
    abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  // Vary the pitch : no effect /!\ NOTE: bug with roll of +-PI/2 which rotates the yaw by PI
  quaternion_tf.setRPY(0.0, M_PI_4, 0.0);
  current_pose.orientation = tf2::toMsg(quaternion_tf);
  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(
    delayed_pose.position.x,
    current_pose.position.x + current_vel * delay_time +
      0.5 * current_acc * delay_time * delay_time,
    abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  // Vary the roll : no effect
  quaternion_tf.setRPY(M_PI_2, 0.0, 0.0);
  current_pose.orientation = tf2::toMsg(quaternion_tf);
  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(
    delayed_pose.position.x,
    current_pose.position.x + current_vel * delay_time +
      0.5 * current_acc * delay_time * delay_time,
    abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);
}

TEST(TestLongitudinalControllerUtils, lerpOrientation)
{
  geometry_msgs::msg::Quaternion result;
  tf2::Quaternion o_from;
  tf2::Quaternion o_to;
  tf2::Quaternion o_result;
  double roll;
  double pitch;
  double yaw;
  double ratio;

  o_from.setRPY(0.0, 0.0, 0.0);
  o_to.setRPY(M_PI_4, M_PI_4, M_PI_4);

  ratio = 0.0;
  result = autoware::interpolation::lerpOrientation(tf2::toMsg(o_from), tf2::toMsg(o_to), ratio);
  tf2::convert(result, o_result);
  tf2::Matrix3x3(o_result).getRPY(roll, pitch, yaw);
  EXPECT_DOUBLE_EQ(roll, 0.0);
  EXPECT_DOUBLE_EQ(pitch, 0.0);
  EXPECT_DOUBLE_EQ(yaw, 0.0);

  ratio = 1.0;
  result = autoware::interpolation::lerpOrientation(tf2::toMsg(o_from), tf2::toMsg(o_to), ratio);
  tf2::convert(result, o_result);
  tf2::Matrix3x3(o_result).getRPY(roll, pitch, yaw);
  EXPECT_DOUBLE_EQ(roll, M_PI_4);
  EXPECT_DOUBLE_EQ(pitch, M_PI_4);
  EXPECT_DOUBLE_EQ(yaw, M_PI_4);

  ratio = 0.5;
  o_to.setRPY(M_PI_4, 0.0, 0.0);
  result = autoware::interpolation::lerpOrientation(tf2::toMsg(o_from), tf2::toMsg(o_to), ratio);
  tf2::convert(result, o_result);
  tf2::Matrix3x3(o_result).getRPY(roll, pitch, yaw);
  EXPECT_DOUBLE_EQ(roll, M_PI_4 / 2);
  EXPECT_DOUBLE_EQ(pitch, 0.0);
  EXPECT_DOUBLE_EQ(yaw, 0.0);

  o_to.setRPY(0.0, M_PI_4, 0.0);
  result = autoware::interpolation::lerpOrientation(tf2::toMsg(o_from), tf2::toMsg(o_to), ratio);
  tf2::convert(result, o_result);
  tf2::Matrix3x3(o_result).getRPY(roll, pitch, yaw);
  EXPECT_DOUBLE_EQ(roll, 0.0);
  EXPECT_DOUBLE_EQ(pitch, M_PI_4 / 2);
  EXPECT_DOUBLE_EQ(yaw, 0.0);

  o_to.setRPY(0.0, 0.0, M_PI_4);
  result = autoware::interpolation::lerpOrientation(tf2::toMsg(o_from), tf2::toMsg(o_to), ratio);
  tf2::convert(result, o_result);
  tf2::Matrix3x3(o_result).getRPY(roll, pitch, yaw);
  EXPECT_DOUBLE_EQ(roll, 0.0);
  EXPECT_DOUBLE_EQ(pitch, 0.0);
  EXPECT_DOUBLE_EQ(yaw, M_PI_4 / 2);
}

TEST(TestLongitudinalControllerUtils, lerpTrajectoryPoint)
{
  using autoware_planning_msgs::msg::TrajectoryPoint;
  using geometry_msgs::msg::Pose;
  const double abs_err = 1e-15;
  decltype(autoware_planning_msgs::msg::Trajectory::points) points;
  TrajectoryPoint p;
  p.pose.position.x = 0.0;
  p.pose.position.y = 0.0;
  p.pose.position.z = 0.0;
  p.longitudinal_velocity_mps = 10.0;
  p.acceleration_mps2 = 10.0;
  points.push_back(p);
  p.pose.position.x = 1.0;
  p.pose.position.y = 0.0;
  p.pose.position.z = 0.0;
  p.longitudinal_velocity_mps = 20.0;
  p.acceleration_mps2 = 20.0;
  points.push_back(p);
  p.pose.position.x = 1.0;
  p.pose.position.y = 1.0;
  p.pose.position.z = 1.0;
  p.longitudinal_velocity_mps = 30.0;
  p.acceleration_mps2 = 30.0;
  points.push_back(p);
  p.pose.position.x = 2.0;
  p.pose.position.y = 1.0;
  p.pose.position.z = 2.0;
  p.longitudinal_velocity_mps = 40.0;
  p.acceleration_mps2 = 40.0;
  points.push_back(p);
  Pose pose;
  double max_dist = 3.0;
  double max_yaw = 0.7;
  // Points on the trajectory gives back the original trajectory points values
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;

  auto result = longitudinal_utils::lerpTrajectoryPoint(points, pose, max_dist, max_yaw);
  EXPECT_NEAR(result.first.pose.position.x, pose.position.x, abs_err);
  EXPECT_NEAR(result.first.pose.position.y, pose.position.y, abs_err);
  EXPECT_NEAR(result.first.pose.position.z, pose.position.z, abs_err);
  EXPECT_NEAR(result.first.longitudinal_velocity_mps, 10.0, abs_err);
  EXPECT_NEAR(result.first.acceleration_mps2, 10.0, abs_err);

  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  result = longitudinal_utils::lerpTrajectoryPoint(points, pose, max_dist, max_yaw);
  EXPECT_NEAR(result.first.pose.position.x, pose.position.x, abs_err);
  EXPECT_NEAR(result.first.pose.position.y, pose.position.y, abs_err);
  EXPECT_NEAR(result.first.pose.position.z, pose.position.z, abs_err);
  EXPECT_NEAR(result.first.longitudinal_velocity_mps, 20.0, abs_err);
  EXPECT_NEAR(result.first.acceleration_mps2, 20.0, abs_err);

  pose.position.x = 1.0;
  pose.position.y = 1.0;
  pose.position.z = 1.0;
  result = longitudinal_utils::lerpTrajectoryPoint(points, pose, max_dist, max_yaw);
  EXPECT_NEAR(result.first.pose.position.x, pose.position.x, abs_err);
  EXPECT_NEAR(result.first.pose.position.y, pose.position.y, abs_err);
  EXPECT_NEAR(result.first.pose.position.z, pose.position.z, abs_err);
  EXPECT_NEAR(result.first.longitudinal_velocity_mps, 30.0, abs_err);
  EXPECT_NEAR(result.first.acceleration_mps2, 30.0, abs_err);

  pose.position.x = 2.0;
  pose.position.y = 1.0;
  pose.position.z = 2.0;
  result = longitudinal_utils::lerpTrajectoryPoint(points, pose, max_dist, max_yaw);
  EXPECT_NEAR(result.first.pose.position.x, pose.position.x, abs_err);
  EXPECT_NEAR(result.first.pose.position.y, pose.position.y, abs_err);
  EXPECT_NEAR(result.first.pose.position.z, pose.position.z, abs_err);
  EXPECT_NEAR(result.first.longitudinal_velocity_mps, 40.0, abs_err);
  EXPECT_NEAR(result.first.acceleration_mps2, 40.0, abs_err);

  // Interpolate between trajectory points
  pose.position.x = 0.5;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  result = longitudinal_utils::lerpTrajectoryPoint(points, pose, max_dist, max_yaw);
  EXPECT_NEAR(result.first.pose.position.x, pose.position.x, abs_err);
  EXPECT_NEAR(result.first.pose.position.y, pose.position.y, abs_err);
  EXPECT_NEAR(result.first.pose.position.z, pose.position.z, abs_err);
  EXPECT_NEAR(result.first.longitudinal_velocity_mps, 15.0, abs_err);
  EXPECT_NEAR(result.first.acceleration_mps2, 15.0, abs_err);
  pose.position.x = 0.75;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  result = longitudinal_utils::lerpTrajectoryPoint(points, pose, max_dist, max_yaw);

  EXPECT_NEAR(result.first.pose.position.x, pose.position.x, abs_err);
  EXPECT_NEAR(result.first.pose.position.y, pose.position.y, abs_err);
  EXPECT_NEAR(result.first.pose.position.z, pose.position.z, abs_err);
  EXPECT_NEAR(result.first.longitudinal_velocity_mps, 17.5, abs_err);
  EXPECT_NEAR(result.first.acceleration_mps2, 17.5, abs_err);

  // Interpolate away from the trajectory (interpolated point is projected)
  pose.position.x = 0.5;
  pose.position.y = -1.0;
  result = longitudinal_utils::lerpTrajectoryPoint(points, pose, max_dist, max_yaw);
  EXPECT_NEAR(result.first.pose.position.x, pose.position.x, abs_err);
  EXPECT_NEAR(result.first.pose.position.y, 0.0, abs_err);
  EXPECT_NEAR(result.first.pose.position.z, pose.position.z, abs_err);
  EXPECT_NEAR(result.first.longitudinal_velocity_mps, 15.0, abs_err);
  EXPECT_NEAR(result.first.acceleration_mps2, 15.0, abs_err);

  // Ambiguous projections: possibility with the lowest index is used
  pose.position.x = 0.5;
  pose.position.y = 0.5;
  result = longitudinal_utils::lerpTrajectoryPoint(points, pose, max_dist, max_yaw);
  EXPECT_NEAR(result.first.pose.position.x, pose.position.x, abs_err);
  EXPECT_NEAR(result.first.pose.position.y, 0.0, abs_err);
  EXPECT_NEAR(result.first.pose.position.z, pose.position.z, abs_err);
  EXPECT_NEAR(result.first.longitudinal_velocity_mps, 15.0, abs_err);
  EXPECT_NEAR(result.first.acceleration_mps2, 15.0, abs_err);
}

TEST(TestLongitudinalControllerUtils, lerpTrajectoryPointByTimeStopHold)
{
  using autoware_planning_msgs::msg::TrajectoryPoint;
  std::vector<TrajectoryPoint> points;

  TrajectoryPoint p;
  p.pose.position.x = 0.0;
  p.longitudinal_velocity_mps = 0.0;
  p.time_from_start = rclcpp::Duration::from_seconds(0.0);
  points.push_back(p);

  p.time_from_start = rclcpp::Duration::from_seconds(0.1);
  points.push_back(p);

  p.time_from_start = rclcpp::Duration::from_seconds(0.2);
  points.push_back(p);

  p.pose.position.x = 0.2;
  p.longitudinal_velocity_mps = 1.0;
  p.time_from_start = rclcpp::Duration::from_seconds(0.3);
  points.push_back(p);

  const auto stop_hold = longitudinal_utils::lerpTrajectoryPointByTime(points, 0.15);
  EXPECT_EQ(stop_hold.second, 1U);
  EXPECT_DOUBLE_EQ(rclcpp::Duration(stop_hold.first.time_from_start).seconds(), 0.15);
  EXPECT_DOUBLE_EQ(stop_hold.first.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(stop_hold.first.longitudinal_velocity_mps, 0.0);
}

TEST(TestLongitudinalControllerUtils, lerpTrajectoryPointByTimeRestartIsSmooth)
{
  using autoware_planning_msgs::msg::TrajectoryPoint;
  std::vector<TrajectoryPoint> points;

  TrajectoryPoint p;
  p.pose.position.x = 0.0;
  p.longitudinal_velocity_mps = 0.0;
  p.time_from_start = rclcpp::Duration::from_seconds(0.0);
  points.push_back(p);

  p.time_from_start = rclcpp::Duration::from_seconds(0.1);
  points.push_back(p);

  p.pose.position.x = 0.1;
  p.longitudinal_velocity_mps = 0.5;
  p.time_from_start = rclcpp::Duration::from_seconds(0.2);
  points.push_back(p);

  p.pose.position.x = 0.3;
  p.longitudinal_velocity_mps = 1.0;
  p.time_from_start = rclcpp::Duration::from_seconds(0.3);
  points.push_back(p);

  const auto early_restart = longitudinal_utils::lerpTrajectoryPointByTime(points, 0.15);
  EXPECT_EQ(early_restart.second, 1U);
  EXPECT_DOUBLE_EQ(rclcpp::Duration(early_restart.first.time_from_start).seconds(), 0.15);
  EXPECT_NEAR(early_restart.first.pose.position.x, 0.05, 1e-12);
  EXPECT_NEAR(early_restart.first.longitudinal_velocity_mps, 0.25, 1e-12);

  const auto late_restart = longitudinal_utils::lerpTrajectoryPointByTime(points, 0.25);
  EXPECT_EQ(late_restart.second, 2U);
  EXPECT_DOUBLE_EQ(rclcpp::Duration(late_restart.first.time_from_start).seconds(), 0.25);
  EXPECT_NEAR(late_restart.first.pose.position.x, 0.2, 1e-12);
  EXPECT_NEAR(late_restart.first.longitudinal_velocity_mps, 0.75, 1e-12);

  EXPECT_LT(
    early_restart.first.longitudinal_velocity_mps, late_restart.first.longitudinal_velocity_mps);
}

TEST(TestLongitudinalControllerUtils, applyDiffLimitFilter)
{
  double dt = 1.0;
  double max_val = 0.0;  // cannot increase
  double min_val = 0.0;  // cannot decrease
  double prev_val = 0.0;

  double input_val = 10.0;
  EXPECT_DOUBLE_EQ(
    longitudinal_utils::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), 0.0);

  max_val = 1.0;  // can only increase by up to 1.0 at a time
  EXPECT_DOUBLE_EQ(
    longitudinal_utils::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), 1.0);

  input_val = -10.0;
  EXPECT_DOUBLE_EQ(
    longitudinal_utils::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), 0.0);

  min_val = -1.0;  // can decrease by up to -1.0 at a time
  EXPECT_DOUBLE_EQ(
    longitudinal_utils::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), -1.0);

  dt = 5.0;  // can now increase/decrease 5 times more
  input_val = 10.0;
  EXPECT_DOUBLE_EQ(
    longitudinal_utils::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), 5.0);
  input_val = -10.0;
  EXPECT_DOUBLE_EQ(
    longitudinal_utils::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), -5.0);

  dt = 1.0;
  input_val = 100.0;
  for (double prev = 0.0; prev < 100.0; ++prev) {
    const double new_val =
      longitudinal_utils::applyDiffLimitFilter(input_val, prev, dt, max_val, min_val);
    EXPECT_DOUBLE_EQ(new_val, prev + max_val);
    prev = new_val;
  }
}

TEST(TestLongitudinalControllerUtils, findTrajectoryPoseAfterDistance)
{
  using autoware_planning_msgs::msg::Trajectory;
  using autoware_planning_msgs::msg::TrajectoryPoint;
  using geometry_msgs::msg::Pose;
  const double abs_err = 1e-5;
  Trajectory traj;
  TrajectoryPoint point;
  point.pose.position.x = 0.0;
  point.pose.position.y = 0.0;
  point.pose.position.z = 0.0;
  traj.points.push_back(point);
  point.pose.position.x = 1.0;
  point.pose.position.y = 0.0;
  point.pose.position.z = 0.0;
  traj.points.push_back(point);
  point.pose.position.x = 1.0;
  point.pose.position.y = 1.0;
  point.pose.position.z = 1.0;
  traj.points.push_back(point);
  point.pose.position.x = 2.0;
  point.pose.position.y = 1.0;
  point.pose.position.z = 2.0;
  traj.points.push_back(point);
  size_t src_idx = 0;
  double distance = 0.0;
  Pose result = longitudinal_utils::findTrajectoryPoseAfterDistance(src_idx, distance, traj);
  EXPECT_NEAR(result.position.x, 0.0, abs_err);
  EXPECT_NEAR(result.position.y, 0.0, abs_err);
  EXPECT_NEAR(result.position.z, 0.0, abs_err);

  src_idx = 0;
  distance = 0.5;
  result = longitudinal_utils::findTrajectoryPoseAfterDistance(src_idx, distance, traj);
  EXPECT_NEAR(result.position.x, 0.5, abs_err);
  EXPECT_NEAR(result.position.y, 0.0, abs_err);
  EXPECT_NEAR(result.position.z, 0.0, abs_err);

  src_idx = 0;
  distance = 1.0;
  result = longitudinal_utils::findTrajectoryPoseAfterDistance(src_idx, distance, traj);
  EXPECT_NEAR(result.position.x, 1.0, abs_err);
  EXPECT_NEAR(result.position.y, 0.0, abs_err);
  EXPECT_NEAR(result.position.z, 0.0, abs_err);

  src_idx = 0;
  distance = 1.5;
  result = longitudinal_utils::findTrajectoryPoseAfterDistance(src_idx, distance, traj);
  EXPECT_NEAR(result.position.x, 1.0, abs_err);
  EXPECT_NEAR(result.position.y, 1.0 / (2.0 * sqrt(2.0)), abs_err);
  EXPECT_NEAR(result.position.z, 1.0 / (2.0 * sqrt(2.0)), abs_err);

  src_idx = 0;
  distance = 20.0;  // beyond the trajectory, should return the last point
  result = longitudinal_utils::findTrajectoryPoseAfterDistance(src_idx, distance, traj);
  EXPECT_NEAR(result.position.x, 2.0, abs_err);
  EXPECT_NEAR(result.position.y, 1.0, abs_err);
  EXPECT_NEAR(result.position.z, 2.0, abs_err);
}
