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

#include "../../../src/filters/safety/collision_check_filter/stop_tracker.hpp"

#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <gtest/gtest.h>

#include <cstdint>

namespace autoware::trajectory_validator::plugin::safety
{
namespace
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using nav_msgs::msg::Odometry;
using unique_identifier_msgs::msg::UUID;

rclcpp::Time make_time(const int64_t nanoseconds)
{
  return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

UUID make_uuid(const uint8_t value)
{
  UUID id;
  id.uuid.fill(value);
  return id;
}

PredictedObject make_object(const UUID & id, const double velocity_x, const double velocity_y = 0.0)
{
  PredictedObject object;
  object.object_id = id;
  auto & linear_velocity = object.kinematics.initial_twist_with_covariance.twist.linear;
  linear_velocity.x = velocity_x;
  linear_velocity.y = velocity_y;
  return object;
}

PredictedObjects make_objects(const rclcpp::Time & stamp)
{
  PredictedObjects objects;
  objects.header.stamp = stamp;
  return objects;
}

Odometry make_odometry(
  const rclcpp::Time & stamp, const double velocity_x, const double velocity_y = 0.0)
{
  Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.twist.twist.linear.x = velocity_x;
  odometry.twist.twist.linear.y = velocity_y;
  return odometry;
}

void update_with_object(
  ObjectStopTracker & tracker, const rclcpp::Time & stamp, const PredictedObject & object)
{
  auto objects = make_objects(stamp);
  objects.objects.push_back(object);
  tracker.update(objects);
}
}  // namespace

TEST(StopTrackingParams, ConvertsEgoAndObjectParametersIndependently)
{
  validator::Params node_params;
  auto & stop_tracking = node_params.collision_check.drac.stop_tracking;
  stop_tracking.ego.stopped_velocity_threshold = 0.01;
  stop_tracking.ego.history_timeout = 0.2;
  stop_tracking.object.stopped_velocity_threshold = 0.3;
  stop_tracking.object.history_timeout = 0.4;

  const StopTrackingParams ego_params(stop_tracking.ego);
  EXPECT_DOUBLE_EQ(ego_params.stopped_velocity_threshold, 0.01);
  EXPECT_DOUBLE_EQ(ego_params.history_timeout, 0.2);

  const StopTrackingParams object_params(stop_tracking.object);
  EXPECT_DOUBLE_EQ(object_params.stopped_velocity_threshold, 0.3);
  EXPECT_DOUBLE_EQ(object_params.history_timeout, 0.4);
}

TEST(DracParams, ExtractsMutualYieldArbitrationPerObjectLabel)
{
  validator::Params node_params;
  auto & mutual_yield = node_params.collision_check.drac.map_based.mutual_yield_timeout_arbitration;
  mutual_yield.enabled.base = false;
  mutual_yield.enabled.pedestrian = true;
  mutual_yield.enabled.bicycle = true;
  mutual_yield.min_wait_time = 3.0;

  const DracParams car_params(node_params, "car");
  EXPECT_FALSE(car_params.map_based.mutual_yield_timeout_arbitration.enabled);
  EXPECT_DOUBLE_EQ(car_params.map_based.mutual_yield_timeout_arbitration.min_wait_time, 3.0);

  const DracParams pedestrian_params(node_params, "pedestrian");
  EXPECT_TRUE(pedestrian_params.map_based.mutual_yield_timeout_arbitration.enabled);
  EXPECT_DOUBLE_EQ(pedestrian_params.map_based.mutual_yield_timeout_arbitration.min_wait_time, 3.0);

  const DracParams bicycle_params(node_params, "bicycle");
  EXPECT_TRUE(bicycle_params.map_based.mutual_yield_timeout_arbitration.enabled);
  EXPECT_DOUBLE_EQ(bicycle_params.map_based.mutual_yield_timeout_arbitration.min_wait_time, 3.0);
}

TEST(ObjectStopTracker, ReportsContinuousStopDuration)
{
  StopTrackingParams params;
  params.history_timeout = 10.0;
  ObjectStopTracker tracker(params);
  const auto id = make_uuid(1);
  const auto stopped_object = make_object(id, 0.0);

  update_with_object(tracker, make_time(10'000'000'000LL), stopped_object);
  update_with_object(tracker, make_time(12'999'999'999LL), stopped_object);
  const auto duration_before_three_seconds = tracker.get_stopped_duration(id);
  ASSERT_TRUE(duration_before_three_seconds.has_value());
  EXPECT_NEAR(duration_before_three_seconds.value().seconds(), 2.999999999, 1e-9);

  update_with_object(tracker, make_time(13'000'000'000LL), stopped_object);
  const auto duration_at_three_seconds = tracker.get_stopped_duration(id);
  ASSERT_TRUE(duration_at_three_seconds.has_value());
  EXPECT_DOUBLE_EQ(duration_at_three_seconds.value().seconds(), 3.0);
}

TEST(ObjectStopTracker, MovingObjectRemovesAndResetsItsStopHistory)
{
  StopTrackingParams params;
  params.stopped_velocity_threshold = 0.1;
  params.history_timeout = 10.0;
  ObjectStopTracker tracker(params);
  const auto id = make_uuid(2);

  update_with_object(tracker, make_time(10'000'000'000LL), make_object(id, 0.0));
  update_with_object(tracker, make_time(13'000'000'000LL), make_object(id, 0.0));
  ASSERT_TRUE(tracker.get_stopped_duration(id).has_value());
  EXPECT_DOUBLE_EQ(tracker.get_stopped_duration(id).value().seconds(), 3.0);

  update_with_object(tracker, make_time(14'000'000'000LL), make_object(id, 0.1));
  EXPECT_FALSE(tracker.get_stopped_duration(id).has_value());

  update_with_object(tracker, make_time(15'000'000'000LL), make_object(id, 0.0));
  update_with_object(tracker, make_time(17'999'999'999LL), make_object(id, 0.0));
  const auto duration_after_restart = tracker.get_stopped_duration(id);
  ASSERT_TRUE(duration_after_restart.has_value());
  EXPECT_NEAR(duration_after_restart.value().seconds(), 2.999999999, 1e-9);

  update_with_object(tracker, make_time(18'000'000'000LL), make_object(id, 0.0));
  ASSERT_TRUE(tracker.get_stopped_duration(id).has_value());
  EXPECT_DOUBLE_EQ(tracker.get_stopped_duration(id).value().seconds(), 3.0);
}

TEST(ObjectStopTracker, RemovesUnobservedObjectAfterHistoryTimeout)
{
  StopTrackingParams params;
  params.history_timeout = 1.0;
  ObjectStopTracker tracker(params);
  const auto id = make_uuid(3);

  update_with_object(tracker, make_time(10'000'000'000LL), make_object(id, 0.0));

  auto empty_objects = make_objects(make_time(11'000'000'000LL));
  tracker.update(empty_objects);
  ASSERT_TRUE(tracker.get_stopped_duration(id).has_value());
  EXPECT_DOUBLE_EQ(tracker.get_stopped_duration(id).value().seconds(), 1.0);

  empty_objects.header.stamp = make_time(11'000'000'001LL);
  tracker.update(empty_objects);
  EXPECT_FALSE(tracker.get_stopped_duration(id).has_value());
}

TEST(ObjectStopTracker, UsesPlanarSpeedForStopClassification)
{
  StopTrackingParams params;
  params.stopped_velocity_threshold = 0.1;
  ObjectStopTracker tracker(params);
  const auto id = make_uuid(4);

  update_with_object(tracker, make_time(10'000'000'000LL), make_object(id, 0.06, 0.06));
  EXPECT_TRUE(tracker.get_stopped_duration(id).has_value());

  update_with_object(tracker, make_time(11'000'000'000LL), make_object(id, 0.08, 0.08));
  EXPECT_FALSE(tracker.get_stopped_duration(id).has_value());
}

TEST(ObjectStopTracker, ResetsHistoryWhenObservationTimeMovesBackward)
{
  StopTrackingParams params;
  params.history_timeout = 10.0;
  ObjectStopTracker tracker(params);
  const auto id = make_uuid(5);
  const auto stopped_object = make_object(id, 0.0);

  update_with_object(tracker, make_time(10'000'000'000LL), stopped_object);
  update_with_object(tracker, make_time(13'000'000'000LL), stopped_object);
  ASSERT_TRUE(tracker.get_stopped_duration(id).has_value());
  EXPECT_DOUBLE_EQ(tracker.get_stopped_duration(id).value().seconds(), 3.0);

  update_with_object(tracker, make_time(5'000'000'000LL), stopped_object);
  ASSERT_TRUE(tracker.get_stopped_duration(id).has_value());
  EXPECT_DOUBLE_EQ(tracker.get_stopped_duration(id).value().seconds(), 0.0);
}

TEST(EgoStopTracker, ReportsContinuousStopDuration)
{
  StopTrackingParams params;
  params.history_timeout = 10.0;
  EgoStopTracker tracker(params);

  tracker.update(make_odometry(make_time(10'000'000'000LL), 0.0));
  tracker.update(make_odometry(make_time(12'999'999'999LL), 0.0));
  const auto duration_before_three_seconds = tracker.get_stopped_duration();
  ASSERT_TRUE(duration_before_three_seconds.has_value());
  EXPECT_NEAR(duration_before_three_seconds.value().seconds(), 2.999999999, 1e-9);

  tracker.update(make_odometry(make_time(13'000'000'000LL), 0.0));
  const auto duration_at_three_seconds = tracker.get_stopped_duration();
  ASSERT_TRUE(duration_at_three_seconds.has_value());
  EXPECT_DOUBLE_EQ(duration_at_three_seconds.value().seconds(), 3.0);
}

TEST(EgoStopTracker, MovingEgoRemovesAndResetsStopHistory)
{
  StopTrackingParams params;
  params.stopped_velocity_threshold = 0.1;
  params.history_timeout = 10.0;
  EgoStopTracker tracker(params);

  tracker.update(make_odometry(make_time(10'000'000'000LL), 0.0));
  tracker.update(make_odometry(make_time(13'000'000'000LL), 0.0));
  ASSERT_TRUE(tracker.get_stopped_duration().has_value());
  EXPECT_DOUBLE_EQ(tracker.get_stopped_duration().value().seconds(), 3.0);

  tracker.update(make_odometry(make_time(14'000'000'000LL), 0.1));
  EXPECT_FALSE(tracker.get_stopped_duration().has_value());

  tracker.update(make_odometry(make_time(15'000'000'000LL), 0.0));
  tracker.update(make_odometry(make_time(18'000'000'000LL), 0.0));
  ASSERT_TRUE(tracker.get_stopped_duration().has_value());
  EXPECT_DOUBLE_EQ(tracker.get_stopped_duration().value().seconds(), 3.0);
}

TEST(EgoStopTracker, ResetsStopHistoryAfterHistoryTimeout)
{
  StopTrackingParams params;
  params.history_timeout = 1.0;
  EgoStopTracker tracker(params);

  tracker.update(make_odometry(make_time(10'000'000'000LL), 0.0));
  tracker.update(make_odometry(make_time(11'000'000'000LL), 0.0));
  ASSERT_TRUE(tracker.get_stopped_duration().has_value());
  EXPECT_DOUBLE_EQ(tracker.get_stopped_duration().value().seconds(), 1.0);

  tracker.update(make_odometry(make_time(12'000'000'001LL), 0.0));
  ASSERT_TRUE(tracker.get_stopped_duration().has_value());
  EXPECT_DOUBLE_EQ(tracker.get_stopped_duration().value().seconds(), 0.0);
}

TEST(EgoStopTracker, UsesPlanarSpeedForStopClassification)
{
  StopTrackingParams params;
  params.stopped_velocity_threshold = 0.1;
  EgoStopTracker tracker(params);

  tracker.update(make_odometry(make_time(10'000'000'000LL), 0.06, 0.06));
  EXPECT_TRUE(tracker.get_stopped_duration().has_value());

  tracker.update(make_odometry(make_time(11'000'000'000LL), 0.08, 0.08));
  EXPECT_FALSE(tracker.get_stopped_duration().has_value());
}

TEST(EgoStopTracker, ResetsHistoryWhenObservationTimeMovesBackward)
{
  StopTrackingParams params;
  params.history_timeout = 10.0;
  EgoStopTracker tracker(params);

  tracker.update(make_odometry(make_time(10'000'000'000LL), 0.0));
  tracker.update(make_odometry(make_time(13'000'000'000LL), 0.0));
  ASSERT_TRUE(tracker.get_stopped_duration().has_value());
  EXPECT_DOUBLE_EQ(tracker.get_stopped_duration().value().seconds(), 3.0);

  tracker.update(make_odometry(make_time(5'000'000'000LL), 0.0));
  ASSERT_TRUE(tracker.get_stopped_duration().has_value());
  EXPECT_DOUBLE_EQ(tracker.get_stopped_duration().value().seconds(), 0.0);
}

TEST(StopTrackers, KeepUpdateTimesIndependent)
{
  StopTrackingParams params;
  params.history_timeout = 100.0;
  ObjectStopTracker object_tracker(params);
  EgoStopTracker ego_tracker(params);
  const auto object_id = make_uuid(6);

  update_with_object(object_tracker, make_time(10'000'000'000LL), make_object(object_id, 0.0));
  update_with_object(object_tracker, make_time(13'000'000'000LL), make_object(object_id, 0.0));

  ego_tracker.update(make_odometry(make_time(100'000'000'000LL), 0.0));
  ego_tracker.update(make_odometry(make_time(102'000'000'000LL), 0.0));

  ASSERT_TRUE(object_tracker.get_stopped_duration(object_id).has_value());
  EXPECT_DOUBLE_EQ(object_tracker.get_stopped_duration(object_id).value().seconds(), 3.0);
  ASSERT_TRUE(ego_tracker.get_stopped_duration().has_value());
  EXPECT_DOUBLE_EQ(ego_tracker.get_stopped_duration().value().seconds(), 2.0);
}
}  // namespace autoware::trajectory_validator::plugin::safety
