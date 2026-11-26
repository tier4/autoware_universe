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
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace
{
using autoware::trajectory_modifier::utils::obstacle_stop::ObstacleTracker;
using autoware::trajectory_modifier::utils::obstacle_stop::TargetObjects;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_perception_msgs::msg::Shape;

rclcpp::Time time_ms(const int64_t milliseconds)
{
  return rclcpp::Time(milliseconds * 1000000, RCL_ROS_TIME);
}

// `now` is the module clock (arrival, activation, erase). `stamp` is the perception time used
// only to extrapolate a stored measurement during association.
void update(
  ObstacleTracker & tracker, const PredictedObjects & objects, TargetObjects & output,
  const rclcpp::Time & stamp, const rclcpp::Time & now)
{
  output.clear();
  tracker.update_objects(objects, output, stamp, now);
}

PredictedObject make_car(const double x, const double velocity)
{
  PredictedObject object;
  auto & pose = object.kinematics.initial_pose_with_covariance.pose;
  pose.position.x = x;
  pose.orientation.w = 1.0;
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

PredictedObjects make_objects(const PredictedObject & object)
{
  PredictedObjects objects;
  objects.objects.push_back(object);
  return objects;
}

PredictedObjects make_objects(const double x, const double velocity)
{
  return make_objects(make_car(x, velocity));
}

PredictedPath make_path(const std::vector<double> & xs, const double time_step)
{
  PredictedPath path;
  path.confidence = 1.0;
  path.time_step = rclcpp::Duration::from_seconds(time_step);
  for (const double x : xs) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.orientation.w = 1.0;
    path.path.push_back(pose);
  }
  return path;
}

ObstacleTracker make_tracker(
  const double on_time = 0.5, const double off_time = 0.7, const double grace = 0.2)
{
  return ObstacleTracker(on_time, off_time, 1.0, 0.1745, 1.0, grace, 0.3);
}

void confirm_stopped_car(ObstacleTracker & tracker, TargetObjects & output)
{
  const auto stopped = make_objects(0.0, 0.0);
  for (int64_t ms = 0; ms <= 500; ms += 100) {
    update(tracker, stopped, output, time_ms(ms), time_ms(ms));
  }
}
}  // namespace

TEST(ObstacleTracker, SteadyTenHzConfirmsAfterOnTimeBuffer)
{
  auto tracker = make_tracker();
  TargetObjects output;
  const auto stopped = make_objects(0.0, 0.0);

  for (int64_t ms = 0; ms <= 400; ms += 100) {
    update(tracker, stopped, output, time_ms(ms), time_ms(ms));
    EXPECT_TRUE(output.empty());
  }
  update(tracker, stopped, output, time_ms(500), time_ms(500));
  EXPECT_EQ(output.size(), 1U);
}

TEST(ObstacleTracker, UnmatchedInactiveTrackDiesAfterGracePeriod)
{
  auto tracker = make_tracker();
  TargetObjects output;
  const auto car = make_objects(0.0, 0.0);
  const auto none = PredictedObjects{};

  // Aging uses a strict `>` and runs after association. An unmatched idle of exactly
  // grace_period_ keeps the unconfirmed track, so sightings spaced by 200 ms still
  // share first_arrival and confirm once the span reaches on_time_buffer_.
  // The empty cycles keep the previous perception stamp. They advance the module clock
  // without becoming a new observation.
  update(tracker, car, output, time_ms(0), time_ms(0));
  update(tracker, none, output, time_ms(0), time_ms(200));
  update(tracker, car, output, time_ms(200), time_ms(200));
  update(tracker, car, output, time_ms(400), time_ms(400));
  EXPECT_TRUE(output.empty());
  update(tracker, car, output, time_ms(600), time_ms(600));
  EXPECT_EQ(output.size(), 1U);

  // 300 ms is past grace_period_, so the empty update erases the track. The sighting
  // at 300 ms starts a new clock, and the unmatched 400 ms gap to 700 ms erases that one too.
  auto restarted = make_tracker();
  update(restarted, car, output, time_ms(0), time_ms(0));
  update(restarted, none, output, time_ms(0), time_ms(300));
  update(restarted, car, output, time_ms(300), time_ms(300));
  update(restarted, none, output, time_ms(300), time_ms(700));
  EXPECT_TRUE(output.empty());
}

TEST(ObstacleTracker, UnmatchedActiveTrackDiesAfterOffTimeBuffer)
{
  auto tracker = make_tracker();
  TargetObjects output;
  confirm_stopped_car(tracker, output);
  ASSERT_EQ(output.size(), 1U);

  const auto none = PredictedObjects{};
  update(tracker, none, output, time_ms(1200), time_ms(1200));
  EXPECT_EQ(output.size(), 1U);

  update(tracker, none, output, time_ms(1300), time_ms(1300));
  EXPECT_TRUE(output.empty());
}

TEST(ObstacleTracker, MovingObjectKeepsItsIdentity)
{
  auto tracker = make_tracker();
  TargetObjects output;
  constexpr double velocity = 5.0;
  for (int64_t ms = 0; ms <= 1000; ms += 100) {
    const double x = velocity * static_cast<double>(ms) / 1000.0;
    update(tracker, make_objects(x, velocity), output, time_ms(ms), time_ms(ms));
  }

  ASSERT_EQ(output.size(), 1U);
  EXPECT_NEAR(
    output.front().object.kinematics.initial_pose_with_covariance.pose.position.x, 5.0, 1e-6);
}

TEST(ObstacleTracker, ReplayedMessageDoesNotConfirm)
{
  auto tracker = make_tracker();
  TargetObjects output;
  const auto car = make_objects(0.0, 0.0);
  const auto stamp = time_ms(0);

  update(tracker, car, output, stamp, time_ms(0));
  update(tracker, car, output, stamp, time_ms(600));
  EXPECT_TRUE(output.empty());

  // An already active track is not refreshed by the same stamp, and it is still emitted.
  auto confirmed = make_tracker();
  confirm_stopped_car(confirmed, output);
  ASSERT_EQ(output.size(), 1U);
  update(confirmed, car, output, time_ms(500), time_ms(600));
  EXPECT_EQ(output.size(), 1U);
}

TEST(ObstacleTracker, ReplayedMessageKeepsUnconfirmedTrack)
{
  auto tracker = make_tracker();
  TargetObjects output;
  const auto car = make_objects(0.0, 0.0);
  const auto stamp = time_ms(0);

  // A repeated stamp is not a miss and does not refresh arrival time. Idle time stays
  // under off_time_buffer_, so the unconfirmed track survives. The next stamp is inside
  // that buffer, and the span from first_arrival reaches on_time_buffer_, so it confirms.
  update(tracker, car, output, stamp, time_ms(0));
  update(tracker, car, output, stamp, time_ms(250));
  EXPECT_TRUE(output.empty());
  update(tracker, car, output, time_ms(500), time_ms(500));
  EXPECT_EQ(output.size(), 1U);
}

TEST(ObstacleTracker, AssociatesAcrossOneSecondStampGap)
{
  // off_time_buffer_ is 1.5 s, so the 1 s perception gap still matches this active track.
  // The message arrives 1.7 s after last_arrival, past that buffer. The emitted pose is the
  // stored measurement moved forward by now - stamp (0.7 s).
  auto tracker = make_tracker(0.5, 1.5);
  TargetObjects output;
  constexpr double velocity = 10.0;

  for (int64_t ms = 0; ms <= 500; ms += 100) {
    const double x = velocity * static_cast<double>(ms) / 1000.0;
    update(tracker, make_objects(x, velocity), output, time_ms(ms), time_ms(ms));
  }
  ASSERT_EQ(output.size(), 1U);

  update(tracker, make_objects(15.0, velocity), output, time_ms(1500), time_ms(2200));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_NEAR(
    output.front().object.kinematics.initial_pose_with_covariance.pose.position.x, 22.0, 1e-6);
}

TEST(ObstacleTracker, RejectsMatchWhenStampGapExceedsBuffer)
{
  auto tracker = make_tracker();
  TargetObjects output;
  const auto car = make_objects(0.0, 0.0);

  // 0.5 s is inside off_time_buffer_, so the unconfirmed track matches. The span from
  // first_arrival reaches on_time_buffer_, so it confirms.
  update(tracker, car, output, time_ms(0), time_ms(0));
  update(tracker, car, output, time_ms(500), time_ms(500));
  EXPECT_EQ(output.size(), 1U);

  // 0.8 s is past off_time_buffer_ for an unconfirmed track as well. The old track is not
  // matched, and the new sighting has not been present long enough to confirm.
  auto unconfirmed = make_tracker();
  update(unconfirmed, car, output, time_ms(0), time_ms(0));
  update(unconfirmed, car, output, time_ms(800), time_ms(800));
  EXPECT_TRUE(output.empty());

  // 0.8 s from the last confirmation is past off_time_buffer_.
  auto confirmed = make_tracker();
  confirm_stopped_car(confirmed, output);
  ASSERT_EQ(output.size(), 1U);
  update(confirmed, car, output, time_ms(1300), time_ms(1300));
  EXPECT_TRUE(output.empty());
}

TEST(ObstacleTracker, TenHzWithLatencyStillConfirms)
{
  auto tracker = make_tracker();
  TargetObjects output;
  const auto car = make_objects(0.0, 0.0);
  constexpr int64_t latency_ms = 150;

  for (int64_t stamp_ms = 0; stamp_ms <= 500; stamp_ms += 100) {
    update(tracker, car, output, time_ms(stamp_ms), time_ms(stamp_ms + latency_ms));
  }
  EXPECT_EQ(output.size(), 1U);
}

TEST(ObstacleTracker, FiveHzBoundaryStillConfirms)
{
  auto tracker = make_tracker();
  TargetObjects output;
  const auto car = make_objects(0.0, 0.0);

  for (int64_t stamp_ms = 0; stamp_ms <= 600; stamp_ms += 200) {
    update(tracker, car, output, time_ms(stamp_ms), time_ms(stamp_ms));
  }
  EXPECT_EQ(output.size(), 1U);
}

TEST(ObstacleTracker, PropagatesOutputToModuleTime)
{
  auto tracker = make_tracker(0.5, 2.0, 0.2);
  TargetObjects output;
  confirm_stopped_car(tracker, output);

  // Confirmation already consumed stamp 500, so the measured object needs a newer stamp.
  auto braking = make_car(0.0, 10.0);
  braking.kinematics.predicted_paths.push_back(make_path({0.0, 4.0, 6.0}, 0.5));
  update(tracker, make_objects(braking), output, time_ms(600), time_ms(600));
  update(tracker, PredictedObjects{}, output, time_ms(600), time_ms(1100));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_NEAR(
    output.front().object.kinematics.initial_pose_with_covariance.pose.position.x, 4.0, 1e-3);

  // dt = 0.25 s falls between the 0.5 s samples. The next path point is the pose at 0.75 s.
  auto off_grid = make_tracker(0.5, 2.0, 0.2);
  confirm_stopped_car(off_grid, output);
  auto mid = make_car(0.0, 10.0);
  mid.kinematics.predicted_paths.push_back(make_path({0.0, 4.0, 6.0}, 0.5));
  update(off_grid, make_objects(mid), output, time_ms(600), time_ms(600));
  update(off_grid, PredictedObjects{}, output, time_ms(600), time_ms(850));
  ASSERT_EQ(output.size(), 1U);
  const auto & mid_object = output.front().object;
  EXPECT_NEAR(mid_object.kinematics.initial_pose_with_covariance.pose.position.x, 2.0, 1e-3);
  ASSERT_EQ(mid_object.kinematics.predicted_paths.size(), 1U);
  const auto & mid_path = mid_object.kinematics.predicted_paths.front();
  ASSERT_EQ(mid_path.path.size(), 2U);
  EXPECT_NEAR(mid_path.path[1].position.x, 5.0, 1e-3);
  EXPECT_DOUBLE_EQ(rclcpp::Duration(mid_path.time_step).seconds(), 0.5);

  auto coasting = make_tracker(0.5, 2.0, 0.2);
  confirm_stopped_car(coasting, output);
  update(coasting, make_objects(0.0, 10.0), output, time_ms(600), time_ms(600));
  update(coasting, PredictedObjects{}, output, time_ms(600), time_ms(800));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_NEAR(
    output.front().object.kinematics.initial_pose_with_covariance.pose.position.x, 2.0, 1e-3);

  auto held = make_tracker(0.5, 2.0, 0.2);
  confirm_stopped_car(held, output);
  auto short_path = make_car(0.0, 10.0);
  short_path.kinematics.predicted_paths.push_back(make_path({0.0, 4.0}, 0.3));
  update(held, make_objects(short_path), output, time_ms(600), time_ms(600));
  update(held, PredictedObjects{}, output, time_ms(600), time_ms(1200));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_NEAR(
    output.front().object.kinematics.initial_pose_with_covariance.pose.position.x, 4.0, 1e-3);

  // The stored car is already at x=5; a module time before its stamp must not move it.
  auto clamped = make_tracker(0.5, 2.0, 0.2);
  for (int64_t ms = 0; ms <= 500; ms += 100) {
    update(clamped, make_objects(5.0, 10.0), output, time_ms(ms), time_ms(ms));
  }
  update(clamped, PredictedObjects{}, output, time_ms(500), time_ms(400));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_NEAR(
    output.front().object.kinematics.initial_pose_with_covariance.pose.position.x, 5.0, 1e-3);

  // A near-zero velocity keeps the measured pose even when its predicted path drifts.
  auto parked = make_tracker(0.5, 2.0, 0.2);
  confirm_stopped_car(parked, output);
  auto slow = make_car(0.0, 0.2);
  slow.kinematics.predicted_paths.push_back(make_path({0.0, 4.0, 6.0}, 0.5));
  update(parked, make_objects(slow), output, time_ms(600), time_ms(600));
  update(parked, PredictedObjects{}, output, time_ms(600), time_ms(1100));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_NEAR(
    output.front().object.kinematics.initial_pose_with_covariance.pose.position.x, 0.0, 1e-3);
}
