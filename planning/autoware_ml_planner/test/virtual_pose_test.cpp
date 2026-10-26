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

#include "autoware/ml_planner/utils/virtual_pose.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/time.hpp>

#include <gtest/gtest.h>

namespace autoware::ml_planner::utils
{
namespace
{
using nav_msgs::msg::Odometry;
using Trajectory = autoware_planning_msgs::msg::Trajectory;

constexpr double HISTORY_WINDOW_S = 2.5;

Odometry make_odometry(
  const double t, const double x, const double y, const double yaw = 0.0, const double v = 5.0)
{
  Odometry odometry;
  odometry.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.x = x;
  odometry.pose.pose.position.y = y;
  odometry.pose.pose.position.z = 1.5;
  odometry.pose.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
  odometry.twist.twist.linear.x = v;
  return odometry;
}

// Straight along x from x0, a point every 0.5 m.
Trajectory make_straight_trajectory(const double x0)
{
  Trajectory trajectory;
  for (int i = 1; i <= 40; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = x0 + 0.5 * i;
    point.pose.orientation.w = 1.0;
    trajectory.points.push_back(point);
  }
  return trajectory;
}

double yaw_of(const geometry_msgs::msg::Pose & pose)
{
  return autoware_utils_geometry::get_rpy(pose.orientation).z;
}

class VirtualPoseTest : public ::testing::Test
{
protected:
  // Measured odometry of a new cycle; returns the planning start.
  const Odometry & step(const Odometry & measured)
  {
    measured_.push_back(measured);
    virtual_pose_.update(measured_, params_);
    return virtual_pose_.history().back();
  }

  VirtualPoseParams params_{true};
  TimedBuffer<Odometry> measured_{
    HISTORY_WINDOW_S, [](const Odometry & msg) { return rclcpp::Time(msg.header.stamp); }};
  VirtualPose virtual_pose_{HISTORY_WINDOW_S};
};
}  // namespace

TEST_F(VirtualPoseTest, StartsOnTheVehicleWithoutAPublishedTrajectory)
{
  const auto & start = step(make_odometry(0.0, 0.0, 0.4));
  EXPECT_DOUBLE_EQ(start.pose.pose.position.y, 0.4);
  EXPECT_TRUE(virtual_pose_.take_restart());
  EXPECT_FALSE(virtual_pose_.take_restart());
}

TEST_F(VirtualPoseTest, PlansFromTheClosestPointOfThePublishedTrajectory)
{
  step(make_odometry(0.0, 0.0, 0.0));
  virtual_pose_.set_published_trajectory(make_straight_trajectory(0.0));
  virtual_pose_.take_restart();

  // The vehicle fell 0.3 m to the side and turned 5 deg: the planning start stays on the plan.
  const auto & start = step(make_odometry(0.1, 0.5, 0.3, 0.087));
  EXPECT_NEAR(start.pose.pose.position.x, 0.5, 1e-3);
  EXPECT_NEAR(start.pose.pose.position.y, 0.0, 1e-3);
  EXPECT_NEAR(start.pose.pose.position.z, 1.5, 1e-9);
  EXPECT_NEAR(yaw_of(start.pose.pose), 0.0, 1e-3);
  EXPECT_DOUBLE_EQ(start.twist.twist.linear.x, 5.0);
  EXPECT_FALSE(virtual_pose_.take_restart());
  // The model's past ends at the planning start.
  EXPECT_EQ(virtual_pose_.history().size(), 2U);
}

TEST_F(VirtualPoseTest, KeepsUpWithTheVehicleOnceThePastIsLong)
{
  // Each cycle a plan is published from the planning start; the vehicle drives 0.5 m per cycle,
  // 0.3 m beside the plan after the first one. Once the earlier planning starts fill the prefix
  // of the reference, the planning start must still follow the vehicle along the plan.
  for (int k = 0; k < 30; ++k) {
    const double x = 0.5 * k;
    const auto & start = step(make_odometry(0.1 * k, x, k == 0 ? 0.0 : 0.3));
    EXPECT_NEAR(start.pose.pose.position.x, x, 1e-3) << "cycle " << k;
    EXPECT_NEAR(start.pose.pose.position.y, 0.0, 1e-3) << "cycle " << k;
    virtual_pose_.set_published_trajectory(make_straight_trajectory(start.pose.pose.position.x));
  }
}

TEST_F(VirtualPoseTest, ResetsOntoTheVehicleBeyondTheLimits)
{
  step(make_odometry(0.0, 0.0, 0.0));
  virtual_pose_.set_published_trajectory(make_straight_trajectory(0.0));
  virtual_pose_.take_restart();

  const auto & lateral = step(make_odometry(0.1, 0.5, 3.5));
  EXPECT_DOUBLE_EQ(lateral.pose.pose.position.y, 3.5);
  EXPECT_TRUE(virtual_pose_.take_restart());

  // The past restarted from the measured poses, and the trajectory left is not snapped onto
  // again: the next cycle without a new one stays on the vehicle.
  const auto & next = step(make_odometry(0.2, 1.0, 0.5));
  EXPECT_DOUBLE_EQ(next.pose.pose.position.y, 0.5);
  EXPECT_FALSE(virtual_pose_.take_restart());

  virtual_pose_.set_published_trajectory(make_straight_trajectory(1.0));
  const auto & heading = step(make_odometry(0.3, 1.5, 0.0, 0.5));
  EXPECT_DOUBLE_EQ(yaw_of(heading.pose.pose), 0.5);
  EXPECT_TRUE(virtual_pose_.take_restart());
}

TEST_F(VirtualPoseTest, HoldsThePlanningStartAtStandstill)
{
  step(make_odometry(0.0, 0.0, 0.0, 0.0, 0.0));
  virtual_pose_.set_published_trajectory(make_straight_trajectory(0.0));
  const auto held = step(make_odometry(0.1, 0.0, 0.1, 0.0, 0.0)).pose.pose;

  // A new plan beside the old one and the localization drifting by a few centimetres: the
  // planning start does not move.
  Trajectory shifted = make_straight_trajectory(0.0);
  for (auto & point : shifted.points) {
    point.pose.position.y = 0.05;
  }
  virtual_pose_.set_published_trajectory(shifted);
  const auto & still = step(make_odometry(0.2, 0.05, 0.12, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(still.pose.pose.position.x, held.position.x);
  EXPECT_DOUBLE_EQ(still.pose.pose.position.y, held.position.y);

  // Moved beyond the tolerance: the hold ends.
  const auto & moved = step(make_odometry(0.3, 0.5, 0.1, 0.0, 0.0));
  EXPECT_NEAR(moved.pose.pose.position.y, 0.05, 1e-2);
}

TEST_F(VirtualPoseTest, ResetClearsThePublishedTrajectory)
{
  step(make_odometry(0.0, 0.0, 0.0));
  virtual_pose_.set_published_trajectory(make_straight_trajectory(0.0));
  virtual_pose_.take_restart();

  virtual_pose_.reset();
  EXPECT_TRUE(virtual_pose_.take_restart());
  const auto & start = step(make_odometry(0.1, 0.5, 0.3));
  EXPECT_DOUBLE_EQ(start.pose.pose.position.y, 0.3);
}

TEST(TimedBufferTest, PushOrReplaceBackReplacesTheNewestEntryWithTheSameStamp)
{
  TimedBuffer<Odometry> buffer{
    HISTORY_WINDOW_S, [](const Odometry & msg) { return rclcpp::Time(msg.header.stamp); }};
  buffer.push_back(make_odometry(0.0, 0.0, 0.0));
  buffer.push_or_replace_back(make_odometry(0.0, 1.0, 0.0));
  ASSERT_EQ(buffer.msgs().size(), 1U);
  EXPECT_DOUBLE_EQ(buffer.back().pose.pose.position.x, 1.0);
  buffer.push_or_replace_back(make_odometry(0.1, 2.0, 0.0));
  EXPECT_EQ(buffer.msgs().size(), 2U);
}

}  // namespace autoware::ml_planner::utils
