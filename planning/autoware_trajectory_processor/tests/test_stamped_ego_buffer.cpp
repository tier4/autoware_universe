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

#include "autoware/trajectory_processor/time_sequence_raw/stamped_ego_buffer.hpp"

#include <gtest/gtest.h>

namespace
{
using autoware::trajectory_processor::time_sequence_raw::StampedEgoBuffer;
using nav_msgs::msg::Odometry;

Odometry make_odom(const int32_t sec, const uint32_t nsec, const double x, const double vx)
{
  Odometry odom;
  odom.header.stamp.sec = sec;
  odom.header.stamp.nanosec = nsec;
  odom.pose.pose.orientation.w = 1.0;
  odom.pose.pose.position.x = x;
  odom.twist.twist.linear.x = vx;
  return odom;
}
}  // namespace

TEST(StampedEgoBuffer, InterpolatesBetween40HzSamples)
{
  StampedEgoBuffer buffer;
  buffer.set_duration(1.0);
  buffer.push_odometry(make_odom(10, 0, 0.0, 10.0));
  buffer.push_odometry(make_odom(10, 25000000, 0.25, 10.0));  // +25 ms, +0.25 m at 10 m/s

  const auto looked = buffer.lookup(rclcpp::Time(10, 12500000, RCL_ROS_TIME), 0.15);
  ASSERT_TRUE(looked.has_value());
  EXPECT_TRUE(looked->interpolated);
  EXPECT_FALSE(looked->fallback_latest);
  EXPECT_NEAR(looked->odometry.pose.pose.position.x, 0.125, 1e-6);
  EXPECT_NEAR(looked->live_lag_s, 0.0125, 1e-6);
  EXPECT_NEAR(looked->lookup_dt_s, 0.0, 1e-9);
}

TEST(StampedEgoBuffer, LatestSampleIsAheadOfPlannerStamp)
{
  StampedEgoBuffer buffer;
  buffer.set_duration(1.0);
  buffer.push_odometry(make_odom(10, 0, 0.0, 9.0));
  buffer.push_odometry(make_odom(10, 50000000, 0.45, 9.0));  // 50 ms later, vehicle already closer

  const auto looked = buffer.lookup(rclcpp::Time(10, 0, RCL_ROS_TIME), 0.15);
  ASSERT_TRUE(looked.has_value());
  EXPECT_NEAR(looked->odometry.pose.pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(looked->live_lag_s, 0.05, 1e-6);
  EXPECT_GT(looked->live_lag_s, 0.0);
}
