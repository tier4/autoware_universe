// Copyright 2021 The Autoware Foundation
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

#include "autoware/mpc_lateral_controller/mpc_trajectory.hpp"
#include "autoware/mpc_lateral_controller/mpc_utils.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include <memory>
#include <vector>

namespace
{
namespace MPCUtils = autoware::motion::control::mpc_lateral_controller::MPCUtils;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

TrajectoryPoint makePoint(const double x, const double y, const float vx)
{
  TrajectoryPoint p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.longitudinal_velocity_mps = vx;
  return p;
}

/* cppcheck-suppress syntaxError */
TEST(TestMPC, CalcStopDistance)
{
  constexpr float MOVE = 1.0f;
  constexpr float STOP = 0.0f;

  Trajectory trajectory_msg;
  trajectory_msg.points.push_back(makePoint(0.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(1.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(2.0, 0.0, STOP));  // STOP
  trajectory_msg.points.push_back(makePoint(3.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(4.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(5.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(6.0, 0.0, STOP));  // STOP
  trajectory_msg.points.push_back(makePoint(7.0, 0.0, STOP));  // STOP

  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 0), 2.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 1), 1.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 2), 0.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 3), 3.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 4), 2.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 5), 1.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 6), 0.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 7), -1.0);
}

TEST(TestMPC, ConvertToMPCTrajectoryTemporalUsesTimeFromStart)
{
  Trajectory trajectory_msg;
  auto p0 = makePoint(0.0, 0.0, 2.0f);
  auto p1 = makePoint(1.0, 0.0, 2.0f);
  auto p2 = makePoint(2.0, 0.0, 2.0f);
  p0.time_from_start = rclcpp::Duration::from_seconds(0.0);
  p1.time_from_start = rclcpp::Duration::from_seconds(0.3);
  p2.time_from_start = rclcpp::Duration::from_seconds(0.8);
  trajectory_msg.points = {p0, p1, p2};

  const auto mpc_traj = MPCUtils::convertToMPCTrajectory(trajectory_msg, true);
  ASSERT_EQ(mpc_traj.relative_time.size(), 3UL);
  EXPECT_DOUBLE_EQ(mpc_traj.relative_time.at(0), 0.0);
  EXPECT_DOUBLE_EQ(mpc_traj.relative_time.at(1), 0.3);
  EXPECT_DOUBLE_EQ(mpc_traj.relative_time.at(2), 0.8);
}

}  // namespace
