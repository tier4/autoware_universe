// Copyright 2020 Tier IV, Inc.
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

#include "surround_obstacle_checker/node.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

namespace
{
geometry_msgs::msg::Pose makePose(const double x, const double y)
{
  geometry_msgs::msg::Pose pose;
  pose.position = tier4_autoware_utils::createPoint(x, y, 0.0);
  pose.orientation = tier4_autoware_utils::createQuaternionFromRPY(0.0, 0.0, 0.0);
  return pose;
}

TrajectoryPoint makeTrajPoint(const double x, const double y)
{
  TrajectoryPoint p;
  p.pose = makePose(x, y);
  return p;
}

TrajectoryPoints traj_line()
{
  return {
    makeTrajPoint(0.0, 0.0), makeTrajPoint(1.0, 1.0), makeTrajPoint(2.0, 2.0),
    makeTrajPoint(3.0, 3.0), makeTrajPoint(4.0, 4.0),
  };
}

rclcpp::NodeOptions makeNodeOptions()
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("wheel_radius", 0.5);
  options.append_parameter_override("wheel_width", 0.2);
  options.append_parameter_override("wheel_base", 3.0);
  options.append_parameter_override("wheel_tread", 2.0);
  options.append_parameter_override("front_overhang", 1.0);
  options.append_parameter_override("rear_overhang", 1.0);
  options.append_parameter_override("left_overhang", 0.5);
  options.append_parameter_override("right_overhang", 0.5);
  options.append_parameter_override("vehicle_height", 1.5);
  return options;
}
}  // namespace

class SurroundObstacleCheckerNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_unique<SurroundObstacleCheckerNode>(makeNodeOptions());
  }

  void TearDown() override { node_.reset(); }

  size_t getFirstClosestIndex(
    const TrajectoryPoints & traj, const geometry_msgs::msg::Pose & pose,
    const double distance_thresh)
  {
    return node_->getFirstClosestIndex(traj, pose, distance_thresh);
  }

  std::unique_ptr<SurroundObstacleCheckerNode> node_;
};

TEST_F(SurroundObstacleCheckerNodeTest, getFirstClosestIndex_01)
{
  const auto traj = traj_line();
  const auto pose = makePose(2.0, 2.0);
  EXPECT_EQ(getFirstClosestIndex(traj, pose, 9.0), 2u);
}

TEST_F(SurroundObstacleCheckerNodeTest, getFirstClosestIndex_02)
{
  const TrajectoryPoints traj;
  const auto pose = makePose(2.0, 2.0);
  EXPECT_EQ(getFirstClosestIndex(traj, pose, 9.0), 0u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
