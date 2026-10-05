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

#include <Eigen/Dense>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace autoware::ml_planner::test
{
namespace
{
Eigen::Matrix4d pose_at_x(const double x)
{
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose(0, 3) = x;
  return pose;
}
}  // namespace

TEST(VirtualPoseTest, ResetLimitsAreSplitAlongAndAcrossTheVirtualHeading)
{
  // Previous trajectory along x, every 0.5 m from 0 to 10 m.
  std::vector<Eigen::Matrix4d> polyline;
  for (int i = 0; i <= 20; ++i) {
    polyline.push_back(pose_at_x(0.5 * i));
  }
  utils::VirtualPoseParams params{};
  params.enable = true;
  params.max_longitudinal_error_m = 0.5;
  params.max_lateral_error_m = 0.3;
  params.max_yaw_error_deg = 5.0;
  // Search the first 4 segments only (x = 0 to 2 m), so a vehicle past them is ahead of the
  // virtual pose.
  params.max_search_segment_count = 4;
  params.yaw_fit_half_window_m = 1.0;
  params.yaw_fit_min_length_m = 0.2;
  params.history_prefix_count = 0;
  params.reference = "optimized";
  const auto vehicle_at = [](const double x, const double y) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation.w = 1.0;
    return pose;
  };

  // Vehicle 0.4 m ahead of the virtual pose: within the longitudinal limit; 0.6 m: beyond it.
  EXPECT_FALSE(utils::compute_virtual_pose(vehicle_at(2.4, 0.0), polyline, 0, params).reset);
  EXPECT_TRUE(utils::compute_virtual_pose(vehicle_at(2.6, 0.0), polyline, 0, params).reset);
  // 0.25 m beside the trajectory: within the lateral limit; 0.35 m: beyond it.
  EXPECT_FALSE(utils::compute_virtual_pose(vehicle_at(1.0, 0.25), polyline, 0, params).reset);
  EXPECT_TRUE(utils::compute_virtual_pose(vehicle_at(1.0, 0.35), polyline, 0, params).reset);
}

}  // namespace autoware::ml_planner::test
