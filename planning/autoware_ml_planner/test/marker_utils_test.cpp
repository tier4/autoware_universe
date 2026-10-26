// Copyright 2025 TIER IV, Inc.
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

#include "autoware/ml_planner/utils/marker_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace autoware::ml_planner::test
{

using autoware::ml_planner::utils::create_lane_marker;
using autoware::ml_planner::utils::find_link_mesh;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::MarkerArray;

TEST(MarkerUtilsTest, CreateLaneMarkerBasic)
{
  // Lane vector: 1 segment, 2 points, 8 dims (X, Y, LB_X, LB_Y, RB_X, RB_Y, ...), minimal
  xt::xarray<float> lane_vector = {1.0f, 2.0f, 0.5f, 0.5f, -0.5f, -0.5f, 0.0f, 0.0f,
                                   2.0f, 3.0f, 0.5f, 0.5f, -0.5f, -0.5f, 0.0f, 0.0f};
  std::vector<int64_t> shape = {1, 1, 2, 8};  // batch, ?, points, dims
  rclcpp::Time stamp(123456, 789, RCL_ROS_TIME);
  rclcpp::Duration lifetime(1, 0);
  Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
  auto marker_array = create_lane_marker(identity, lane_vector, shape, stamp, lifetime);

  // Should create at least 1 marker for the centerline, and possibly for bounds/spheres
  EXPECT_GE(marker_array.markers.size(), 1u);

  for (const auto & marker : marker_array.markers) {
    EXPECT_EQ(marker.header.stamp, stamp);
    EXPECT_EQ(marker.header.frame_id, "base_link");
    EXPECT_GT(marker.points.size(), 0u);
    EXPECT_GT(marker.color.a, 0.0f);
  }
}

TEST(MarkerUtilsTest, FindLinkMeshReadsTheVehicleBody)
{
  const std::string urdf = R"(<robot name="vehicle">
  <link name="base_link">
    <visual>
      <geometry><mesh filename="package://my_vehicle_description/mesh/body.dae" scale="1 1 1"/></geometry>
    </visual>
  </link>
  <link name="sensor_kit_base_link">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
  </link>
  <joint name="sensor_kit_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_kit_base_link"/>
  </joint>
</robot>)";
  EXPECT_EQ(find_link_mesh(urdf, "base_link"), "package://my_vehicle_description/mesh/body.dae");
  EXPECT_EQ(find_link_mesh(urdf, "sensor_kit_base_link"), "");  // a box, not a mesh
  EXPECT_EQ(find_link_mesh(urdf, "missing_link"), "");
  EXPECT_EQ(find_link_mesh("not a urdf", "base_link"), "");
}

}  // namespace autoware::ml_planner::test
