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

#include "autoware/tensorrt_e2e/providers/map_tensors.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace autoware::tensorrt_e2e
{
namespace
{

Eigen::Matrix4d planar(const double x, const double y, const double yaw)
{
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m(0, 0) = std::cos(yaw);
  m(0, 1) = -std::sin(yaw);
  m(1, 0) = std::sin(yaw);
  m(1, 1) = std::cos(yaw);
  m(0, 3) = x;
  m(1, 3) = y;
  return m;
}

//! The cloud pose, and the planning pose 1.1 m ahead of it and yawed 0.2 rad (10 m/s, L ~ 0.11 s).
EgoFrame frame(const Eigen::Matrix4d & cloud, const Eigen::Matrix4d & planning)
{
  EgoFrame ego;
  ego.sensor_to_map = cloud;
  ego.map_to_sensor = cloud.inverse();
  ego.ego_to_map = planning;
  ego.map_to_ego = planning.inverse();
  return ego;
}

//! Two elements of three points, [x, y, type0, type1]; the second element is padding.
std::vector<float> cloud_tensor()
{
  return {
    5.0f, 1.0f, 1.0f, 0.0f,  8.0f, 1.5f, 1.0f, 0.0f,  11.0f, 2.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 0.0f, 0.0f,
  };
}

}  // namespace

TEST(MapTensors, CloudStampLeavesTheTensorBitIdentical)
{
  const Eigen::Matrix4d pose = planar(12.3, -4.5, 0.7);
  const auto ego = frame(pose, pose);
  const auto data = cloud_tensor();
  EXPECT_EQ(map_tensors::in_planning_frame(data, ego, 4), data);
}

TEST(MapTensors, PlanningTimeMovesPresentPointsIntoThePlanningFrame)
{
  const Eigen::Matrix4d cloud = planar(100.0, 50.0, 0.3);
  const Eigen::Matrix4d planning = cloud * planar(1.1, 0.0, 0.2);
  const auto ego = frame(cloud, planning);
  const auto data = cloud_tensor();
  const auto moved = map_tensors::in_planning_frame(data, ego, 4);
  ASSERT_EQ(moved.size(), data.size());
  const Eigen::Matrix4d cloud_to_planning = planning.inverse() * cloud;
  for (size_t base = 0; base < data.size(); base += 4) {
    const bool present = data[base] != 0.0f || data[base + 1] != 0.0f || data[base + 2] != 0.0f ||
                         data[base + 3] != 0.0f;
    if (!present) {
      for (size_t k = 0; k < 4; ++k) EXPECT_EQ(moved[base + k], 0.0f);  // padding stays zero
      continue;
    }
    const Eigen::Vector4d expected =
      cloud_to_planning * Eigen::Vector4d(data[base], data[base + 1], 0.0, 1.0);
    EXPECT_NEAR(moved[base], expected.x(), 1e-5);
    EXPECT_NEAR(moved[base + 1], expected.y(), 1e-5);
    EXPECT_EQ(moved[base + 2], data[base + 2]);  // type columns untouched
    EXPECT_EQ(moved[base + 3], data[base + 3]);
  }
  // A point 1.1 m ahead of the cloud pose sits at the planning origin.
  std::vector<float> at_planning = {1.1f, 0.0f, 1.0f, 0.0f};
  const auto origin = map_tensors::in_planning_frame(at_planning, ego, 4);
  EXPECT_NEAR(origin[0], 0.0, 1e-6);
  EXPECT_NEAR(origin[1], 0.0, 1e-6);
}

}  // namespace autoware::tensorrt_e2e
