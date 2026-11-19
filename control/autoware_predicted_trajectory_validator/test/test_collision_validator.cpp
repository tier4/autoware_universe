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

#include "test_utils.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace autoware::predicted_trajectory_validator
{
using namespace test_utils;  // NOLINT(build/namespaces)
using autoware_internal_control_msgs::msg::HazardSector;
using sensor_msgs::msg::PointCloud2;

TEST(CollisionValidatorTest, NothingRaisedGivesAnEmptyArray)
{
  auto validator = make_validator();
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto now = at(10.0);

  auto short_trajectory = make_trajectory(now, pose);
  short_trajectory.points.resize(1);
  for (const auto & trajectory : {make_trajectory(now, pose), short_trajectory}) {
    HazardWarningSources sources;
    validator.validate(
      trajectory, make_odom(now, pose), make_objects(now, {make_car(15.0, 5.0)}),
      test_vehicle_info(), nullptr, sources);
    ASSERT_TRUE(sources.collision.has_value());
    EXPECT_TRUE(sources.collision->empty());
  }
}

struct SkippedCase
{
  std::string name;
  std::function<void(::predicted_trajectory_validator::Params &)> modify_params;
  std::function<void(Trajectory &)> modify_trajectory;
  bool with_objects;
  bool with_cloud;
};

class CollisionSourcesUnsetTest : public ::testing::TestWithParam<SkippedCase>
{
};

TEST_P(CollisionSourcesUnsetTest, CollisionSourcesAreLeftUnset)
{
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto now = at(10.0);
  auto params = default_params();
  GetParam().modify_params(params);
  auto trajectory = make_trajectory(now, pose);
  GetParam().modify_trajectory(trajectory);

  auto validator = make_validator(params);
  HazardWarningSources sources;
  validator.validate(
    trajectory, make_odom(now, pose),
    GetParam().with_objects ? make_objects(now, {make_car(15.0, 0.0)}) : nullptr,
    test_vehicle_info(), GetParam().with_cloud ? make_cloud(now, 15.0) : nullptr, sources);
  EXPECT_FALSE(sources.collision.has_value());
}

INSTANTIATE_TEST_SUITE_P(
  CollisionValidatorTest, CollisionSourcesUnsetTest,
  ::testing::Values(
    SkippedCase{"NoInput", [](auto &) {}, [](auto &) {}, false, false},
    SkippedCase{
      "UnsortedTrajectory", [](auto &) {},
      [](auto & t) { t.points[1].time_from_start = rclcpp::Duration::from_seconds(100.0); }, true,
      false},
    SkippedCase{
      "PointcloudDisabled", [](auto & p) { p.collision_validator.enable_pointcloud = false; },
      [](auto &) {}, false, true},
    SkippedCase{
      "WarningDisabled", [](auto & p) { p.collision_validator.enable_warning = false; },
      [](auto &) {}, true, true}),
  [](const ::testing::TestParamInfo<SkippedCase> & info) { return info.param.name; });

TEST(CollisionValidatorTest, EitherInputAloneIsEvaluated)
{
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto now = at(10.0);
  const auto vehicle_info = test_vehicle_info();

  {
    auto validator = make_validator(default_params());
    HazardWarningSources sources;
    validator.validate(
      make_trajectory(now, pose), make_odom(now, pose), make_objects(now, {make_car(15.0, 0.0)}),
      vehicle_info, nullptr, sources);
    ASSERT_TRUE(sources.collision.has_value());
    EXPECT_EQ(sources.collision->size(), 1U);
    EXPECT_EQ(count_points(*sources.collision), 0U);
  }
  {
    auto validator = make_validator(default_params());
    HazardWarningSources sources;
    validator.validate(
      make_trajectory(now, pose), make_odom(now, pose), nullptr, vehicle_info,
      make_cloud(now, 15.0), sources);
    ASSERT_TRUE(sources.collision.has_value());
    EXPECT_FALSE(sources.collision->empty());
    EXPECT_EQ(count_points(*sources.collision), sources.collision->size());
  }
}

TEST(CollisionValidatorTest, PointcloudWithoutXyzIsSkippedAndObjectsAreStillEvaluated)
{
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto now = at(10.0);
  auto cloud = std::make_shared<PointCloud2>();
  cloud->header.stamp = now;
  cloud->header.frame_id = "base_link";
  sensor_msgs::PointCloud2Modifier modifier(*cloud);
  modifier.setPointCloud2Fields(1, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(1);

  auto validator = make_validator(default_params());
  HazardWarningSources sources;
  EXPECT_NO_THROW(validator.validate(
    make_trajectory(now, pose), make_odom(now, pose), make_objects(now, {make_car(15.0, 0.0)}),
    test_vehicle_info(), cloud, sources));
  ASSERT_TRUE(sources.collision.has_value());
  EXPECT_EQ(sources.collision->size(), 1U);
  EXPECT_EQ(count_points(*sources.collision), 0U);
}

TEST(CollisionValidatorTest, PointcloudWithoutCloseOdometryIsDropped)
{
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto now = at(10.0);
  const auto run = [&](const rclcpp::Time & cloud_stamp) {
    auto validator = make_validator();
    HazardWarningSources sources;
    validator.validate(
      make_trajectory(now, pose), make_odom(now, pose), make_objects(now, {}), test_vehicle_info(),
      make_cloud(cloud_stamp, 15.0), sources);
    return sources;
  };

  const auto close = run(at(9.95));
  ASSERT_TRUE(close.collision.has_value());
  EXPECT_FALSE(close.collision->empty());

  const auto far = run(at(9.8));
  ASSERT_TRUE(far.collision.has_value());
  EXPECT_TRUE(far.collision->empty());
}

TEST(CollisionValidatorTest, NearCandidatesAreHandedOverAsProximityWithoutALevel)
{
  auto validator = make_validator();
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto now = at(10.0);

  HazardWarningSources sources;
  validator.validate(
    make_trajectory(now, pose), make_odom(now, pose),
    make_objects(now, {make_car(1.0, 1.648), make_car(1.0, -1.948)}), test_vehicle_info(), nullptr,
    sources);
  ASSERT_TRUE(sources.collision.has_value());
  ASSERT_EQ(sources.collision->size(), 1U);
  EXPECT_EQ(sources.collision->front().risk_level, HazardSector::NONE);
  EXPECT_EQ(sources.collision->front().sources, HazardSector::SOURCE_PROXIMITY);
  for (const auto & vertex : sources.collision->front().shape.outer()) {
    EXPECT_GT(vertex.y(), 1.0);
  }
}
}  // namespace autoware::predicted_trajectory_validator
