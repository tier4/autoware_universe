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

#include "autoware/boundary_departure_checker/detail/footprints_generator.hpp"
#include "autoware/boundary_departure_checker/detail/type_alias.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace autoware::boundary_departure_checker
{
namespace
{
using autoware_utils_geometry::create_quaternion_from_yaw;
}  // namespace

class FootprintGeneratorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 1. Setup Vehicle Info
    vehicle_info_ = autoware::vehicle_info_utils::createVehicleInfo(
      0.383, 0.235, 2.79, 1.64, 1.0, 1.1, 2.5, 0.128, 0.128, 0.70);

    // 2. Setup Trajectory
    TrajectoryPoint p1;
    p1.pose.position.x = 0.0;
    p1.pose.position.y = 0.0;
    p1.pose.orientation.w = 1.0;
    p1.longitudinal_velocity_mps = 10.0;
    p1.time_from_start.sec = static_cast<int32_t>(0.0);
    p1.time_from_start.nanosec = static_cast<uint32_t>((0.0 - p1.time_from_start.sec) * 1e9);
    pred_traj_.push_back(p1);

    TrajectoryPoint p2 = p1;
    p2.pose.position.x = 1.0;
    p2.time_from_start.sec = static_cast<int32_t>(0.1);
    p2.time_from_start.nanosec = static_cast<uint32_t>((0.1 - p2.time_from_start.sec) * 1e9);
    pred_traj_.push_back(p2);

    // 3. Setup covariance
    pose_with_cov_.pose.orientation.w = 1.0;
    for (auto & c : pose_with_cov_.covariance) {
      c = 0.0;
    }
  }

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  TrajectoryPoints pred_traj_;
  geometry_msgs::msg::PoseWithCovariance pose_with_cov_;
};

TEST_F(FootprintGeneratorTest, TestNormalFootprintGenerator)
{
  // 1-line summary: Verifies that footprints are correctly generated along a trajectory.

  // Act:
  const auto footprints = footprints::generate(pred_traj_, vehicle_info_, pose_with_cov_);

  // Assert:
  ASSERT_EQ(footprints.size(), pred_traj_.size());
  const double expected_x = vehicle_info_.wheel_base_m + vehicle_info_.front_overhang_m;
  EXPECT_DOUBLE_EQ(footprints[0][VehicleInfo::FrontLeftIndex].x(), expected_x);
}

TEST_F(FootprintGeneratorTest, TestSteeringFootprintGeneratorEmptyTrajectory)
{
  // Arrange:
  TrajectoryPoints empty_traj;

  // Act:
  const auto footprints = footprints::generate(empty_traj, vehicle_info_, pose_with_cov_);

  // Assert:
  EXPECT_TRUE(footprints.empty());
}

TEST_F(FootprintGeneratorTest, TestCountPointsWithinDistance)
{
  // 1-line summary: The count covers the leading points inside the arc length from the start.

  // Arrange: the fixture holds two poses that sit 1.0 m apart.

  // Act and assert:
  EXPECT_EQ(footprints::count_points_within_distance(pred_traj_, 2.0).align_count, 2U);
  EXPECT_EQ(footprints::count_points_within_distance(pred_traj_, 0.5).align_count, 1U);
  EXPECT_EQ(footprints::count_points_within_distance(pred_traj_, 0.0).align_count, 0U);
  EXPECT_EQ(footprints::count_points_within_distance(TrajectoryPoints{}, 2.0).align_count, 0U);
}

TEST_F(FootprintGeneratorTest, TestAlignToEgoPoseIsNoOpWhenTrajectoryStartsAtEgo)
{
  // 1-line summary: A trajectory that already starts at the measured pose keeps every pose.

  // Arrange: a correction here would overwrite the heading that the generator chose.
  const auto ego_pose = pred_traj_.front().pose;

  // Act:
  const auto aligned = footprints::align_to_ego_pose(
    pred_traj_, ego_pose, footprints::count_points_within_distance(pred_traj_, 5.71));

  // Assert:
  ASSERT_EQ(aligned.size(), pred_traj_.size());
  for (size_t i = 0; i < aligned.size(); ++i) {
    EXPECT_NEAR(aligned[i].pose.position.x, pred_traj_[i].pose.position.x, 1e-9);
    EXPECT_NEAR(aligned[i].pose.position.y, pred_traj_[i].pose.position.y, 1e-9);
    EXPECT_NEAR(
      tf2::getYaw(aligned[i].pose.orientation), tf2::getYaw(pred_traj_[i].pose.orientation), 1e-9);
  }
}

TEST_F(FootprintGeneratorTest, TestAlignToEgoPoseCorrectsStoppedEgoPoses)
{
  // 1-line summary: A stopped ego moves every stacked trajectory pose onto its measured pose.

  // Arrange: neither pose advances, so both keep the full correction.
  geometry_msgs::msg::Pose ego_pose;
  ego_pose.orientation = create_quaternion_from_yaw(0.0);

  TrajectoryPoints stopped_traj = pred_traj_;
  for (auto & p : stopped_traj) {
    p.pose.position.x = 0.0;
    p.pose.position.y = 0.3;
    p.pose.orientation = create_quaternion_from_yaw(0.1);
    p.longitudinal_velocity_mps = 0.0;
  }

  // Act:
  const auto aligned = footprints::align_to_ego_pose(
    stopped_traj, ego_pose, footprints::count_points_within_distance(stopped_traj, 5.71));

  // Assert:
  ASSERT_EQ(aligned.size(), stopped_traj.size());
  for (const auto & p : aligned) {
    EXPECT_NEAR(p.pose.position.x, 0.0, 1e-9);
    EXPECT_NEAR(p.pose.position.y, 0.0, 1e-9);
    EXPECT_NEAR(tf2::getYaw(p.pose.orientation), 0.0, 1e-9);
  }
}

TEST_F(FootprintGeneratorTest, TestAlignToEgoPoseKeepsGeneratorHeadingChanges)
{
  // 1-line summary: The correction keeps the heading changes that the generator gives.

  // Arrange: the generator turns 0.2 rad over 1.0 m, and ego measures 0.1 rad off at the start.
  constexpr double generator_turn_rad = 0.2;
  constexpr double ego_yaw_offset_rad = 0.1;
  TrajectoryPoints turning_traj = pred_traj_;
  turning_traj[1].pose.orientation = create_quaternion_from_yaw(generator_turn_rad);

  auto ego_pose = turning_traj.front().pose;
  ego_pose.orientation = create_quaternion_from_yaw(ego_yaw_offset_rad);

  // Act: the align distance is 2.0 m, so the second pose keeps half of the correction.
  const auto aligned = footprints::align_to_ego_pose(
    turning_traj, ego_pose, footprints::count_points_within_distance(turning_traj, 2.0));

  // Assert: the turn of the generator survives the correction.
  ASSERT_EQ(aligned.size(), 2U);
  EXPECT_NEAR(tf2::getYaw(aligned[0].pose.orientation), ego_yaw_offset_rad, 1e-9);
  EXPECT_NEAR(
    tf2::getYaw(aligned[1].pose.orientation), generator_turn_rad + 0.5 * ego_yaw_offset_rad, 1e-9);
}

TEST_F(FootprintGeneratorTest, TestAlignToEgoPoseKeepsPosesBeyondTheAlignDistance)
{
  // 1-line summary: A pose past the align distance keeps the value that the generator gives.

  // Arrange: the second pose sits 1.0 m ahead, past the 0.5 m align distance.
  TrajectoryPoints offset_traj = pred_traj_;
  offset_traj[0].pose.position.y = 0.3;
  offset_traj[1].pose.position.y = 0.3;

  geometry_msgs::msg::Pose ego_pose;
  ego_pose.orientation = create_quaternion_from_yaw(0.0);

  // Act:
  const auto aligned = footprints::align_to_ego_pose(
    offset_traj, ego_pose, footprints::count_points_within_distance(offset_traj, 0.5));

  // Assert:
  ASSERT_EQ(aligned.size(), 2U);
  EXPECT_NEAR(aligned[0].pose.position.y, 0.0, 1e-9);
  EXPECT_NEAR(aligned[1].pose.position.y, 0.3, 1e-9);
}

TEST_F(FootprintGeneratorTest, TestAlignToEgoPoseEmptyTrajectory)
{
  // Arrange:
  TrajectoryPoints empty_traj;
  geometry_msgs::msg::Pose ego_pose;
  ego_pose.orientation = create_quaternion_from_yaw(0.0);

  // Act:
  const auto aligned = footprints::align_to_ego_pose(
    empty_traj, ego_pose, footprints::count_points_within_distance(empty_traj, 5.71));

  // Assert:
  EXPECT_TRUE(aligned.empty());
}

TEST_F(FootprintGeneratorTest, TestGetSidesFromFootprintsEmpty)
{
  // Arrange:
  footprints::Footprints empty_footprints;

  // Act:
  const auto sides_array = footprints::get_sides_from_footprints(empty_footprints);

  // Assert:
  EXPECT_TRUE(sides_array.empty());
}

TEST_F(FootprintGeneratorTest, TestGetSidesFromFootprints)
{
  // 1-line summary: Verifies extraction of left and right side segments from footprints.

  // Arrange:
  using autoware::vehicle_info_utils::VehicleInfo;
  const auto base_fp = vehicle_info_.createFootprint(0.0, 0.0);
  footprints::Footprints test_footprints = {base_fp};

  auto offset_fp = base_fp;
  for (auto & p : offset_fp) {
    p = autoware_utils_geometry::Point2d{p.x() + 5.0, p.y()};  // 5.0m offset
  }
  test_footprints.push_back(offset_fp);

  // Act:
  const auto sides_array = footprints::get_sides_from_footprints(test_footprints);

  // Assert:
  ASSERT_EQ(sides_array.size(), 2);

  EXPECT_DOUBLE_EQ(sides_array[0].left.first.x(), base_fp[VehicleInfo::FrontLeftIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[0].left.first.y(), base_fp[VehicleInfo::FrontLeftIndex].y());
  EXPECT_DOUBLE_EQ(sides_array[0].left.second.x(), base_fp[VehicleInfo::RearLeftIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[0].left.second.y(), base_fp[VehicleInfo::RearLeftIndex].y());

  EXPECT_DOUBLE_EQ(sides_array[0].right.first.x(), base_fp[VehicleInfo::FrontRightIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[0].right.first.y(), base_fp[VehicleInfo::FrontRightIndex].y());
  EXPECT_DOUBLE_EQ(sides_array[0].right.second.x(), base_fp[VehicleInfo::RearRightIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[0].right.second.y(), base_fp[VehicleInfo::RearRightIndex].y());

  EXPECT_DOUBLE_EQ(sides_array[1].left.first.x(), offset_fp[VehicleInfo::FrontLeftIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[1].left.first.y(), offset_fp[VehicleInfo::FrontLeftIndex].y());
  EXPECT_DOUBLE_EQ(sides_array[1].right.second.x(), offset_fp[VehicleInfo::RearRightIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[1].right.second.y(), offset_fp[VehicleInfo::RearRightIndex].y());
}
}  // namespace autoware::boundary_departure_checker
