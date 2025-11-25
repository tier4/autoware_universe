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

#include <gtest/gtest.h>
#include <lane_departure_checker/lane_departure_checker.hpp>
#include <lane_departure_checker/util/create_vehicle_footprint.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>

#include <cmath>
#include <vector>

using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using lane_departure_checker::LaneDepartureChecker;
using lane_departure_checker::Param;

class LaneDepartureCheckerTestable : public LaneDepartureChecker
{
public:
  using LaneDepartureChecker::createVehicleFootprints;
};

vehicle_info_util::VehicleInfo createVehicleInfo(
  double wheel_base, double front_overhang, double rear_overhang,
  double wheel_tread, double left_overhang, double right_overhang)
{
  vehicle_info_util::VehicleInfo info;
  info.wheel_base_m = wheel_base;
  info.front_overhang_m = front_overhang;
  info.rear_overhang_m = rear_overhang;
  info.wheel_tread_m = wheel_tread;
  info.left_overhang_m = left_overhang;
  info.right_overhang_m = right_overhang;
  return info;
}

Param createParam(double footprint_margin_scale)
{
  Param param;
  param.footprint_margin_scale = footprint_margin_scale;
  param.resample_interval = 2.0;
  param.max_deceleration = 2.8;
  param.delay_time = 0.3;
  param.max_lateral_deviation = 2.0;
  param.max_longitudinal_deviation = 2.0;
  param.max_yaw_deviation_deg = 60.0;
  param.delta_yaw_threshold_for_closest_point = M_PI / 3.0;
  return param;
}

geometry_msgs::msg::PoseWithCovariance createPoseWithCovariance(
  double yaw, double cov_xx, double cov_yy)
{
  geometry_msgs::msg::PoseWithCovariance pose_cov;
  pose_cov.pose.position.x = 0.0;
  pose_cov.pose.position.y = 0.0;
  pose_cov.pose.position.z = 0.0;
  
  pose_cov.pose.orientation.x = 0.0;
  pose_cov.pose.orientation.y = 0.0;
  pose_cov.pose.orientation.z = std::sin(yaw / 2.0);
  pose_cov.pose.orientation.w = std::cos(yaw / 2.0);
  
  // 共分散行列の初期化（6x6）
  for (int i = 0; i < 36; ++i) {
    pose_cov.covariance[i] = 0.0;
  }
  pose_cov.covariance[0 * 6 + 0] = cov_xx;  // x分散
  pose_cov.covariance[1 * 6 + 1] = cov_yy;  // y分散
  
  return pose_cov;
}

TrajectoryPoint createTrajectoryPoint(double x, double y, double yaw)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = 0.0;
  point.pose.orientation.x = 0.0;
  point.pose.orientation.y = 0.0;
  point.pose.orientation.z = std::sin(yaw / 2.0);
  point.pose.orientation.w = std::cos(yaw / 2.0);
  return point;
}

TEST(LaneDepartureChecker, CreateVehicleFootprints)
{
  const double wheel_base = 1.335;
  const double front_overhang = 0.530;
  const double rear_overhang = 0.375;
  const double wheel_tread = 0.955;
  const double left_overhang = 0.0725;
  const double right_overhang = 0.0725;
  
  auto vehicle_info = createVehicleInfo(
    wheel_base, front_overhang, rear_overhang,
    wheel_tread, left_overhang, right_overhang);
  
  auto param = createParam(2.0);
  
  LaneDepartureCheckerTestable checker;
  checker.setParam(param, vehicle_info);
  
  std::vector<TrajectoryPoint> trajectory;
  trajectory.push_back(createTrajectoryPoint(0.0, 0.0, 0.0));
  trajectory.push_back(createTrajectoryPoint(2.0, 5.0, 30.0/180.0*M_PI));
  trajectory.push_back(createTrajectoryPoint(8.0, 6.0, 60.0/180.0*M_PI));
  
  auto pose_cov = createPoseWithCovariance(30.0/180.0*M_PI, 0.04, 0.12);
  
  auto footprints = checker.createVehicleFootprints(pose_cov, trajectory, param);
  
  ASSERT_EQ(footprints.size(), 3);
  
  for (const auto & footprint : footprints) {
    EXPECT_EQ(footprint.size(), 7);
  }

    // footprint[0]の想定座標
    std::array<double, 2> p0 = {1.985, 0.75};
    std::array<double, 2> p1 = {1.985, -0.75};
    std::array<double, 2> p2 = {0.6675, -0.75};
    std::array<double, 2> p3 = {-0.495, -0.75};
    std::array<double, 2> p4 = {-0.495, 0.75};
    std::array<double, 2> p5 = {0.6675, 0.75};

    EXPECT_NEAR(footprints[0][0].x(), p0[0], 1e-3);
    EXPECT_NEAR(footprints[0][0].y(), p0[1], 1e-3);
    EXPECT_NEAR(footprints[0][1].x(), p1[0], 1e-3);
    EXPECT_NEAR(footprints[0][1].y(), p1[1], 1e-3);
    EXPECT_NEAR(footprints[0][2].x(), p2[0], 1e-3);
    EXPECT_NEAR(footprints[0][2].y(), p2[1], 1e-3);
    EXPECT_NEAR(footprints[0][3].x(), p3[0], 1e-3);
    EXPECT_NEAR(footprints[0][3].y(), p3[1], 1e-3);
    EXPECT_NEAR(footprints[0][4].x(), p4[0], 1e-3);
    EXPECT_NEAR(footprints[0][4].y(), p4[1], 1e-3);
    EXPECT_NEAR(footprints[0][5].x(), p5[0], 1e-3);
    EXPECT_NEAR(footprints[0][5].y(), p5[1], 1e-3);
    EXPECT_NEAR(footprints[0][6].x(), p0[0], 1e-3);
    EXPECT_NEAR(footprints[0][6].y(), p0[1], 1e-3);

    // footprint[1] 回転+並進したもの
    const double root3 = 1.73205080757;
    std::array<double, 2> q0 = {(root3 * p0[0] - p0[1]) / 2.0 + 2.0, (p0[0] + root3 * p0[1]) / 2.0 + 5.0};
    std::array<double, 2> q1 = {(root3 * p1[0] - p1[1]) / 2.0 + 2.0, (p1[0] + root3 * p1[1]) / 2.0 + 5.0};
    std::array<double, 2> q2 = {(root3 * p2[0] - p2[1]) / 2.0 + 2.0, (p2[0] + root3 * p2[1]) / 2.0 + 5.0};
    std::array<double, 2> q3 = {(root3 * p3[0] - p3[1]) / 2.0 + 2.0, (p3[0] + root3 * p3[1]) / 2.0 + 5.0};
    std::array<double, 2> q4 = {(root3 * p4[0] - p4[1]) / 2.0 + 2.0, (p4[0] + root3 * p4[1]) / 2.0 + 5.0};
    std::array<double, 2> q5 = {(root3 * p5[0] - p5[1]) / 2.0 + 2.0, (p5[0] + root3 * p5[1]) / 2.0 + 5.0};

    EXPECT_NEAR(footprints[1][0].x(), q0[0], 1e-3);
    EXPECT_NEAR(footprints[1][0].y(), q0[1], 1e-3);
    EXPECT_NEAR(footprints[1][1].x(), q1[0], 1e-3);
    EXPECT_NEAR(footprints[1][1].y(), q1[1], 1e-3);
    EXPECT_NEAR(footprints[1][2].x(), q2[0], 1e-3);
    EXPECT_NEAR(footprints[1][2].y(), q2[1], 1e-3);
    EXPECT_NEAR(footprints[1][3].x(), q3[0], 1e-3);
    EXPECT_NEAR(footprints[1][3].y(), q3[1], 1e-3);
    EXPECT_NEAR(footprints[1][4].x(), q4[0], 1e-3);
    EXPECT_NEAR(footprints[1][4].y(), q4[1], 1e-3);
    EXPECT_NEAR(footprints[1][5].x(), q5[0], 1e-3);
    EXPECT_NEAR(footprints[1][5].y(), q5[1], 1e-3);
    EXPECT_NEAR(footprints[1][6].x(), q0[0], 1e-3);
    EXPECT_NEAR(footprints[1][6].y(), q0[1], 1e-3);

    // footprint[2] 別変換
    q0 = {(p0[0] - root3 * p0[1]) / 2.0 + 8.0, (root3 * p0[0] + p0[1]) / 2.0 + 6.0};
    q1 = {(p1[0] - root3 * p1[1]) / 2.0 + 8.0, (root3 * p1[0] + p1[1]) / 2.0 + 6.0};
    q2 = {(p2[0] - root3 * p2[1]) / 2.0 + 8.0, (root3 * p2[0] + p2[1]) / 2.0 + 6.0};
    q3 = {(p3[0] - root3 * p3[1]) / 2.0 + 8.0, (root3 * p3[0] + p3[1]) / 2.0 + 6.0};
    q4 = {(p4[0] - root3 * p4[1]) / 2.0 + 8.0, (root3 * p4[0] + p4[1]) / 2.0 + 6.0};
    q5 = {(p5[0] - root3 * p5[1]) / 2.0 + 8.0, (root3 * p5[0] + p5[1]) / 2.0 + 6.0};

    EXPECT_NEAR(footprints[2][0].x(), q0[0], 1e-3);
    EXPECT_NEAR(footprints[2][0].y(), q0[1], 1e-3);
    EXPECT_NEAR(footprints[2][1].x(), q1[0], 1e-3);
    EXPECT_NEAR(footprints[2][1].y(), q1[1], 1e-3);
    EXPECT_NEAR(footprints[2][2].x(), q2[0], 1e-3);
    EXPECT_NEAR(footprints[2][2].y(), q2[1], 1e-3);
    EXPECT_NEAR(footprints[2][3].x(), q3[0], 1e-3);
    EXPECT_NEAR(footprints[2][3].y(), q3[1], 1e-3);
    EXPECT_NEAR(footprints[2][4].x(), q4[0], 1e-3);
    EXPECT_NEAR(footprints[2][4].y(), q4[1], 1e-3);
    EXPECT_NEAR(footprints[2][5].x(), q5[0], 1e-3);
    EXPECT_NEAR(footprints[2][5].y(), q5[1], 1e-3);
    EXPECT_NEAR(footprints[2][6].x(), q0[0], 1e-3);
    EXPECT_NEAR(footprints[2][6].y(), q0[1], 1e-3);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}