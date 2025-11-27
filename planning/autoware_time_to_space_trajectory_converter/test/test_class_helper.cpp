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
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
// implied. See the License for the specific language governing
// permissions and limitations under the License.

#include "class_helper.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>
#include <tf2/utils.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::time_to_space_trajectory_converter::helper
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;

// ---------------------------------------------------------
// 1. Check Odometry Msg
// ---------------------------------------------------------
TEST(UtilsTest, CheckOdometryMsg)
{
  const auto now = rclcpp::Time(10, 0);  // T=10.0s
  const double timeout = 1.0;

  // Case 1: Null Pointer
  {
    EXPECT_TRUE(check_odometry_msg(nullptr, now, timeout).has_value())
      << "Should return error for nullptr";
  }

  // Case 2: Stale Data (Time = 8.0, Delay = 2.0 > 1.0)
  {
    auto odom = std::make_shared<Odometry>();
    odom->header.stamp = rclcpp::Time(8, 0);
    auto result = check_odometry_msg(odom, now, timeout);
    ASSERT_TRUE(result.has_value());
    EXPECT_NE(result->find("stale"), std::string::npos);
  }

  // Case 3: Fresh Data (Time = 9.5, Delay = 0.5 < 1.0)
  {
    auto odom = std::make_shared<Odometry>();
    odom->header.stamp = rclcpp::Time(9, 500000000);
    EXPECT_FALSE(check_odometry_msg(odom, now, timeout).has_value())
      << "Should return nullopt (success) for fresh data";
  }
}

// ---------------------------------------------------------
// 2. Check Trajectory Msg
// ---------------------------------------------------------
TEST(UtilsTest, CheckTrajectoryMsg)
{
  const auto now = rclcpp::Time(10, 0);
  const double timeout = 1.0;

  // Case 1: Null Pointer
  {
    EXPECT_TRUE(check_trajectory_msg(nullptr, now, timeout).has_value());
  }

  // Case 2: Empty Points
  // NOTE: Your provided code had `!traj_ptr->points.empty()`.
  // If that was intentional, swap this logic. I assume you meant Fail on Empty.
  {
    auto traj = std::make_shared<Trajectory>();
    traj->points.clear();
    EXPECT_TRUE(check_trajectory_msg(traj, now, timeout).has_value())
      << "Should fail on empty trajectory";
  }

  // Case 3: Stale Trajectory
  {
    auto traj = std::make_shared<Trajectory>();
    traj->header.stamp = rclcpp::Time(8, 0);  // Stale
    traj->points.resize(2);                   // Not empty
    // Ensure monotonic to pass that check
    traj->points[0].time_from_start = rclcpp::Duration::from_seconds(0.0);
    traj->points[1].time_from_start = rclcpp::Duration::from_seconds(1.0);

    auto result = check_trajectory_msg(traj, now, timeout);
    ASSERT_TRUE(result.has_value());
    EXPECT_NE(result->find("stale"), std::string::npos);
  }

  // Case 4: Non-Monotonic (Fresh but invalid data)
  {
    auto traj = std::make_shared<Trajectory>();
    traj->header.stamp = rclcpp::Time(10, 0);
    traj->points.resize(2);
    traj->points[0].time_from_start = rclcpp::Duration::from_seconds(1.0);
    traj->points[1].time_from_start = rclcpp::Duration::from_seconds(0.5);  // Backwards

    auto result = check_trajectory_msg(traj, now, timeout);
    ASSERT_TRUE(result.has_value());
    EXPECT_NE(result->find("not increasing"), std::string::npos);
  }

  // Case 5: Valid
  {
    auto traj = std::make_shared<Trajectory>();
    traj->header.stamp = rclcpp::Time(10, 0);
    traj->points.resize(2);
    traj->points[0].time_from_start = rclcpp::Duration::from_seconds(0.0);
    traj->points[1].time_from_start = rclcpp::Duration::from_seconds(1.0);

    EXPECT_FALSE(check_trajectory_msg(traj, now, timeout).has_value())
      << "Should succeed for valid input";
  }
}

// ---------------------------------------------------------
// 3. Has Non Monotonic
// ---------------------------------------------------------
TEST(UtilsTest, HasNonMonotonic)
{
  std::vector<TrajectoryPoint> points;

  // Case 1: Empty or Single point (Always monotonic)
  EXPECT_FALSE(has_non_monotonic(points).has_value());
  points.resize(1);
  EXPECT_FALSE(has_non_monotonic(points).has_value());

  // Case 2: Strictly Increasing (Good)
  points.resize(3);
  points[0].time_from_start = rclcpp::Duration::from_seconds(0.0);
  points[1].time_from_start = rclcpp::Duration::from_seconds(1.0);
  points[2].time_from_start = rclcpp::Duration::from_seconds(2.0);
  EXPECT_FALSE(has_non_monotonic(points).has_value());

  // Case 3: Same time (Usually okay, depending on epsilon check)
  // Logic: curr < prev + eps. If curr == prev, false < false + eps -> false.
  // Should be Valid.
  points[2].time_from_start = rclcpp::Duration::from_seconds(1.0);
  EXPECT_FALSE(has_non_monotonic(points).has_value());

  // Case 4: Decreasing (Bad)
  points[2].time_from_start = rclcpp::Duration::from_seconds(0.9);
  auto result = has_non_monotonic(points);
  ASSERT_TRUE(result.has_value());
  // Verify error index formatting
  EXPECT_NE(result->find("index 2"), std::string::npos);
}

// ---------------------------------------------------------
// 4. Convert Msg to PlannerPoints
// ---------------------------------------------------------
TEST(UtilsTest, ConvertMsgToPlannerPoints)
{
  Trajectory traj;
  traj.points.resize(1);

  // Set Input Values
  traj.points[0].time_from_start = rclcpp::Duration::from_seconds(1.5);
  traj.points[0].pose.position.x = 10.0;
  traj.points[0].pose.position.y = 5.0;
  traj.points[0].pose.position.z = 1.0;
  traj.points[0].longitudinal_velocity_mps = 15.0;
  traj.points[0].acceleration_mps2 = 2.0;

  // Set Orientation (Yaw = 90 degrees / PI/2)
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  traj.points[0].pose.orientation = tf2::toMsg(q);

  // Execute
  auto result = convert_msg_to_planner_points(traj);

  // Verify
  ASSERT_EQ(result.size(), 1);
  EXPECT_NEAR(result[0].t, 1.5, 1e-6);
  EXPECT_NEAR(result[0].pos.x, 10.0, 1e-6);
  EXPECT_NEAR(result[0].pos.y, 5.0, 1e-6);
  EXPECT_NEAR(result[0].v, 15.0, 1e-6);
  EXPECT_NEAR(result[0].a, 2.0, 1e-6);
  EXPECT_NEAR(result[0].yaw, M_PI_2, 1e-6);
}

TEST(TrajectoryConverterTest, AutoCalculateYaw)
{
  SplineData data;

  // Create a path moving diagonally (45 degrees)
  // (0,0) -> (1,1) -> (2,2)
  data.s = {0.0, 1.414, 2.828};
  data.x = {0.0, 1.0, 2.0};
  data.y = {0.0, 1.0, 2.0};
  data.z = {0.0, 0.0, 0.0};
  data.t = {0.0, 1.0, 2.0};
  data.v = {1.0, 1.0, 1.0};

  auto points = convert_spline_data_to_trajectory_points(data);

  ASSERT_EQ(points.size(), 3);

  // Verify Yaw Calculation
  // 45 degrees = PI/4 ~= 0.785 rad
  // Quat Z = sin(22.5 deg) ~= 0.38268
  // Quat W = cos(22.5 deg) ~= 0.92388

  // Check Point 0 (Lookahead to Point 1)
  EXPECT_NEAR(points[0].pose.orientation.z, 0.38268, g_geom_eps);
  EXPECT_NEAR(points[0].pose.orientation.w, 0.92388, g_geom_eps);

  // Check Point 2 (Last point, should copy Point 1's heading)
  EXPECT_NEAR(points[2].pose.orientation.z, 0.38268, g_geom_eps);
}

TEST(TrajectoryConverterTest, HandleStationaryPoints)
{
  // Test if yaw is preserved when car stops (x/y don't change)
  SplineData data;
  // Move X (0->1), then Stop (1->1)
  data.s = {0.0, 1.0, 1.0};
  data.x = {0.0, 1.0, 1.0};
  data.y = {0.0, 0.0, 0.0};
  data.t = {0.0, 1.0, 2.0};
  data.v = {1.0, 0.0, 0.0};
  data.z = {0.0, 0.0, 0.0};

  auto points = convert_spline_data_to_trajectory_points(data);

  // Point 0: Moving +X. Yaw = 0. Quat=(0,0,0,1)
  EXPECT_NEAR(points[0].pose.orientation.z, 0.0, g_geom_eps);
  EXPECT_NEAR(points[0].pose.orientation.w, 1.0, g_geom_eps);

  // Point 1: Next point is same pos. Logic should preserve previous Yaw (0).
  EXPECT_NEAR(points[1].pose.orientation.z, 0.0, g_geom_eps);
  EXPECT_NEAR(points[1].pose.orientation.w, 1.0, g_geom_eps);
}

class IntegrationTest : public ::testing::Test
{
protected:
  // Helper to build a spline quickly
  HermiteSpline build_v_spline(const std::vector<double> & s, const std::vector<double> & v)
  {
    HermiteSpline sv;
    sv.build_impl(s, v);
    return sv;
  }
};

TEST_F(IntegrationTest, ConstantVelocity)
{
  auto sv = build_v_spline({0.0, 10.0}, {10.0, 10.0});
  // Testing the hybrid function
  double dt = solve_time_step(0.0, 10.0, sv);
  EXPECT_NEAR(dt, 1.0, g_math_eps);
}

TEST_F(IntegrationTest, LinearVelocityRamp)
{
  // Scenario: Linear Ramp 0 -> 20 m/s over 100m.
  auto sv = build_v_spline({0.0, 100.0}, {0.0, 20.0});

  // Analytical: 3.4657s. Simpson's: 3.4722s.
  double dt = solve_time_step(10.0, 20.0, sv);
  EXPECT_NEAR(dt, 3.4722, 1e-4);
}

TEST_F(IntegrationTest, StopHandling)
{
  // Scenario: Stopped (v=0).
  auto sv = build_v_spline({0.0, 10.0}, {0.0, 0.0});

  // Distance 1.0m.
  // Code uses 1e-3 (0.001) as minimum divisor.
  // dt = 1.0 / 0.001 = 1000.0s
  double dt = solve_time_step(0.0, 1.0, sv);
  EXPECT_NEAR(dt, 1000.0, 1e-3);
}

TEST_F(IntegrationTest, ZeroDistance)
{
  auto sv = build_v_spline({0.0, 10.0}, {10.0, 10.0});
  double dt = solve_time_step(5.0, 5.0, sv);
  EXPECT_EQ(dt, 0.0);
}

class ResampleTest : public ::testing::Test
{
protected:
  static SplineData create_input(
    const std::vector<double> & s, const std::vector<double> & x, const std::vector<double> & v,
    const std::vector<double> & a)
  {
    SplineData d;
    d.s = s;
    d.x = x;
    d.v = v;
    size_t n = s.size();
    d.y.resize(n, 0.0);
    d.z.resize(n, 0.0);
    d.t.resize(n, 0.0);
    d.wait_times.resize(n, 0.0);
    if (a.empty())
      d.a.resize(n, 0.0);
    else
      d.a = a;
    return d;
  }

  // Generate physics-accurate velocity for v = sqrt(v0^2 + 2as)
  static double calc_physics_v(double s, double v0, double a)
  {
    double val = v0 * v0 + 2 * a * s;
    return (val > 0) ? std::sqrt(val) : 0.0;
  }
};

TEST_F(ResampleTest, BasicResolutionCheck)
{
  const double total_dist = 100.0;
  const double resolution = 1.0;
  const size_t expected_points = static_cast<size_t>(total_dist / resolution) + 1;

  auto input = create_input({0.0, total_dist}, {0.0, total_dist}, {10.0, 10.0}, {0.0, 0.0});

  SplineData output = resample(input, resolution, false);

  EXPECT_EQ(output.s.size(), expected_points);
  EXPECT_NEAR(output.s[1] - output.s[0], resolution, 1e-5);
}

TEST_F(ResampleTest, RecomputeAcceleration)
{
  // Scenario: Constant acceleration from rest
  // v0 = 0, a_target = 5.0 m/s^2
  // We generate input points that perfectly match this physics
  const double a_target = 5.0;
  const double v0 = 0.0;
  const double max_s = 10.0;
  const double ds_input = 0.5;

  std::vector<double> s_vec, x_vec, v_vec, a_original;
  for (double s = 0.0; s <= max_s; s += ds_input) {
    s_vec.push_back(s);
    x_vec.push_back(s);
    v_vec.push_back(calc_physics_v(s, v0, a_target));
    a_original.push_back(0.0);  // Dummy original data
  }

  auto input = create_input(s_vec, x_vec, v_vec, a_original);

  // 1. Verify Recompute Logic (Should recover a_target)
  // We use a fine resolution to minimize discrete error
  SplineData output_recomp = resample(input, 0.1, true);

  // Check the middle of the trajectory where derivatives are stable
  size_t mid_idx = output_recomp.a.size() / 2;
  EXPECT_NEAR(output_recomp.a[mid_idx], a_target, 0.5);

  // 2. Verify Original Logic (Should keep 0.0)
  SplineData output_orig = resample(input, 0.1, false);
  EXPECT_NEAR(output_orig.a[mid_idx], 0.0, 1e-3);
}

TEST_F(ResampleTest, PhysicsIntegration_Ramp)
{
  // Scenario: Accelerate 5.0 -> 20.0 m/s over 100m.
  constexpr double v0 = 5.0;
  constexpr double a = 1.875;

  // Use dense knots to ensure spline matches physics curve
  std::vector<double> s_knots;
  std::vector<double> v_knots;
  std::vector<double> a_knots;
  for (double s = 0.0; s <= 100.0 + 1e-3; s += 10.0) {
    s_knots.push_back(s);
    v_knots.push_back(calc_physics_v(s, v0, a));
    a_knots.push_back(a);
  }

  auto input = create_input(s_knots, s_knots, v_knots, a_knots);
  SplineData output = resample(input, 1.0, false);

  // Expected Time: t = (v - u) / a = 15 / 1.875 = 8.0s
  EXPECT_NEAR(output.t.back(), 8.0, 0.1);
}

TEST_F(ResampleTest, PhysicsIntegration_Time)
{
  // Scenario: Linear Velocity Ramp
  const double v0 = 5.0;
  const double a = 1.875;
  const double total_s = 100.0;

  const double v_final = calc_physics_v(total_s, v0, a);
  const double expected_total_time = (v_final - v0) / a;

  std::vector<double> s_knots, x_knots, v_knots, a_knots;
  for (double s = 0.0; s <= total_s + 1e-3; s += 10.0) {
    s_knots.push_back(s);
    x_knots.push_back(s);  // Corrected: Push to x_knots, not s_knots again
    v_knots.push_back(calc_physics_v(s, v0, a));
    a_knots.push_back(a);
  }

  // Use correct vectors
  auto input = create_input(s_knots, x_knots, v_knots, a_knots);
  SplineData output = resample(input, 1.0, false);

  EXPECT_NEAR(output.t.back(), expected_total_time, 0.1);
}

TEST_F(ResampleTest, ZAxisLinearity)
{
  auto input = create_input({0.0, 100.0}, {0.0, 100.0}, {10.0, 10.0}, {0.0, 0.0});
  input.z[0] = 0.0;
  input.z[1] = 10.0;

  SplineData output = resample(input, 1.0, false);
  EXPECT_NEAR(output.z[50], 5.0, g_math_eps);
}

TEST_F(ResampleTest, HandleStopIntegration)
{
  // Scenario: Decelerate 10.0 -> 0.0 m/s over 20m.
  constexpr double v0 = 10.0;
  constexpr double a = -2.5;

  // FIX: Use NON-LINEAR knot distribution.
  // We need high density ONLY near the stop (s=20) to capture the sharp velocity drop.
  std::vector<double> s_knots = {
    0.0,  5.0,  10.0, 15.0,       // Coarse start
    18.0, 19.0, 19.5, 19.8, 20.0  // Super-dense end
  };

  std::vector<double> v_knots;
  std::vector<double> a_knots;
  for (double s : s_knots) {
    v_knots.push_back(calc_physics_v(s, v0, a));
    a_knots.push_back(a);
  }
  // Ensure exact zero at end to avoid float noise
  v_knots.back() = 0.0;

  auto input = create_input(s_knots, s_knots, v_knots, a_knots);
  SplineData output = resample(input, 0.5, false);

  EXPECT_NEAR(output.v.back(), 0.0, g_math_eps);

  // Time check: t = 4.0s
  // With the dense knots, the error drops from ~0.49s to < 0.1s
  EXPECT_NEAR(output.t.back(), 4.0, 0.1);
}

TEST_F(ResampleTest, WaitTimeAccumulation)
{
  // Scenario: Move 10m @ 10m/s (1s) -> Wait 5s -> Move 10m @ 10m/s (1s)
  std::vector<double> s = {0.0, 10.0, 20.0};
  std::vector<double> x = {0.0, 10.0, 20.0};
  std::vector<double> v = {10.0, 10.0, 10.0};

  auto input = create_input(s, x, v, {0.0, 0.0, 0.0});
  input.wait_times[1] = 5.0;  // Explicit wait at 10m

  SplineData output = resample(input, 1.0, false);

  // Check arrival at 10m (should be ~1.0s)
  EXPECT_NEAR(output.t[10], 1.0, 0.1);

  // Check point immediately after 10m (e.g., 11m)
  // It should be 1.0s (travel) + 5.0s (wait) + 0.1s (travel 1m) = 6.1s
  EXPECT_NEAR(output.t[11], 6.1, 0.1);

  // Total time: 1s + 5s + 1s = 7s
  EXPECT_NEAR(output.t.back(), 7.0, 0.1);
}
}  // namespace autoware::time_to_space_trajectory_converter::helper
