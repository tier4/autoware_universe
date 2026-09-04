// Copyright 2026 TIER IV, inc.
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

#include "autoware/map_based_prediction/predictor_vehicle/path_processing.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>

namespace autoware::map_based_prediction
{
namespace
{
class LateralAccelerationConstraintTest : public ::testing::Test
{
protected:
  static constexpr double initial_speed = 10.0;                  // [m/s]
  static constexpr double max_lateral_accel = 2.0;               // [m/s^2]
  static constexpr double min_acceleration_before_curve = -2.0;  // [m/s^2]
  static constexpr double point_interval = 1.0;                  // [m]

  // The test cases below use powers of two for the yaw differences so that every intermediate value
  // is exactly representable as a double and the expectations hold without a tolerance.
  // With the values above, the lowest speed the object can reach after braking over an arc length s
  // is v^2 = 100 - 4 * s, and a segment is rejected when curvature * v^2 > 2.0.

  void SetUp() override
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_path_processing");
    path_processor_ = std::make_shared<PathProcessor>(*node_);
    setParams(max_lateral_accel, min_acceleration_before_curve);
  }

  void setParams(const double max_lateral_accel_param, const double min_acceleration_param)
  {
    PathProcessor::Params params;
    params.max_lateral_accel = max_lateral_accel_param;
    params.min_acceleration_before_curve = min_acceleration_param;
    path_processor_->setParams(params);
  }

  bool isSatisfied(const TrajectoryPoints & trajectory) const
  {
    return path_processor_->isLateralAccelerationConstraintSatisfied(trajectory);
  }

  static void appendPoint(TrajectoryPoints & trajectory, const double x, const double yaw)
  {
    TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
    point.longitudinal_velocity_mps = initial_speed;
    trajectory.push_back(point);
  }

  /**
   * @brief make a trajectory that runs straight and has a single curved segment
   *
   * The points are laid out along the x axis, and the points after the curve are given a yaw of
   * `curve_delta_yaw`, so that the segment entered at `curve_entry_arc_length` has a curvature of
   * `curve_delta_yaw / curve_delta_s` while every other segment is straight.
   *
   * Note that the positions and the orientations are intentionally not consistent with each other:
   * the check derives the curvature from the distance and the yaw difference of the two points
   * independently, so building the inputs this way keeps the curvature of the segment under test
   * exact and hand-verifiable.
   */
  static TrajectoryPoints makeTrajectoryWithSingleCurve(
    const double curve_entry_arc_length, const double curve_delta_s, const double curve_delta_yaw,
    const double total_arc_length)
  {
    TrajectoryPoints trajectory;
    for (double x = 0.0; x <= curve_entry_arc_length; x += point_interval) {
      appendPoint(trajectory, x, 0.0);
    }
    for (double x = curve_entry_arc_length + curve_delta_s; x <= total_arc_length;
         x += point_interval) {
      appendPoint(trajectory, x, curve_delta_yaw);
    }
    return trajectory;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<PathProcessor> path_processor_;
};

TEST_F(LateralAccelerationConstraintTest, straightTrajectoryIsSatisfied)
{
  // no yaw difference anywhere, so the curvature is 0 and the lateral acceleration is 0
  const auto trajectory = makeTrajectoryWithSingleCurve(10.0, point_interval, 0.0, 20.0);
  EXPECT_TRUE(isSatisfied(trajectory));
}

TEST_F(LateralAccelerationConstraintTest, gentleCurveIsSatisfied)
{
  // curvature = 0.0078125, lateral acceleration = 0.0078125 * 100 = 0.78125 <= 2.0
  const auto trajectory = makeTrajectoryWithSingleCurve(0.0, point_interval, 0.0078125, 20.0);
  EXPECT_TRUE(isSatisfied(trajectory));
}

TEST_F(LateralAccelerationConstraintTest, sharpCurveAtTheBeginningIsNotSatisfied)
{
  // curvature = 0.0625 with no distance to brake, so 0.0625 * 100 = 6.25 > 2.0
  const auto trajectory = makeTrajectoryWithSingleCurve(0.0, point_interval, 0.0625, 20.0);
  EXPECT_FALSE(isSatisfied(trajectory));
}

TEST_F(LateralAccelerationConstraintTest, curveFarEnoughToBrakeIsSatisfied)
{
  // curvature = 0.0625, v^2 = 100 - 4 * 18 = 28, so 0.0625 * 28 = 1.75 <= 2.0
  const auto trajectory = makeTrajectoryWithSingleCurve(18.0, point_interval, 0.0625, 30.0);
  EXPECT_TRUE(isSatisfied(trajectory));
}

TEST_F(LateralAccelerationConstraintTest, curveTooCloseToBrakeIsNotSatisfied)
{
  // curvature = 0.0625, v^2 = 100 - 4 * 16 = 36, so 0.0625 * 36 = 2.25 > 2.0
  const auto trajectory = makeTrajectoryWithSingleCurve(16.0, point_interval, 0.0625, 30.0);
  EXPECT_FALSE(isSatisfied(trajectory));
}

TEST_F(LateralAccelerationConstraintTest, brakingDistanceIsMeasuredToTheCurveEntry)
{
  // The curved segment is 4 m long, so crediting its own length as braking distance would change
  // the verdict: v^2 = 100 - 4 * 15 = 40 gives 0.0625 * 40 = 2.5 > 2.0 at the entry of the curve,
  // while the exit at 19 m would give 0.0625 * 24 = 1.5 <= 2.0.
  const auto trajectory = makeTrajectoryWithSingleCurve(15.0, 4.0, 0.25, 30.0);
  EXPECT_FALSE(isSatisfied(trajectory));
}

TEST_F(LateralAccelerationConstraintTest, curveAfterStoppingDistanceIsSatisfied)
{
  // The object can stop within 25 m, so v^2 = 100 - 4 * 40 = -60 is negative and any curvature is
  // acceptable there.
  const auto trajectory = makeTrajectoryWithSingleCurve(40.0, point_interval, 0.5, 50.0);
  EXPECT_TRUE(isSatisfied(trajectory));
}

TEST_F(LateralAccelerationConstraintTest, duplicatedPointsAreSkipped)
{
  // A pair of points at the same position would give an infinite curvature without the guard.
  TrajectoryPoints trajectory;
  appendPoint(trajectory, 0.0, 0.0);
  appendPoint(trajectory, 0.0, 0.5);
  for (double x = point_interval; x <= 20.0; x += point_interval) {
    appendPoint(trajectory, x, 0.5);
  }
  EXPECT_TRUE(isSatisfied(trajectory));
}

TEST_F(LateralAccelerationConstraintTest, trajectoryWithoutSegmentIsSatisfied)
{
  EXPECT_TRUE(isSatisfied(TrajectoryPoints{}));

  TrajectoryPoints single_point_trajectory;
  appendPoint(single_point_trajectory, 0.0, 0.0);
  EXPECT_TRUE(isSatisfied(single_point_trajectory));
}

TEST_F(LateralAccelerationConstraintTest, parameterSignsAreIgnored)
{
  // the absolute values of the parameters are used, so the verdict does not change
  setParams(-max_lateral_accel, -min_acceleration_before_curve);
  const auto trajectory = makeTrajectoryWithSingleCurve(16.0, point_interval, 0.0625, 30.0);
  EXPECT_FALSE(isSatisfied(trajectory));
}

}  // namespace
}  // namespace autoware::map_based_prediction
