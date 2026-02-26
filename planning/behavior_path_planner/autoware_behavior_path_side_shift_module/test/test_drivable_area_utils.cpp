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

#include "autoware/behavior_path_side_shift_module/drivable_area_utils.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <atomic>
#include <cmath>

namespace autoware::behavior_path_planner
{

// ---------------------------------------------------------------------------
// Helper: create a straight horizontal lanelet from (x_start, y_left) to
// (x_end, y_left) for left bound and (x_start, y_right) to (x_end, y_right)
// for right bound.  Left boundary is at higher y, right boundary at lower y
// (standard Autoware convention when heading in +x direction).
// ---------------------------------------------------------------------------
namespace
{
std::atomic<lanelet::Id> g_next_id{10000};
lanelet::Id nextId()
{
  return g_next_id++;
}

lanelet::Lanelet makeStraightLanelet(
  lanelet::Id id, double x_start, double x_end, double y_left, double y_right,
  double interval = 1.0)
{
  lanelet::Points3d left_pts, right_pts;
  const int num = static_cast<int>(std::ceil((x_end - x_start) / interval)) + 1;
  for (int i = 0; i < num; ++i) {
    const double x = x_start + i * interval;
    // Use unique IDs to avoid collisions
    left_pts.emplace_back(nextId(), x, y_left, 0.0);
    right_pts.emplace_back(nextId(), x, y_right, 0.0);
  }
  lanelet::LineString3d left_ls(nextId(), left_pts);
  lanelet::LineString3d right_ls(nextId(), right_pts);
  return lanelet::Lanelet(id, left_ls, right_ls);
}
}  // namespace

// ===========================================================================
// Test fixture
// ===========================================================================
class DrivableAreaUtilsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Default lane: x=[0..50], left_y=2.0, right_y=-2.0 → width 4.0 m, centered on y=0
    center_lane_ = makeStraightLanelet(100, 0.0, 50.0, 2.0, -2.0);
    // Left adjacent lane: same x, y=[2.0..5.5] → width 3.5 m
    left_lane_ = makeStraightLanelet(101, 0.0, 50.0, 5.5, 2.0);
    // Right adjacent lane: same x, y=[-2.0..-5.5] → width 3.5 m
    right_lane_ = makeStraightLanelet(102, 0.0, 50.0, -2.0, -5.5);

    vehicle_half_width_ = 0.9;  // typical passenger car ~1.8 m width
    margin_ = 0.3;
  }

  lanelet::Lanelet center_lane_;
  lanelet::Lanelet left_lane_;
  lanelet::Lanelet right_lane_;
  double vehicle_half_width_{};
  double margin_{};
};

// ===========================================================================
// clampOffsetToLimits
// ===========================================================================
TEST(ClampOffsetToLimitsTest, ZeroRequestReturnsZero)
{
  EXPECT_DOUBLE_EQ(clampOffsetToLimits(0.0, 5.0, 5.0), 0.0);
}

TEST(ClampOffsetToLimitsTest, LeftShiftWithinLimit)
{
  // Request 1.0 left, limit 2.0 → should pass through
  EXPECT_DOUBLE_EQ(clampOffsetToLimits(1.0, 2.0, 2.0), 1.0);
}

TEST(ClampOffsetToLimitsTest, LeftShiftExceedsLimit)
{
  // Request 3.0 left, limit 2.0 → clamped to 2.0
  EXPECT_DOUBLE_EQ(clampOffsetToLimits(3.0, 2.0, 5.0), 2.0);
}

TEST(ClampOffsetToLimitsTest, RightShiftWithinLimit)
{
  // Request -1.0, limit 2.0 → should pass through
  EXPECT_DOUBLE_EQ(clampOffsetToLimits(-1.0, 2.0, 2.0), -1.0);
}

TEST(ClampOffsetToLimitsTest, RightShiftExceedsLimit)
{
  // Request -3.0, limit 2.0 → clamped to -2.0
  EXPECT_DOUBLE_EQ(clampOffsetToLimits(-3.0, 5.0, 2.0), -2.0);
}

TEST(ClampOffsetToLimitsTest, LeftShiftSnapsToZeroWhenBelowTolerance)
{
  // Request tiny left shift where limit is very small (< 1e-3)
  EXPECT_DOUBLE_EQ(clampOffsetToLimits(0.5, 0.0005, 5.0), 0.0);
}

TEST(ClampOffsetToLimitsTest, RightShiftSnapsToZeroWhenBelowTolerance)
{
  // Request tiny right shift where limit is very small
  EXPECT_DOUBLE_EQ(clampOffsetToLimits(-0.5, 5.0, 0.0005), 0.0);
}

TEST(ClampOffsetToLimitsTest, LeftShiftExactlyAtLimit)
{
  EXPECT_DOUBLE_EQ(clampOffsetToLimits(2.0, 2.0, 5.0), 2.0);
}

TEST(ClampOffsetToLimitsTest, RightShiftExactlyAtLimit)
{
  EXPECT_DOUBLE_EQ(clampOffsetToLimits(-2.0, 5.0, 2.0), -2.0);
}

TEST(ClampOffsetToLimitsTest, ZeroLimitClampsLeftToZero)
{
  EXPECT_DOUBLE_EQ(clampOffsetToLimits(1.0, 0.0, 5.0), 0.0);
}

TEST(ClampOffsetToLimitsTest, ZeroLimitClampsRightToZero)
{
  EXPECT_DOUBLE_EQ(clampOffsetToLimits(-1.0, 5.0, 0.0), 0.0);
}

// ===========================================================================
// determineLanePosition
// ===========================================================================
TEST_F(DrivableAreaUtilsTest, DeterminePosition_PointInCurrentLane)
{
  const lanelet::BasicPoint2d pt(10.0, 0.0);  // center of the lane
  AdjacentLaneInfo adj;
  const auto result = determineLanePosition(center_lane_, pt, adj);
  EXPECT_FALSE(result.is_in_adjacent);
  EXPECT_FALSE(result.is_outside_all);
  EXPECT_EQ(result.check_lane.id(), center_lane_.id());
}

TEST_F(DrivableAreaUtilsTest, DeterminePosition_PointInLeftAdjacentLane)
{
  const lanelet::BasicPoint2d pt(10.0, 3.5);  // inside left lane
  AdjacentLaneInfo adj;
  adj.has_left = true;
  adj.left_lane = left_lane_;
  const auto result = determineLanePosition(center_lane_, pt, adj);
  EXPECT_TRUE(result.is_in_adjacent);
  EXPECT_FALSE(result.is_outside_all);
  EXPECT_EQ(result.check_lane.id(), left_lane_.id());
}

TEST_F(DrivableAreaUtilsTest, DeterminePosition_PointInRightAdjacentLane)
{
  const lanelet::BasicPoint2d pt(10.0, -3.5);  // inside right lane
  AdjacentLaneInfo adj;
  adj.has_right = true;
  adj.right_lane = right_lane_;
  const auto result = determineLanePosition(center_lane_, pt, adj);
  EXPECT_TRUE(result.is_in_adjacent);
  EXPECT_FALSE(result.is_outside_all);
  EXPECT_EQ(result.check_lane.id(), right_lane_.id());
}

TEST_F(DrivableAreaUtilsTest, DeterminePosition_PointOutsideAll)
{
  const lanelet::BasicPoint2d pt(10.0, 10.0);  // far outside
  AdjacentLaneInfo adj;                        // no adjacent lanes
  const auto result = determineLanePosition(center_lane_, pt, adj);
  EXPECT_FALSE(result.is_in_adjacent);
  EXPECT_TRUE(result.is_outside_all);
}

TEST_F(DrivableAreaUtilsTest, DeterminePosition_NoAdjacentLanesButOutsideCurrent)
{
  // Point is in the left lane area, but no left lane is registered → outside_all
  const lanelet::BasicPoint2d pt(10.0, 3.5);
  AdjacentLaneInfo adj;  // empty
  const auto result = determineLanePosition(center_lane_, pt, adj);
  EXPECT_TRUE(result.is_outside_all);
}

TEST_F(DrivableAreaUtilsTest, DeterminePosition_PointOnLeftBoundary)
{
  // Exactly on left boundary (y=2.0). lanelet::geometry::inside considers this.
  // The exact boundary behavior depends on lanelet2 implementation.
  const lanelet::BasicPoint2d pt(10.0, 2.0);
  AdjacentLaneInfo adj;
  adj.has_left = true;
  adj.left_lane = left_lane_;
  adj.has_right = true;
  adj.right_lane = right_lane_;
  const auto result = determineLanePosition(center_lane_, pt, adj);
  // The point is on boundary - should be classified as either current or adjacent, not outside
  EXPECT_FALSE(result.is_outside_all);
}

// ===========================================================================
// updateLaneLimitsForOutsidePoint
// ===========================================================================
TEST_F(DrivableAreaUtilsTest, OutsidePoint_LeftSide)
{
  // Point is to the left of the lane (y=3.0, left boundary at y=2.0)
  const lanelet::BasicPoint2d pt(10.0, 3.0);
  LaneLimitInfo limits;
  updateLaneLimitsForOutsidePoint(center_lane_, pt, vehicle_half_width_, margin_, limits);

  EXPECT_TRUE(limits.found_valid_limit);
  // Point is on the left → safe_left_limit should be 0 (no more left shift allowed)
  EXPECT_DOUBLE_EQ(limits.safe_left_limit, 0.0);
  // Right limit should be the return distance
  EXPECT_GT(limits.safe_right_limit, 0.0);
}

TEST_F(DrivableAreaUtilsTest, OutsidePoint_RightSide)
{
  // Point is to the right of the lane (y=-3.0, right boundary at y=-2.0)
  const lanelet::BasicPoint2d pt(10.0, -3.0);
  LaneLimitInfo limits;
  updateLaneLimitsForOutsidePoint(center_lane_, pt, vehicle_half_width_, margin_, limits);

  EXPECT_TRUE(limits.found_valid_limit);
  // Point is on the right → safe_right_limit should be 0 (no more right shift allowed)
  EXPECT_DOUBLE_EQ(limits.safe_right_limit, 0.0);
  // Left limit should be the return distance
  EXPECT_GT(limits.safe_left_limit, 0.0);
}

TEST_F(DrivableAreaUtilsTest, OutsidePoint_ReturnDistanceCalculation)
{
  // Point is 1.0 m to the left of left boundary (left boundary at y=2.0, point at y=3.0)
  const lanelet::BasicPoint2d pt(10.0, 3.0);
  LaneLimitInfo limits;
  updateLaneLimitsForOutsidePoint(center_lane_, pt, vehicle_half_width_, margin_, limits);

  // return_distance = dist_to_left_bound + vehicle_half_width + margin
  // dist_to_left ~= 1.0 (approximately, depends on boundary discretization)
  // return_distance ~= 1.0 + 0.9 + 0.3 = 2.2
  EXPECT_NEAR(limits.safe_right_limit, 1.0 + vehicle_half_width_ + margin_, 0.1);
}

TEST_F(DrivableAreaUtilsTest, OutsidePoint_AccumulatesMultiple)
{
  // Two outside points: limits should take the most restrictive
  LaneLimitInfo limits;

  const lanelet::BasicPoint2d pt1(10.0, 3.0);  // left side, 1m from boundary
  updateLaneLimitsForOutsidePoint(center_lane_, pt1, vehicle_half_width_, margin_, limits);
  const double first_right_limit = limits.safe_right_limit;

  const lanelet::BasicPoint2d pt2(20.0, 4.0);  // left side, 2m from boundary → larger return
  updateLaneLimitsForOutsidePoint(center_lane_, pt2, vehicle_half_width_, margin_, limits);

  // safe_left_limit stays 0 (both points on left)
  EXPECT_DOUBLE_EQ(limits.safe_left_limit, 0.0);
  // right limit should not decrease further since min() is applied
  // but pt2 is further away so return_distance is larger → does not reduce right_limit
  // The first point was closer, so its return_distance is smaller → more restrictive
  EXPECT_DOUBLE_EQ(limits.safe_right_limit, first_right_limit);
}

// ===========================================================================
// updateLaneLimitsForInsidePoint
// ===========================================================================
TEST_F(DrivableAreaUtilsTest, InsidePoint_CenterOfLane_NoAdjacent)
{
  // Point at center of a 4m-wide lane
  const lanelet::BasicPoint2d pt(10.0, 0.0);
  LaneLimitInfo limits;
  const lanelet::ConstLanelet dummy;  // not used when allow_left/right = false

  updateLaneLimitsForInsidePoint(
    center_lane_, pt, /*allow_left=*/false, /*allow_right=*/false,
    /*point_in_adjacent=*/false, dummy, dummy, vehicle_half_width_, margin_, limits);

  EXPECT_TRUE(limits.found_valid_limit);
  // dist_to_left = 2.0, dist_to_right = 2.0
  // max_left = 2.0 - 0.9 - 0.3 - 0.05 = 0.75
  // max_right = same = 0.75
  const double expected = 2.0 - vehicle_half_width_ - margin_ - kLaneBoundaryDiscretizationBuffer;
  EXPECT_NEAR(limits.safe_left_limit, expected, 0.05);
  EXPECT_NEAR(limits.safe_right_limit, expected, 0.05);
}

TEST_F(DrivableAreaUtilsTest, InsidePoint_OffsetFromCenter)
{
  // Point shifted 1m to the left (y=1.0) inside a 4m-wide lane
  const lanelet::BasicPoint2d pt(10.0, 1.0);
  LaneLimitInfo limits;
  const lanelet::ConstLanelet dummy;

  updateLaneLimitsForInsidePoint(
    center_lane_, pt, false, false, false, dummy, dummy, vehicle_half_width_, margin_, limits);

  EXPECT_TRUE(limits.found_valid_limit);
  // dist_to_left ≈ 1.0, dist_to_right ≈ 3.0
  const double expected_left =
    1.0 - vehicle_half_width_ - margin_ - kLaneBoundaryDiscretizationBuffer;
  const double expected_right =
    3.0 - vehicle_half_width_ - margin_ - kLaneBoundaryDiscretizationBuffer;
  // If expected_left < 0, it becomes max(0, negative) = 0
  EXPECT_NEAR(limits.safe_left_limit, std::max(0.0, expected_left), 0.05);
  EXPECT_NEAR(limits.safe_right_limit, std::max(0.0, expected_right), 0.05);
}

TEST_F(DrivableAreaUtilsTest, InsidePoint_InsufficientSpace)
{
  // Very narrow lane: width = 1.6m, vehicle_half_width = 0.9m, margin = 0.3m
  // Available space per side = 0.8m, needed = 0.9 + 0.3 + 0.05 = 1.25 → negative → 0
  auto narrow_lane = makeStraightLanelet(200, 0.0, 50.0, 0.8, -0.8);  // 1.6m wide
  const lanelet::BasicPoint2d pt(10.0, 0.0);
  LaneLimitInfo limits;
  const lanelet::ConstLanelet dummy;

  updateLaneLimitsForInsidePoint(
    narrow_lane, pt, false, false, false, dummy, dummy, vehicle_half_width_, margin_, limits);

  EXPECT_TRUE(limits.found_valid_limit);
  // Both limits should be 0 since there's not enough space
  EXPECT_DOUBLE_EQ(limits.safe_left_limit, 0.0);
  EXPECT_DOUBLE_EQ(limits.safe_right_limit, 0.0);
}

TEST_F(DrivableAreaUtilsTest, InsidePoint_WithLeftAdjacentLane)
{
  // allow_left = true, so max left shift uses left lane's far-left boundary
  const lanelet::BasicPoint2d pt(10.0, 0.0);
  LaneLimitInfo limits;
  const lanelet::ConstLanelet dummy;

  updateLaneLimitsForInsidePoint(
    center_lane_, pt, /*allow_left=*/true, /*allow_right=*/false,
    /*point_in_adjacent=*/false, left_lane_, dummy, vehicle_half_width_, margin_, limits);

  EXPECT_TRUE(limits.found_valid_limit);
  // dist_to_far_left (left_lane's left boundary at y=5.5) from pt(10, 0) ≈ 5.5
  // max_left = 5.5 - 0.9 - 0.3 - 0.05 = 4.25
  const double expected_left =
    5.5 - vehicle_half_width_ - margin_ - kLaneBoundaryDiscretizationBuffer;
  EXPECT_NEAR(limits.safe_left_limit, expected_left, 0.1);
  // Right shift still uses current lane's right boundary
  const double expected_right =
    2.0 - vehicle_half_width_ - margin_ - kLaneBoundaryDiscretizationBuffer;
  EXPECT_NEAR(limits.safe_right_limit, expected_right, 0.1);
}

TEST_F(DrivableAreaUtilsTest, InsidePoint_WithRightAdjacentLane)
{
  // allow_right = true, so max right shift uses right lane's far-right boundary
  const lanelet::BasicPoint2d pt(10.0, 0.0);
  LaneLimitInfo limits;
  const lanelet::ConstLanelet dummy;

  updateLaneLimitsForInsidePoint(
    center_lane_, pt, /*allow_left=*/false, /*allow_right=*/true,
    /*point_in_adjacent=*/false, dummy, right_lane_, vehicle_half_width_, margin_, limits);

  EXPECT_TRUE(limits.found_valid_limit);
  // Left shift uses current lane's left boundary
  const double expected_left =
    2.0 - vehicle_half_width_ - margin_ - kLaneBoundaryDiscretizationBuffer;
  EXPECT_NEAR(limits.safe_left_limit, expected_left, 0.1);
  // dist_to_far_right (right_lane's right boundary at y=-5.5) from pt(10, 0) ≈ 5.5
  const double expected_right =
    5.5 - vehicle_half_width_ - margin_ - kLaneBoundaryDiscretizationBuffer;
  EXPECT_NEAR(limits.safe_right_limit, expected_right, 0.1);
}

TEST_F(DrivableAreaUtilsTest, InsidePoint_WithBothAdjacentLanes)
{
  // Both adjacent lanes available
  const lanelet::BasicPoint2d pt(10.0, 0.0);
  LaneLimitInfo limits;

  updateLaneLimitsForInsidePoint(
    center_lane_, pt, /*allow_left=*/true, /*allow_right=*/true,
    /*point_in_adjacent=*/false, left_lane_, right_lane_, vehicle_half_width_, margin_, limits);

  EXPECT_TRUE(limits.found_valid_limit);
  const double expected_left =
    5.5 - vehicle_half_width_ - margin_ - kLaneBoundaryDiscretizationBuffer;
  const double expected_right =
    5.5 - vehicle_half_width_ - margin_ - kLaneBoundaryDiscretizationBuffer;
  EXPECT_NEAR(limits.safe_left_limit, expected_left, 0.1);
  EXPECT_NEAR(limits.safe_right_limit, expected_right, 0.1);
}

TEST_F(DrivableAreaUtilsTest, InsidePoint_InAdjacentLane_NoFurtherExpansion)
{
  // Point is already in the adjacent lane (point_in_adjacent=true)
  // → should NOT use the far boundary even if allow_left/right is true
  const lanelet::BasicPoint2d pt(10.0, 3.5);  // inside left_lane
  LaneLimitInfo limits;

  updateLaneLimitsForInsidePoint(
    left_lane_, pt, /*allow_left=*/true, /*allow_right=*/true,
    /*point_in_adjacent=*/true, left_lane_, right_lane_, vehicle_half_width_, margin_, limits);

  EXPECT_TRUE(limits.found_valid_limit);
  // Since point_in_adjacent=true, adjacent expansion is skipped.
  // Left boundary of left_lane is at y=5.5, dist_to_left = 5.5 - 3.5 = 2.0
  // Right boundary of left_lane is at y=2.0, dist_to_right = 3.5 - 2.0 = 1.5
  const double expected_left =
    2.0 - vehicle_half_width_ - margin_ - kLaneBoundaryDiscretizationBuffer;
  const double expected_right =
    1.5 - vehicle_half_width_ - margin_ - kLaneBoundaryDiscretizationBuffer;
  EXPECT_NEAR(limits.safe_left_limit, std::max(0.0, expected_left), 0.1);
  EXPECT_NEAR(limits.safe_right_limit, std::max(0.0, expected_right), 0.1);
}

TEST_F(DrivableAreaUtilsTest, InsidePoint_AccumulatesMinimum)
{
  // Multiple points should accumulate the minimum (most restrictive) limit
  LaneLimitInfo limits;
  const lanelet::ConstLanelet dummy;

  // First point: center
  const lanelet::BasicPoint2d pt1(10.0, 0.0);
  updateLaneLimitsForInsidePoint(
    center_lane_, pt1, false, false, false, dummy, dummy, vehicle_half_width_, margin_, limits);

  const double left1 = limits.safe_left_limit;
  const double right1 = limits.safe_right_limit;

  // Second point: shifted 0.5m to the left → more restrictive left limit
  const lanelet::BasicPoint2d pt2(20.0, 0.5);
  updateLaneLimitsForInsidePoint(
    center_lane_, pt2, false, false, false, dummy, dummy, vehicle_half_width_, margin_, limits);

  // Left limit should be reduced (more restrictive)
  EXPECT_LE(limits.safe_left_limit, left1);
  // Right limit should also be updated (less restrictive from this point, but min takes prev)
  EXPECT_LE(limits.safe_right_limit, right1 + 0.5 + 0.01);  // should not exceed previous
}

// ===========================================================================
// Integration-like tests combining multiple functions
// ===========================================================================
TEST_F(DrivableAreaUtilsTest, Integration_PointInsideLane_ClampLeft)
{
  // Simulate a point in the center of a 4m lane, requesting 5m left shift
  const lanelet::BasicPoint2d pt(10.0, 0.0);
  LaneLimitInfo limits;
  const lanelet::ConstLanelet dummy;

  updateLaneLimitsForInsidePoint(
    center_lane_, pt, false, false, false, dummy, dummy, vehicle_half_width_, margin_, limits);

  const double result = clampOffsetToLimits(5.0, limits.safe_left_limit, limits.safe_right_limit);
  // Should be clamped to the safe_left_limit
  EXPECT_DOUBLE_EQ(result, limits.safe_left_limit);
  EXPECT_LT(result, 5.0);
  EXPECT_GT(result, 0.0);
}

TEST_F(DrivableAreaUtilsTest, Integration_PointInsideLane_ClampRight)
{
  // Simulate requesting -5m (right) shift
  const lanelet::BasicPoint2d pt(10.0, 0.0);
  LaneLimitInfo limits;
  const lanelet::ConstLanelet dummy;

  updateLaneLimitsForInsidePoint(
    center_lane_, pt, false, false, false, dummy, dummy, vehicle_half_width_, margin_, limits);

  const double result = clampOffsetToLimits(-5.0, limits.safe_left_limit, limits.safe_right_limit);
  EXPECT_DOUBLE_EQ(result, -limits.safe_right_limit);
  EXPECT_GT(result, -5.0);
  EXPECT_LT(result, 0.0);
}

TEST_F(DrivableAreaUtilsTest, Integration_NarrowLane_BothDirectionsBlocked)
{
  // Very narrow lane where no shift is possible
  auto narrow_lane = makeStraightLanelet(300, 0.0, 50.0, 0.8, -0.8);
  const lanelet::BasicPoint2d pt(10.0, 0.0);
  LaneLimitInfo limits;
  const lanelet::ConstLanelet dummy;

  updateLaneLimitsForInsidePoint(
    narrow_lane, pt, false, false, false, dummy, dummy, vehicle_half_width_, margin_, limits);

  // Both limits should be 0
  const double left_result =
    clampOffsetToLimits(1.0, limits.safe_left_limit, limits.safe_right_limit);
  const double right_result =
    clampOffsetToLimits(-1.0, limits.safe_left_limit, limits.safe_right_limit);
  EXPECT_DOUBLE_EQ(left_result, 0.0);
  EXPECT_DOUBLE_EQ(right_result, 0.0);
}

TEST_F(DrivableAreaUtilsTest, Integration_WithAdjacentLanes_LargerShiftAllowed)
{
  // With adjacent lanes, a larger shift should be allowed
  const lanelet::BasicPoint2d pt(10.0, 0.0);

  // Without adjacent lanes
  LaneLimitInfo limits_no_adj;
  const lanelet::ConstLanelet dummy;
  updateLaneLimitsForInsidePoint(
    center_lane_, pt, false, false, false, dummy, dummy, vehicle_half_width_, margin_,
    limits_no_adj);

  // With left adjacent lane
  LaneLimitInfo limits_with_adj;
  updateLaneLimitsForInsidePoint(
    center_lane_, pt, true, false, false, left_lane_, dummy, vehicle_half_width_, margin_,
    limits_with_adj);

  // Left limit should be larger with adjacent lane
  EXPECT_GT(limits_with_adj.safe_left_limit, limits_no_adj.safe_left_limit);
  // Right limit should be the same
  EXPECT_NEAR(limits_with_adj.safe_right_limit, limits_no_adj.safe_right_limit, 0.01);
}

TEST_F(DrivableAreaUtilsTest, Integration_OutsidePointThenInsidePoint)
{
  // First point outside (left), then point inside
  LaneLimitInfo limits;
  const lanelet::ConstLanelet dummy;

  // Outside point on left (y=3.0)
  const lanelet::BasicPoint2d pt_out(10.0, 3.0);
  updateLaneLimitsForOutsidePoint(center_lane_, pt_out, vehicle_half_width_, margin_, limits);

  // Inside point at center
  const lanelet::BasicPoint2d pt_in(20.0, 0.0);
  updateLaneLimitsForInsidePoint(
    center_lane_, pt_in, false, false, false, dummy, dummy, vehicle_half_width_, margin_, limits);

  // Left is restricted by outside point → safe_left_limit = 0
  EXPECT_DOUBLE_EQ(limits.safe_left_limit, 0.0);
  // Right should still have some room
  EXPECT_GT(limits.safe_right_limit, 0.0);

  // Requesting left shift should return 0
  EXPECT_DOUBLE_EQ(clampOffsetToLimits(1.0, limits.safe_left_limit, limits.safe_right_limit), 0.0);
}

// ===========================================================================
// LaneLimitInfo default values
// ===========================================================================
TEST(LaneLimitInfoTest, DefaultValues)
{
  LaneLimitInfo info;
  EXPECT_DOUBLE_EQ(info.safe_left_limit, 100.0);
  EXPECT_DOUBLE_EQ(info.safe_right_limit, 100.0);
  EXPECT_FALSE(info.found_valid_limit);
}

// ===========================================================================
// AdjacentLaneInfo default values
// ===========================================================================
TEST(AdjacentLaneInfoTest, DefaultValues)
{
  AdjacentLaneInfo info;
  EXPECT_FALSE(info.allow_left);
  EXPECT_FALSE(info.allow_right);
  EXPECT_FALSE(info.has_left);
  EXPECT_FALSE(info.has_right);
}

// ===========================================================================
// LanePositionResult default values
// ===========================================================================
TEST(LanePositionResultTest, DefaultValues)
{
  LanePositionResult result;
  EXPECT_FALSE(result.is_in_adjacent);
  EXPECT_FALSE(result.is_outside_all);
}

// ===========================================================================
// Edge cases
// ===========================================================================
TEST_F(DrivableAreaUtilsTest, EdgeCase_PointExactlyOnBoundary)
{
  // Point exactly on the left boundary
  const lanelet::BasicPoint2d pt(10.0, 2.0);
  LaneLimitInfo limits;
  const lanelet::ConstLanelet dummy;

  // Should not crash; behavior depends on lanelet2's inside() definition
  updateLaneLimitsForInsidePoint(
    center_lane_, pt, false, false, false, dummy, dummy, vehicle_half_width_, margin_, limits);
  // Test that limits are set regardless
  EXPECT_TRUE(limits.found_valid_limit);
}

TEST_F(DrivableAreaUtilsTest, EdgeCase_VeryLargeOffset)
{
  // Extremely large requested offset should be clamped
  const double result = clampOffsetToLimits(1000.0, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(result, 1.0);

  const double result2 = clampOffsetToLimits(-1000.0, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(result2, -1.0);
}

TEST_F(DrivableAreaUtilsTest, EdgeCase_NegativeMarginValues)
{
  // Even with negative margin (which shouldn't happen in practice), function should not crash
  const lanelet::BasicPoint2d pt(10.0, 0.0);
  LaneLimitInfo limits;
  const lanelet::ConstLanelet dummy;

  EXPECT_NO_THROW(updateLaneLimitsForInsidePoint(
    center_lane_, pt, false, false, false, dummy, dummy, vehicle_half_width_, -1.0, limits));
  EXPECT_TRUE(limits.found_valid_limit);
}

TEST_F(DrivableAreaUtilsTest, EdgeCase_ZeroWidthVehicle)
{
  // Zero vehicle half width (degenerate case)
  const lanelet::BasicPoint2d pt(10.0, 0.0);
  LaneLimitInfo limits;
  const lanelet::ConstLanelet dummy;

  updateLaneLimitsForInsidePoint(
    center_lane_, pt, false, false, false, dummy, dummy, /*vehicle_half_width=*/0.0, margin_,
    limits);

  EXPECT_TRUE(limits.found_valid_limit);
  // With zero vehicle width, more shift should be allowed
  const double expected = 2.0 - 0.0 - margin_ - kLaneBoundaryDiscretizationBuffer;
  EXPECT_NEAR(limits.safe_left_limit, expected, 0.05);
  EXPECT_NEAR(limits.safe_right_limit, expected, 0.05);
}

// ===========================================================================
// kLaneBoundaryDiscretizationBuffer constant
// ===========================================================================
TEST(ConstantsTest, DiscretizationBufferIsPositive)
{
  EXPECT_GT(kLaneBoundaryDiscretizationBuffer, 0.0);
  EXPECT_DOUBLE_EQ(kLaneBoundaryDiscretizationBuffer, 0.05);
}

}  // namespace autoware::behavior_path_planner
