#include "test_first_nearest_inputs.hpp"
#include "tier4_autoware_utils/trajectory/trajectory.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
constexpr double epsilon = 1e-6;
constexpr double epsilon_diff = 1e-2;
}  // namespace

TEST(trajectory, calcFirstSignedArcLength_01)
{
  using first_nearest_inputs::distance_thresh_default;
  using first_nearest_inputs::max_dist;
  using first_nearest_inputs::max_yaw;
  using first_nearest_inputs::plain_n_point;
  using first_nearest_inputs::plain_n_pose;
  using first_nearest_inputs::traj_plain;
  using tier4_autoware_utils::calcFirstSignedArcLength;
  using tier4_autoware_utils::calcSignedArcLength;

  const auto points = traj_plain();
  const auto src = plain_n_pose();
  const auto dst = plain_n_point();

  const auto first =
    calcFirstSignedArcLength(points, src, dst, max_dist, max_yaw, distance_thresh_default);
  ASSERT_TRUE(first);

  const auto baseline = calcSignedArcLength(points, src, dst, max_dist, max_yaw);
  ASSERT_TRUE(baseline);
  EXPECT_NEAR(*first, *baseline, epsilon);
}

TEST(trajectory, calcFirstSignedArcLength_02)
{
  using first_nearest_inputs::distance_thresh_default;
  using first_nearest_inputs::max_dist;
  using first_nearest_inputs::max_yaw;
  using first_nearest_inputs::overlap1_n_point;
  using first_nearest_inputs::overlap1_n_pose;
  using first_nearest_inputs::traj_overlap;
  using tier4_autoware_utils::calcFirstSignedArcLength;
  using tier4_autoware_utils::calcSignedArcLength;

  const auto points = traj_overlap();
  const auto src = overlap1_n_pose();
  const auto dst = overlap1_n_point();

  const auto first =
    calcFirstSignedArcLength(points, src, dst, max_dist, max_yaw, distance_thresh_default);
  ASSERT_TRUE(first);

  const auto baseline = calcSignedArcLength(points, src, dst, max_dist, max_yaw);
  ASSERT_TRUE(baseline);
  EXPECT_NEAR(*first, *baseline, epsilon);
}

TEST(trajectory, calcFirstSignedArcLength_03)
{
  using first_nearest_inputs::distance_thresh_default;
  using first_nearest_inputs::max_dist;
  using first_nearest_inputs::max_yaw;
  using first_nearest_inputs::overlap2_n_point;
  using first_nearest_inputs::overlap2_n_pose;
  using first_nearest_inputs::traj_overlap;
  using tier4_autoware_utils::calcFirstSignedArcLength;
  using tier4_autoware_utils::calcSignedArcLength;

  const auto points = traj_overlap();
  const auto src = overlap2_n_pose();
  const auto dst = overlap2_n_point();

  const auto first =
    calcFirstSignedArcLength(points, src, dst, max_dist, max_yaw, distance_thresh_default);
  ASSERT_TRUE(first);

  const auto baseline = calcSignedArcLength(points, src, dst, max_dist, max_yaw);
  ASSERT_TRUE(baseline);
  EXPECT_GT(std::fabs(*first - *baseline), epsilon_diff);

  constexpr double expected = 0.1;
  EXPECT_NEAR(*first, expected, epsilon);
}

TEST(trajectory, calcFirstSignedArcLength_04)
{
  using first_nearest_inputs::distance_thresh_default;
  using first_nearest_inputs::max_dist;
  using first_nearest_inputs::max_yaw;
  using first_nearest_inputs::overlap1_n_pose;
  using first_nearest_inputs::overlap2_n_point;
  using first_nearest_inputs::traj_overlap;
  using tier4_autoware_utils::calcFirstSignedArcLength;
  using tier4_autoware_utils::calcSignedArcLength;

  const auto points = traj_overlap();
  const auto src = overlap1_n_pose();
  const auto dst = overlap2_n_point();

  const auto first =
    calcFirstSignedArcLength(points, src, dst, max_dist, max_yaw, distance_thresh_default);
  ASSERT_TRUE(first);

  const auto baseline = calcSignedArcLength(points, src, dst, max_dist, max_yaw);
  ASSERT_TRUE(baseline);
  EXPECT_GT(std::fabs(*first - *baseline), epsilon_diff);

  constexpr double expected = 0.34;
  EXPECT_NEAR(*first, expected, epsilon);
}

TEST(trajectory, calcFirstSignedArcLength_05)
{
  using first_nearest_inputs::distance_thresh_default;
  using first_nearest_inputs::max_dist;
  using first_nearest_inputs::max_yaw;
  using first_nearest_inputs::overlap1_n_point;
  using first_nearest_inputs::overlap2_n_pose;
  using first_nearest_inputs::traj_overlap;
  using tier4_autoware_utils::calcFirstSignedArcLength;
  using tier4_autoware_utils::calcSignedArcLength;

  const auto points = traj_overlap();
  const auto src = overlap2_n_pose();
  const auto dst = overlap1_n_point();

  const auto first =
    calcFirstSignedArcLength(points, src, dst, max_dist, max_yaw, distance_thresh_default);
  ASSERT_TRUE(first);

  const auto baseline = calcSignedArcLength(points, src, dst, max_dist, max_yaw);
  ASSERT_TRUE(baseline);
  EXPECT_GT(std::fabs(*first - *baseline), epsilon_diff);

  constexpr double expected = 0.36;
  EXPECT_NEAR(*first, expected, epsilon);
}

TEST(trajectory, calcFirstSignedArcLength_06)
{
  using first_nearest_inputs::distance_thresh_small;
  using first_nearest_inputs::max_dist;
  using first_nearest_inputs::max_yaw;
  using first_nearest_inputs::overlap2_f_point;
  using first_nearest_inputs::overlap2_n_pose;
  using first_nearest_inputs::traj_overlap;
  using tier4_autoware_utils::calcFirstSignedArcLength;
  using tier4_autoware_utils::calcSignedArcLength;

  const auto points = traj_overlap();
  const auto src = overlap2_n_pose();
  const auto dst = overlap2_f_point();

  const auto first =
    calcFirstSignedArcLength(points, src, dst, max_dist, max_yaw, distance_thresh_small);
  ASSERT_TRUE(first);

  const auto baseline = calcSignedArcLength(points, src, dst, max_dist, max_yaw);
  ASSERT_TRUE(baseline);
  EXPECT_GT(std::fabs(*first - *baseline), epsilon_diff);

  constexpr double expected = 24.76;
  EXPECT_NEAR(*first, expected, epsilon);
}

TEST(trajectory, calcFirstSignedArcLength_07)
{
  using first_nearest_inputs::distance_thresh_small;
  using first_nearest_inputs::max_dist;
  using first_nearest_inputs::max_yaw;
  using first_nearest_inputs::overlap2_f_pose;
  using first_nearest_inputs::overlap2_n_point;
  using first_nearest_inputs::traj_overlap;
  using tier4_autoware_utils::calcFirstSignedArcLength;
  using tier4_autoware_utils::calcSignedArcLength;

  const auto points = traj_overlap();
  const auto src = overlap2_f_pose();
  const auto dst = overlap2_n_point();

  const auto first =
    calcFirstSignedArcLength(points, src, dst, max_dist, max_yaw, distance_thresh_small);
  ASSERT_TRUE(first);

  const auto baseline = calcSignedArcLength(points, src, dst, max_dist, max_yaw);
  ASSERT_TRUE(baseline);
  EXPECT_GT(std::fabs(*first - *baseline), epsilon_diff);

  constexpr double expected = -23.26;
  EXPECT_NEAR(*first, expected, epsilon);
}

TEST(trajectory, calcFirstSignedArcLength_08)
{
  using first_nearest_inputs::distance_thresh_small;
  using first_nearest_inputs::max_dist;
  using first_nearest_inputs::max_yaw;
  using first_nearest_inputs::plain_f_point;
  using first_nearest_inputs::plain_n_pose;
  using first_nearest_inputs::traj_plain;
  using tier4_autoware_utils::calcFirstSignedArcLength;
  using tier4_autoware_utils::calcSignedArcLength;

  const auto points = traj_plain();
  const auto src = plain_n_pose();
  const auto dst = plain_f_point();

  const auto first =
    calcFirstSignedArcLength(points, src, dst, max_dist, max_yaw, distance_thresh_small);
  ASSERT_TRUE(first);

  const auto baseline = calcSignedArcLength(points, src, dst, max_dist, max_yaw);
  ASSERT_TRUE(baseline);
  EXPECT_NEAR(*first, *baseline, epsilon);
}

TEST(trajectory, calcFirstSignedArcLength_09)
{
  using first_nearest_inputs::distance_thresh_small;
  using first_nearest_inputs::max_dist;
  using first_nearest_inputs::max_yaw;
  using first_nearest_inputs::plain_f_pose;
  using first_nearest_inputs::plain_n_point;
  using first_nearest_inputs::traj_plain;
  using tier4_autoware_utils::calcFirstSignedArcLength;
  using tier4_autoware_utils::calcSignedArcLength;

  const auto points = traj_plain();
  const auto src = plain_f_pose();
  const auto dst = plain_n_point();

  const auto first =
    calcFirstSignedArcLength(points, src, dst, max_dist, max_yaw, distance_thresh_small);
  ASSERT_TRUE(first);

  const auto baseline = calcSignedArcLength(points, src, dst, max_dist, max_yaw);
  ASSERT_TRUE(baseline);
  EXPECT_NEAR(*first, *baseline, epsilon);
}

TEST(trajectory, calcFirstSignedArcLength_10)
{
  using first_nearest_inputs::distance_thresh_small;
  using first_nearest_inputs::max_dist;
  using first_nearest_inputs::max_yaw;
  using first_nearest_inputs::overlap2_f_point;
  using first_nearest_inputs::overlap2_f_pose;
  using first_nearest_inputs::traj_overlap;
  using tier4_autoware_utils::calcFirstSignedArcLength;
  using tier4_autoware_utils::calcSignedArcLength;

  const auto points = traj_overlap();
  const auto src = overlap2_f_pose();
  const auto dst = overlap2_f_point();

  const auto first =
    calcFirstSignedArcLength(points, src, dst, max_dist, max_yaw, distance_thresh_small);
  ASSERT_TRUE(first);

  const auto baseline = calcSignedArcLength(points, src, dst, max_dist, max_yaw);
  ASSERT_TRUE(baseline);
  EXPECT_NEAR(*first, *baseline, epsilon);
}

namespace
{
class SegmentIndexedArcLength : public ::testing::Test
{
protected:
  using Destination = tier4_autoware_utils::SegmentIndexWithPoint;
  const first_nearest_inputs::TrajectoryPoints points{
    first_nearest_inputs::makeTrajPoint(0.0, 0.0, 0.0),
    first_nearest_inputs::makeTrajPoint(10.0, 0.0, 0.0),
    first_nearest_inputs::makeTrajPoint(20.0, 0.0, 0.0)};
};
}  // namespace

TEST_F(SegmentIndexedArcLength, ForwardWithOffsets)
{
  const auto result = tier4_autoware_utils::calcFirstSignedArcLength(
    points, first_nearest_inputs::makePose(2.5, 0.0, 0.0),
    Destination{1, first_nearest_inputs::makePoint(17.5, 0.0)});
  ASSERT_TRUE(result);
  EXPECT_NEAR(*result, 15.0, epsilon);
}

TEST_F(SegmentIndexedArcLength, BackwardWithOffsets)
{
  const auto result = tier4_autoware_utils::calcFirstSignedArcLength(
    points, first_nearest_inputs::makePose(17.5, 0.0, 0.0),
    Destination{0, first_nearest_inputs::makePoint(2.5, 0.0)});
  ASSERT_TRUE(result);
  EXPECT_NEAR(*result, -15.0, epsilon);
}

TEST_F(SegmentIndexedArcLength, SameSegment)
{
  const auto result = tier4_autoware_utils::calcFirstSignedArcLength(
    points, first_nearest_inputs::makePose(2.5, 0.0, 0.0),
    Destination{0, first_nearest_inputs::makePoint(7.5, 0.0)});
  ASSERT_TRUE(result);
  EXPECT_NEAR(*result, 5.0, epsilon);
}

TEST_F(SegmentIndexedArcLength, CoincidentAtSegmentBoundary)
{
  const auto result = tier4_autoware_utils::calcFirstSignedArcLength(
    points, first_nearest_inputs::makePose(10.0, 0.0, 0.0),
    Destination{1, first_nearest_inputs::makePoint(10.0, 0.0)});
  ASSERT_TRUE(result);
  EXPECT_NEAR(*result, 0.0, epsilon);
}

TEST_F(SegmentIndexedArcLength, LateralOffsetsDoNotAddLength)
{
  const auto result = tier4_autoware_utils::calcFirstSignedArcLength(
    points, first_nearest_inputs::makePose(2.5, 1.0, 0.0),
    Destination{1, first_nearest_inputs::makePoint(17.5, -2.0)});
  ASSERT_TRUE(result);
  EXPECT_NEAR(*result, 15.0, epsilon);
}

TEST_F(SegmentIndexedArcLength, EmptyPath)
{
  const first_nearest_inputs::TrajectoryPoints empty;
  EXPECT_FALSE(tier4_autoware_utils::calcFirstSignedArcLength(
    empty, first_nearest_inputs::makePose(0.0, 0.0, 0.0),
    Destination{0, first_nearest_inputs::makePoint(0.0, 0.0)}));
}

TEST_F(SegmentIndexedArcLength, SourceOutsideDistanceLimit)
{
  EXPECT_FALSE(tier4_autoware_utils::calcFirstSignedArcLength(
    points, first_nearest_inputs::makePose(0.0, 2.0, 0.0),
    Destination{1, first_nearest_inputs::makePoint(17.5, 0.0)}, 1.0));
}

TEST_F(SegmentIndexedArcLength, SourceOutsideYawLimit)
{
  EXPECT_FALSE(tier4_autoware_utils::calcFirstSignedArcLength(
    points, first_nearest_inputs::makePose(0.0, 0.0, tier4_autoware_utils::deg2rad(90.0)),
    Destination{1, first_nearest_inputs::makePoint(17.5, 0.0)}, 10.0,
    tier4_autoware_utils::deg2rad(30.0)));
}

TEST_F(SegmentIndexedArcLength, KnownDestinationAtCrossing)
{
  // (5, 0) occurs at arc lengths 5 m and 29 m. The collision is on the later pass.
  const auto crossing = first_nearest_inputs::traj_overlap();
  const auto src = first_nearest_inputs::makePose(4.0, 0.0, 0.0);
  const auto dst = first_nearest_inputs::makePoint(5.0, 0.0);
  const auto result =
    tier4_autoware_utils::calcFirstSignedArcLength(crossing, src, Destination{289, dst});
  ASSERT_TRUE(result);
  EXPECT_NEAR(*result, 25.0, epsilon);

  const auto searched = tier4_autoware_utils::calcFirstSignedArcLength(crossing, src, dst);
  ASSERT_TRUE(searched);
  EXPECT_NEAR(*searched, 1.0, epsilon);
}

TEST_F(SegmentIndexedArcLength, SourceSearchRespectsDistanceThreshold)
{
  const auto crossing = first_nearest_inputs::traj_overlap();
  const auto src = first_nearest_inputs::overlap2_n_pose();
  const Destination dst{292, first_nearest_inputs::overlap2_n_point()};
  // A wide search includes the closer later pass; a narrow window picks the first pass.
  const auto wide = tier4_autoware_utils::calcFirstSignedArcLength(
    crossing, src, dst, first_nearest_inputs::max_dist, first_nearest_inputs::max_yaw, 20.0);
  const auto narrow = tier4_autoware_utils::calcFirstSignedArcLength(
    crossing, src, dst, first_nearest_inputs::max_dist, first_nearest_inputs::max_yaw,
    first_nearest_inputs::distance_thresh_small);
  ASSERT_TRUE(wide);
  ASSERT_TRUE(narrow);
  EXPECT_NEAR(*wide, 0.7, epsilon);
  EXPECT_NEAR(*narrow, 24.36, epsilon);
}
