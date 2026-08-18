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
