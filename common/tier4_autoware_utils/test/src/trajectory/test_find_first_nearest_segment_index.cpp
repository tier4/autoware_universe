#include "test_first_nearest_inputs.hpp"
#include "tier4_autoware_utils/trajectory/trajectory.hpp"

#include <gtest/gtest.h>

TEST(trajectory, findFirstNearestSegmentIndex_11)
{
  using first_nearest_inputs::distance_thresh_default;
  using first_nearest_inputs::max_dist;
  using first_nearest_inputs::max_yaw;
  using first_nearest_inputs::plain_n_pose;
  using first_nearest_inputs::traj_plain;
  using tier4_autoware_utils::findFirstNearestSegmentIndex;
  using tier4_autoware_utils::findNearestSegmentIndex;

  const auto points = traj_plain();
  const auto pose = plain_n_pose();

  const auto first =
    findFirstNearestSegmentIndex(points, pose, max_dist, max_yaw, distance_thresh_default);
  ASSERT_TRUE(first);

  const auto baseline = findNearestSegmentIndex(points, pose, max_dist, max_yaw);
  ASSERT_TRUE(baseline);
  EXPECT_EQ(*first, *baseline);
}

TEST(trajectory, findFirstNearestSegmentIndex_12)
{
  using first_nearest_inputs::distance_thresh_default;
  using first_nearest_inputs::max_dist;
  using first_nearest_inputs::max_yaw;
  using first_nearest_inputs::overlap2_n_pose;
  using first_nearest_inputs::traj_overlap;
  using tier4_autoware_utils::findFirstNearestSegmentIndex;
  using tier4_autoware_utils::findNearestSegmentIndex;

  const auto points = traj_overlap();
  const auto pose = overlap2_n_pose();

  const auto first =
    findFirstNearestSegmentIndex(points, pose, max_dist, max_yaw, distance_thresh_default);
  ASSERT_TRUE(first);

  const auto baseline = findNearestSegmentIndex(points, pose, max_dist, max_yaw);
  ASSERT_TRUE(baseline);
  EXPECT_NE(*first, *baseline);

  constexpr size_t expected = 49;
  EXPECT_EQ(*first, expected);
}

TEST(trajectory, findFirstNearestSegmentIndex_13)
{
  using first_nearest_inputs::distance_thresh_small;
  using first_nearest_inputs::max_dist;
  using first_nearest_inputs::max_yaw;
  using first_nearest_inputs::plain_f_pose;
  using first_nearest_inputs::traj_plain;
  using tier4_autoware_utils::findFirstNearestSegmentIndex;
  using tier4_autoware_utils::findNearestSegmentIndex;

  const auto points = traj_plain();
  const auto pose = plain_f_pose();

  const auto first =
    findFirstNearestSegmentIndex(points, pose, max_dist, max_yaw, distance_thresh_small);
  ASSERT_TRUE(first);

  const auto baseline = findNearestSegmentIndex(points, pose, max_dist, max_yaw);
  ASSERT_TRUE(baseline);
  EXPECT_EQ(*first, *baseline);
}

TEST(trajectory, findFirstNearestSegmentIndex_14)
{
  using first_nearest_inputs::distance_thresh_small;
  using first_nearest_inputs::max_dist;
  using first_nearest_inputs::max_yaw;
  using first_nearest_inputs::overlap2_f_pose;
  using first_nearest_inputs::traj_overlap;
  using tier4_autoware_utils::findFirstNearestSegmentIndex;
  using tier4_autoware_utils::findNearestSegmentIndex;

  const auto points = traj_overlap();
  const auto pose = overlap2_f_pose();

  const auto first =
    findFirstNearestSegmentIndex(points, pose, max_dist, max_yaw, distance_thresh_small);
  ASSERT_TRUE(first);

  const auto baseline = findNearestSegmentIndex(points, pose, max_dist, max_yaw);
  ASSERT_TRUE(baseline);
  EXPECT_EQ(*first, *baseline);
}

TEST(trajectory, findFirstNearestSegmentIndex_15)
{
  using first_nearest_inputs::distance_thresh_default;
  using first_nearest_inputs::plain_n_point;
  using first_nearest_inputs::traj_plain;
  using tier4_autoware_utils::findFirstNearestSegmentIndex;
  using tier4_autoware_utils::findNearestSegmentIndex;

  const auto points = traj_plain();
  const auto point = plain_n_point();

  const auto first = findFirstNearestSegmentIndex(points, point, distance_thresh_default);
  const auto baseline = findNearestSegmentIndex(points, point);
  EXPECT_EQ(first, baseline);
}

TEST(trajectory, findFirstNearestSegmentIndex_16)
{
  using first_nearest_inputs::distance_thresh_default;
  using first_nearest_inputs::overlap2_n_point;
  using first_nearest_inputs::traj_overlap;
  using tier4_autoware_utils::findFirstNearestSegmentIndex;
  using tier4_autoware_utils::findNearestSegmentIndex;

  const auto points = traj_overlap();
  const auto point = overlap2_n_point();

  const auto first = findFirstNearestSegmentIndex(points, point, distance_thresh_default);
  const auto baseline = findNearestSegmentIndex(points, point);
  EXPECT_NE(first, baseline);

  constexpr size_t expected = 50;
  EXPECT_EQ(first, expected);
}

TEST(trajectory, findFirstNearestSegmentIndex_17)
{
  using first_nearest_inputs::distance_thresh_small;
  using first_nearest_inputs::plain_f_point;
  using first_nearest_inputs::traj_plain;
  using tier4_autoware_utils::findFirstNearestSegmentIndex;
  using tier4_autoware_utils::findNearestSegmentIndex;

  const auto points = traj_plain();
  const auto point = plain_f_point();

  const auto first = findFirstNearestSegmentIndex(points, point, distance_thresh_small);
  const auto baseline = findNearestSegmentIndex(points, point);
  EXPECT_EQ(first, baseline);
}

TEST(trajectory, findFirstNearestSegmentIndex_18)
{
  using first_nearest_inputs::distance_thresh_small;
  using first_nearest_inputs::overlap2_f_point;
  using first_nearest_inputs::traj_overlap;
  using tier4_autoware_utils::findFirstNearestSegmentIndex;
  using tier4_autoware_utils::findNearestSegmentIndex;

  const auto points = traj_overlap();
  const auto point = overlap2_f_point();

  const auto first = findFirstNearestSegmentIndex(points, point, distance_thresh_small);
  const auto baseline = findNearestSegmentIndex(points, point);
  EXPECT_EQ(first, baseline);
}
