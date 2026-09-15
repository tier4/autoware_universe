// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "autoware/mppi_optimizer/preferred_lane_centerline.hpp"

#include <gtest/gtest.h>

namespace autoware::mppi_optimizer
{
namespace
{
using Selector = PreferredLaneCenterlineSelector;
Selector::Lane lane(double x0, double x1, double y, double z = 0)
{
  return {
    {{x0, y, z}, {x1, y, z}},
    {{x0, y - 1.5, z}, {x1, y - 1.5, z}, {x1, y + 1.5, z}, {x0, y + 1.5, z}}};
}
Selector::Section section(double x0, double x1, double y = 0, double z = 0)
{
  const auto l = lane(x0, x1, y, z);
  return {{l}, l.centerline};
}

TEST(PreferredLaneCenterline, PreferenceIsIndependentOfEgoLane)
{
  Selector selector;
  auto s = section(0, 20);
  s.lanes.push_back(lane(0, 20, 3));
  selector.reset({s});
  auto result = selector.select({5, 3, 0}, 0, 10);
  ASSERT_EQ(result.status, "active");
  ASSERT_EQ(result.segments.size(), 1U);
  EXPECT_FLOAT_EQ(result.segments[0].y0, 0);
  const auto revision = result.revision;
  s.preferred = s.lanes[1].centerline;
  selector.reset({s});
  result = selector.select({5, 3, 0}, 0, 10);
  ASSERT_EQ(result.status, "active");
  EXPECT_GT(result.revision, revision);
  EXPECT_FLOAT_EQ(result.segments[0].y0, 3);
}

TEST(PreferredLaneCenterline, DoesNotConnectLaneletEndpoints)
{
  Selector selector;
  selector.reset({section(0, 10), section(10, 20, 3)});
  const auto result = selector.select({5, 0, 0}, 0, 20);
  ASSERT_EQ(result.status, "active");
  ASSERT_EQ(result.segments.size(), 2U);
  EXPECT_FLOAT_EQ(result.segments[0].y1, 0);
  EXPECT_FLOAT_EQ(result.segments[1].y0, 3);
}

TEST(PreferredLaneCenterline, ElevationAndHeadingDisambiguateCrossings)
{
  Selector selector;
  selector.reset({section(0, 20), section(50, 60), section(0, 20, 0, 5)});
  const auto result = selector.select({5, 0, 5}, 0, 5, 0);
  ASSERT_EQ(result.status, "active");
  ASSERT_EQ(result.segments.size(), 1U);
  EXPECT_EQ(selector.select({5, 0, 5}, 3.141592653589793, 5).status, "ego_not_associated");
}

TEST(PreferredLaneCenterline, AmbiguousRepeatedOccurrenceNeedsHistory)
{
  Selector selector;
  selector.reset({section(0, 20), section(20, 30), section(0, 20)});
  EXPECT_EQ(selector.select({5, 0, 0}, 0, 5).status, "ambiguous_route");
  ASSERT_EQ(selector.select({25, 0, 0}, 0, 5).status, "active");
  // Both occurrences are equally adjacent to section 1; history cannot resolve this ambiguity.
  EXPECT_EQ(selector.select({5, 0, 0}, 0, 5).status, "ambiguous_route");
}

TEST(PreferredLaneCenterline, InvalidMissingAndOverflowGeometryAreExplicit)
{
  Selector selector;
  auto s = section(0, 300);
  s.preferred.clear();
  selector.reset({s});
  EXPECT_EQ(selector.select({5, 0, 0}, 0, 10).status, "invalid_geometry");
  s.preferred = {{0, 0, 0}, {0, 0, 0}, {300, 0, 0}};
  selector.reset({s});
  EXPECT_EQ(selector.select({5, 0, 0}, 0, 10).segments.size(), 1U);
  s.preferred[1].x = NAN;
  selector.reset({s});
  EXPECT_EQ(selector.select({5, 0, 0}, 0, 10).status, "invalid_geometry");
  s.preferred.clear();
  for (int i = 0; i <= 256; ++i) s.preferred.push_back({static_cast<double>(i), 0, 0});
  selector.reset({s});
  EXPECT_EQ(selector.select({5, 0, 0}, 0, 300).segments.size(), 256U);
  s.preferred.push_back({257, 0, 0});
  selector.reset({s});
  const auto overflow = selector.select({5, 0, 0}, 0, 300);
  EXPECT_EQ(overflow.status, "overflow");
  EXPECT_TRUE(overflow.segments.empty());
  selector.reset({});
  EXPECT_EQ(selector.select({5, 0, 0}, 0, 10).status, "unavailable");
}

TEST(PreferredLaneCenterline, LocalWindowRetainsSegmentsAtBothEnds)
{
  Selector selector;
  selector.reset({section(0, 10), section(10, 20), section(20, 30), section(30, 40)});
  const auto result = selector.select({15, 0, 0}, 0, 12, 6);
  ASSERT_EQ(result.status, "active");
  ASSERT_EQ(result.segments.size(), 3U);
  EXPECT_FLOAT_EQ(result.segments.front().x0, 0);
  EXPECT_FLOAT_EQ(result.segments.back().x1, 30);
}
}  // namespace
}  // namespace autoware::mppi_optimizer
