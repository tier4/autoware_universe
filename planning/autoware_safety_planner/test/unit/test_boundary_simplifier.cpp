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

#include "utils/boundary_simplifier.hpp"

#include <gtest/gtest.h>

namespace autoware::safety_planner
{

namespace
{
LineString2d make_jittered_line(const int count, const double jitter)
{
  LineString2d line;
  for (int i = 0; i < count; ++i) {
    line.emplace_back(static_cast<double>(i), (i % 2 == 0) ? jitter : -jitter);
  }
  return line;
}
}  // namespace

TEST(BoundarySimplifier, CollapsesVerticesWithinTolerance)
{
  BoundarySimplifier simplifier(0.05, 8);
  const auto simplified = simplifier.simplify(make_jittered_line(20, 0.01));
  EXPECT_EQ(simplified.size(), 2U);
}

TEST(BoundarySimplifier, KeepsVerticesBeyondTolerance)
{
  BoundarySimplifier simplifier(0.05, 8);
  const auto input = make_jittered_line(20, 0.5);
  EXPECT_EQ(simplifier.simplify(input).size(), input.size());
}

TEST(BoundarySimplifier, ZeroToleranceIsPassThrough)
{
  BoundarySimplifier simplifier(0.0, 8);
  const auto input = make_jittered_line(20, 0.01);
  EXPECT_EQ(simplifier.simplify(input).size(), input.size());
}

TEST(BoundarySimplifier, CacheReturnsSameResultForSameGeometry)
{
  BoundarySimplifier simplifier(0.05, 8);
  const auto input = make_jittered_line(20, 0.01);
  const auto first = simplifier.simplify(input);
  const auto second = simplifier.simplify(input);
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t i = 0; i < first.size(); ++i) {
    EXPECT_DOUBLE_EQ(first[i].x(), second[i].x());
    EXPECT_DOUBLE_EQ(first[i].y(), second[i].y());
  }
}

}  // namespace autoware::safety_planner
