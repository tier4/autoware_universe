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

#include "../../src/filters/safety/collision_check_filter.cpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::geometry
{

class GeometryTest : public ::testing::Test
{
protected:
  Polygon2d create_rect_poly(double min_x, double min_y, double max_x, double max_y)
  {
    Polygon2d poly;
    poly.outer().push_back(Point2d(min_x, min_y));
    poly.outer().push_back(Point2d(max_x, min_y));
    poly.outer().push_back(Point2d(max_x, max_y));
    poly.outer().push_back(Point2d(min_x, max_y));
    poly.outer().push_back(Point2d(min_x, min_y));
    boost::geometry::correct(poly);
    return poly;
  }
};


}  // namespace autoware::trajectory_validator::plugin::safety::geometry
