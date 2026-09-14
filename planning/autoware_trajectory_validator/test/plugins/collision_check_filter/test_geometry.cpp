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

#include "../../../src/filters/safety/collision_check_filter/trajectory_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <string_view>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::geometry
{
namespace
{

Polygon2d create_polygon(const std::vector<Point2d> & vertices)
{
  Polygon2d poly;
  poly.outer().reserve(vertices.size() + 1);
  for (const auto & vertex : vertices) {
    poly.outer().push_back(vertex);
  }
  poly.outer().push_back(vertices.front());
  boost::geometry::correct(poly);
  return poly;
}

// Reproduces how `to_polygon2d()` builds an object ring: the vertices are copied verbatim and the
// ring is closed only when it is not empty. A perception POLYGON footprint is therefore not padded,
// so zero vertices stay an empty ring, one vertex becomes [p, p] and two vertices become
// [p0, p1, p0]. `create_polygon()` above cannot express those cases because it dereferences
// `front()` unconditionally and runs `boost::geometry::correct()`.
Polygon2d create_closed_ring(const std::vector<Point2d> & vertices)
{
  Polygon2d poly;
  poly.outer().reserve(vertices.size() + 1);
  for (const auto & vertex : vertices) {
    poly.outer().push_back(vertex);
  }
  if (!poly.outer().empty()) {
    poly.outer().push_back(poly.outer().front());
  }
  return poly;
}

Polygon2d create_rect_poly(
  const double min_x, const double min_y, const double max_x, const double max_y)
{
  return create_polygon(
    {Point2d(min_x, min_y), Point2d(max_x, min_y), Point2d(max_x, max_y), Point2d(min_x, max_y)});
}

Point2d rotate_and_translate(
  const double local_x, const double local_y, const double center_x, const double center_y,
  const double yaw)
{
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  return Point2d(
    center_x + cos_yaw * local_x - sin_yaw * local_y,
    center_y + sin_yaw * local_x + cos_yaw * local_y);
}

Polygon2d create_oriented_box(
  const double center_x, const double center_y, const double length, const double width,
  const double yaw)
{
  const double half_length = length * 0.5;
  const double half_width = width * 0.5;
  return create_polygon(
    {rotate_and_translate(half_length, half_width, center_x, center_y, yaw),
     rotate_and_translate(half_length, -half_width, center_x, center_y, yaw),
     rotate_and_translate(-half_length, -half_width, center_x, center_y, yaw),
     rotate_and_translate(-half_length, half_width, center_x, center_y, yaw)});
}

Polygon2d create_regular_polygon(
  const double center_x, const double center_y, const double radius, const size_t vertex_count,
  const double yaw)
{
  std::vector<Point2d> vertices;
  vertices.reserve(vertex_count);
  for (size_t i = 0; i < vertex_count; ++i) {
    const double theta =
      yaw + 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(vertex_count);
    vertices.emplace_back(center_x + radius * std::cos(theta), center_y + radius * std::sin(theta));
  }
  return create_polygon(vertices);
}

std::string make_xy_case_name(const std::string_view prefix, const double dx, const double dy)
{
  return std::string(prefix) + " dx=" + std::to_string(dx) + ", dy=" + std::to_string(dy);
}

std::string make_delta_case_name(const std::string_view prefix, const double delta)
{
  return std::string(prefix) + " delta=" + std::to_string(delta);
}

void expect_both_outcomes_covered(
  const std::string_view case_group_name, const bool saw_true_case, const bool saw_false_case)
{
  EXPECT_TRUE(saw_true_case) << case_group_name << " did not include an intersecting case";
  EXPECT_TRUE(saw_false_case) << case_group_name << " did not include a separating case";
}

bool expect_matches_boost_intersects(
  const std::string & case_name, const Polygon2d & poly_a, const Polygon2d & poly_b)
{
  SCOPED_TRACE(case_name);

  const bool boost_result = boost::geometry::intersects(poly_a, poly_b);
  const bool sat_result = intersects_sat(poly_a, poly_b);

  EXPECT_EQ(sat_result, boost_result);
  return sat_result;
}

TEST(GeometryTest, IntersectsSatMatchesBoostForAxisAlignedPolygons)
{
  const auto base_poly = create_rect_poly(0.0, 0.0, 2.0, 2.0);
  bool saw_intersection = false;
  bool saw_separation = false;

  for (int x_step = -4; x_step <= 12; ++x_step) {
    for (int y_step = -4; y_step <= 12; ++y_step) {
      const double dx = 0.25 * static_cast<double>(x_step);
      const double dy = 0.25 * static_cast<double>(y_step);
      const bool intersects = expect_matches_boost_intersects(
        make_xy_case_name("axis-aligned shift", dx, dy), base_poly,
        create_rect_poly(dx, dy, dx + 2.0, dy + 2.0));
      saw_intersection = saw_intersection || intersects;
      saw_separation = saw_separation || !intersects;
    }
  }

  expect_both_outcomes_covered("axis-aligned shifts", saw_intersection, saw_separation);
}

TEST(GeometryTest, IntersectsSatMatchesBoostForRotatedPolygons)
{
  const auto base_poly = create_oriented_box(0.0, 0.0, 4.0, 1.6, M_PI / 6.0);
  bool saw_intersection = false;
  bool saw_separation = false;

  for (int x_step = -12; x_step <= 12; ++x_step) {
    for (int y_step = -8; y_step <= 8; ++y_step) {
      const double dx = 0.35 * static_cast<double>(x_step);
      const double dy = 0.25 * static_cast<double>(y_step);
      const bool intersects = expect_matches_boost_intersects(
        make_xy_case_name("rotated shift", dx, dy), base_poly,
        create_oriented_box(0.7 + dx, 0.2 + dy, 2.8, 1.4, -M_PI / 5.0));
      saw_intersection = saw_intersection || intersects;
      saw_separation = saw_separation || !intersects;
    }
  }

  expect_both_outcomes_covered("rotated shifts", saw_intersection, saw_separation);
}

// SAT is only defined here for convex polygons, so these tests intentionally use convex inputs.
TEST(GeometryTest, IntersectsSatMatchesBoostForConvexPolygonsWithoutMatchingEdgeSlopes)
{
  const auto base_poly = create_regular_polygon(0.0, 0.0, 2.0, 3U, 0.2);
  bool saw_intersection = false;
  bool saw_separation = false;

  for (int x_step = -12; x_step <= 12; ++x_step) {
    for (int y_step = -8; y_step <= 8; ++y_step) {
      const double dx = 0.3 * static_cast<double>(x_step);
      const double dy = 0.25 * static_cast<double>(y_step);
      const bool intersects = expect_matches_boost_intersects(
        make_xy_case_name("different edge slopes shift", dx, dy), base_poly,
        create_regular_polygon(0.25 + dx, -0.1 + dy, 1.2, 5U, -0.35));
      saw_intersection = saw_intersection || intersects;
      saw_separation = saw_separation || !intersects;
    }
  }

  expect_both_outcomes_covered("different edge slopes shifts", saw_intersection, saw_separation);
}

TEST(GeometryTest, IntersectsSatMatchesBoostForEdgeContactAndGap)
{
  constexpr double epsilon = 1e-6;
  const auto base_poly = create_rect_poly(0.0, 0.0, 1.0, 1.0);
  bool saw_intersection = false;
  bool saw_separation = false;

  for (int step = -4; step <= 4; ++step) {
    const double delta = epsilon * static_cast<double>(step);
    const bool intersects = expect_matches_boost_intersects(
      make_delta_case_name("edge boundary shift", delta), base_poly,
      create_rect_poly(1.0 + delta, 0.0, 2.0 + delta, 1.0));
    saw_intersection = saw_intersection || intersects;
    saw_separation = saw_separation || !intersects;
  }

  expect_both_outcomes_covered("edge boundary shifts", saw_intersection, saw_separation);
}

TEST(GeometryTest, IntersectsSatMatchesBoostForPointContactAndNearPointCases)
{
  constexpr double epsilon = 1e-6;
  const auto base_poly = create_rect_poly(0.0, 0.0, 1.0, 1.0);
  bool saw_intersection = false;
  bool saw_separation = false;

  for (int step = -4; step <= 4; ++step) {
    const double delta = epsilon * static_cast<double>(step);
    const bool intersects = expect_matches_boost_intersects(
      make_delta_case_name("point boundary shift", delta), base_poly,
      create_rect_poly(1.0 + delta, 1.0 + delta, 2.0 + delta, 2.0 + delta));
    saw_intersection = saw_intersection || intersects;
    saw_separation = saw_separation || !intersects;
  }

  expect_both_outcomes_covered("point boundary shifts", saw_intersection, saw_separation);
}

// A degenerate ring is not a valid boost polygon, so the reference value is taken on the
// equivalent point / segment geometry instead of on the ring itself.
bool expect_degenerate_matches_boost(
  const std::string & case_name, const Polygon2d & full_dimensional_poly,
  const std::vector<Point2d> & degenerate_footprint)
{
  SCOPED_TRACE(case_name);

  const bool reference =
    degenerate_footprint.size() == 1U
      ? boost::geometry::intersects(full_dimensional_poly, degenerate_footprint.front())
      : boost::geometry::intersects(
          full_dimensional_poly, autoware_utils_geometry::Segment2d{
                                   degenerate_footprint.at(0), degenerate_footprint.at(1)});

  const auto degenerate_poly = create_closed_ring(degenerate_footprint);
  const bool sat_result = intersects_sat(full_dimensional_poly, degenerate_poly);
  EXPECT_EQ(sat_result, reference);
  // The predicate must not depend on the argument order.
  EXPECT_EQ(intersects_sat(degenerate_poly, full_dimensional_poly), sat_result);

  return sat_result;
}

// An empty ring is the empty set: it intersects nothing. It must also be rejected before
// has_separating_axis() runs, because that function would increment end() on an empty ring.
TEST(GeometryTest, IntersectsSatTreatsEmptyRingAsNoIntersection)
{
  const auto rect_poly = create_rect_poly(-2.0, -1.0, 2.0, 1.0);
  const auto empty_poly = create_closed_ring({});
  const auto point_poly = create_closed_ring({Point2d(0.0, 0.0)});
  const auto segment_poly = create_closed_ring({Point2d(-5.0, 0.0), Point2d(5.0, 0.0)});

  ASSERT_TRUE(empty_poly.outer().empty());

  // The non-empty operand overlaps the origin in every case below, so a "no intersection" result
  // can only come from the empty guard itself.
  EXPECT_FALSE(intersects_sat(rect_poly, empty_poly));
  EXPECT_FALSE(intersects_sat(empty_poly, rect_poly));
  EXPECT_FALSE(intersects_sat(point_poly, empty_poly));
  EXPECT_FALSE(intersects_sat(empty_poly, point_poly));
  EXPECT_FALSE(intersects_sat(segment_poly, empty_poly));
  EXPECT_FALSE(intersects_sat(empty_poly, segment_poly));
  EXPECT_FALSE(intersects_sat(empty_poly, empty_poly));
}

// Pins the guard boundary at "empty" rather than at "fewer than three vertices". A perception
// POLYGON footprint with one or two vertices reaches intersects_sat() as a two- or three-element
// ring; those are valid convex sets and must be evaluated, not rejected. Rejecting them would make
// an object that overlaps the ego footprint report "no collision", which is the fail-open this
// guard exists to avoid. The other operand is a rectangle because SAT over edge normals is only
// exhaustive while at least one operand has non-zero area; every production call site passes the
// ego footprint, which always satisfies that.
TEST(GeometryTest, IntersectsSatMatchesBoostForRingsAdmittedByTheEmptyGuard)
{
  const auto rect_poly = create_rect_poly(-2.0, -1.0, 2.0, 1.0);

  struct DegenerateCase
  {
    std::string name;
    std::vector<Point2d> footprint;
  };

  const std::vector<DegenerateCase> cases = {
    {"single vertex inside", {Point2d(0.0, 0.0)}},
    {"single vertex on edge", {Point2d(2.0, 0.0)}},
    {"single vertex just outside", {Point2d(2.0 + 1e-6, 0.0)}},
    {"single vertex far outside", {Point2d(9.0, 9.0)}},
    {"two vertices crossing", {Point2d(-5.0, 0.0), Point2d(5.0, 0.0)}},
    {"two vertices touching a corner", {Point2d(2.0, 1.0), Point2d(6.0, 5.0)}},
    {"two vertices collinear but outside", {Point2d(5.0, 0.0), Point2d(9.0, 0.0)}},
    {"two vertices diagonally outside", {Point2d(5.0, 5.0), Point2d(6.0, 6.0)}},
    {"two identical vertices inside", {Point2d(0.0, 0.0), Point2d(0.0, 0.0)}},
  };

  bool saw_intersection = false;
  bool saw_separation = false;
  for (const auto & degenerate_case : cases) {
    const auto ring_size = create_closed_ring(degenerate_case.footprint).outer().size();
    EXPECT_LT(ring_size, 4U) << degenerate_case.name << " is not a degenerate ring";

    const bool intersects =
      expect_degenerate_matches_boost(degenerate_case.name, rect_poly, degenerate_case.footprint);
    saw_intersection = saw_intersection || intersects;
    saw_separation = saw_separation || !intersects;
  }

  expect_both_outcomes_covered("degenerate footprints", saw_intersection, saw_separation);
}

TEST(TargetShapeTypeParamsTest, SupportsConfiguredShapeTypes)
{
  using autoware_perception_msgs::msg::Shape;

  Shape bbox;
  bbox.type = Shape::BOUNDING_BOX;

  Shape polygon;
  polygon.type = Shape::POLYGON;

  Shape cylinder;
  cylinder.type = Shape::CYLINDER;

  const TargetShapeTypeParams target_shape_types({"bbox", "polygon"});
  EXPECT_TRUE(target_shape_types.bbox);
  EXPECT_TRUE(target_shape_types.polygon);
  EXPECT_TRUE(target_shape_types.contains(bbox.type));
  EXPECT_TRUE(target_shape_types.contains(polygon.type));
  EXPECT_FALSE(target_shape_types.contains(cylinder.type));
}

TEST(TargetShapeTypeParamsTest, EmptyConfigurationDisablesAllShapeTypes)
{
  using autoware_perception_msgs::msg::Shape;

  Shape bbox;
  bbox.type = Shape::BOUNDING_BOX;

  const TargetShapeTypeParams target_shape_types(std::vector<std::string>{});
  EXPECT_FALSE(target_shape_types.bbox);
  EXPECT_FALSE(target_shape_types.polygon);
  EXPECT_FALSE(target_shape_types.contains(bbox.type));
}

TEST(TargetShapeTypeParamsTest, EmptyStringConfigurationDisablesAllShapeTypes)
{
  using autoware_perception_msgs::msg::Shape;

  Shape bbox;
  bbox.type = Shape::BOUNDING_BOX;

  const TargetShapeTypeParams target_shape_types({""});
  EXPECT_FALSE(target_shape_types.bbox);
  EXPECT_FALSE(target_shape_types.polygon);
  EXPECT_FALSE(target_shape_types.contains(bbox.type));
}

TEST(TargetShapeTypeParamsTest, RejectsUnsupportedShapeTypes)
{
  EXPECT_THROW(TargetShapeTypeParams({"cylinder"}), std::invalid_argument);
  EXPECT_THROW(TargetShapeTypeParams({"", "bbox"}), std::invalid_argument);
}

}  // namespace
}  // namespace autoware::trajectory_validator::plugin::safety::geometry
