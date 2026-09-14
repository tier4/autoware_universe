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

#include <geometry_msgs/msg/point32.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
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

autoware_perception_msgs::msg::Shape create_shape(
  const uint8_t type, const double dimension_x, const double dimension_y,
  const std::vector<Point2d> & footprint = {})
{
  autoware_perception_msgs::msg::Shape shape;
  shape.type = type;
  shape.dimensions.x = dimension_x;
  shape.dimensions.y = dimension_y;
  shape.dimensions.z = 1.5;
  shape.footprint.points.reserve(footprint.size());
  for (const auto & vertex : footprint) {
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<float>(vertex.x());
    point.y = static_cast<float>(vertex.y());
    point.z = 0.0F;
    shape.footprint.points.push_back(point);
  }
  return shape;
}

std::vector<Point2d> bounding_box_outline(const double length, const double width)
{
  const double half_length = length * 0.5;
  const double half_width = width * 0.5;
  return {
    Point2d(half_length, half_width), Point2d(half_length, -half_width),
    Point2d(-half_length, -half_width), Point2d(-half_length, half_width)};
}

void expect_same_vertices(
  const std::vector<Point2d> & actual, const std::vector<Point2d> & expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (size_t i = 0; i < actual.size(); ++i) {
    EXPECT_DOUBLE_EQ(actual.at(i).x(), expected.at(i).x()) << "vertex " << i;
    EXPECT_DOUBLE_EQ(actual.at(i).y(), expected.at(i).y()) << "vertex " << i;
  }
}

// `create_base_polygon()` returns an open ring whose first vertex is not repeated at the end, which
// `to_polygon2d()` and `compute_footprint_trajectory()` both rely on.
void expect_open_ring(const std::vector<Point2d> & ring)
{
  ASSERT_FALSE(ring.empty());
  if (ring.size() < 2U) {
    return;
  }
  EXPECT_FALSE(boost::geometry::equals(ring.front(), ring.back()))
    << "the ring repeats its first vertex at the end";
}

void expect_convex(const Polygon2d & polygon)
{
  Polygon2d hull;
  boost::geometry::convex_hull(polygon, hull);
  EXPECT_TRUE(boost::geometry::equals(polygon, hull)) << "the polygon is not convex";
}

void expect_covers_all(const Polygon2d & polygon, const std::vector<Point2d> & points)
{
  for (const auto & point : points) {
    EXPECT_TRUE(boost::geometry::covered_by(point, polygon))
      << "(" << point.x() << ", " << point.y() << ") is outside the polygon";
  }
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

// The DRAC path passes use_extra_polygon = false, so this is the guard that keeps it untouched: the
// vertices have to stay identical down to their order, which is what preserves the four-vertex fast
// path of compute_footprint_trajectory().
TEST(CreateBasePolygonTest, KeepsPrimitiveOutlineWhenExtraPolygonIsDisabled)
{
  using autoware_perception_msgs::msg::Shape;

  const std::vector<Point2d> extra_footprint = {Point2d(3.0, 0.0), Point2d(0.0, 2.0)};
  const auto bbox = create_shape(Shape::BOUNDING_BOX, 4.0, 2.0, extra_footprint);
  const auto cylinder = create_shape(Shape::CYLINDER, 2.0, 2.0, extra_footprint);

  expect_same_vertices(create_base_polygon(bbox, false), bounding_box_outline(4.0, 2.0));
  expect_same_vertices(create_base_polygon(cylinder, false), bounding_box_outline(2.0, 2.0));
}

// An object that carries no extra footprint keeps the polygon it had: the convex hull of the
// primitive outline is that outline again, only starting at another vertex.
TEST(CreateBasePolygonTest, ReproducesThePrimitiveOutlineWhenExtraFootprintIsEmpty)
{
  using autoware_perception_msgs::msg::Shape;

  for (const auto & shape :
       {create_shape(Shape::BOUNDING_BOX, 4.0, 2.0), create_shape(Shape::CYLINDER, 2.0, 2.0)}) {
    const auto outline = create_base_polygon(shape, false);
    const auto polygon = create_base_polygon(shape, true);

    EXPECT_EQ(polygon.size(), outline.size());
    expect_open_ring(polygon);
    EXPECT_TRUE(boost::geometry::equals(create_closed_ring(polygon), create_polygon(outline)));
  }
}

TEST(CreateBasePolygonTest, MergesExtraFootprintOfBoundingBoxIntoConvexHull)
{
  using autoware_perception_msgs::msg::Shape;

  const std::vector<Point2d> extra_footprint = {
    Point2d(3.0, 0.0), Point2d(2.0, 1.0), Point2d(-1.0, 0.5)};
  const auto shape = create_shape(Shape::BOUNDING_BOX, 4.0, 2.0, extra_footprint);

  const auto polygon = create_base_polygon(shape, true);

  expect_open_ring(polygon);
  const auto hull = create_closed_ring(polygon);
  expect_convex(hull);
  expect_covers_all(hull, extra_footprint);
  expect_covers_all(hull, bounding_box_outline(4.0, 2.0));
  // The bounding box alone spans 4.0 x 2.0, and (3.0, 0.0) sticks out of it.
  EXPECT_GT(std::abs(boost::geometry::area(hull)), 8.0);
}

TEST(CreateBasePolygonTest, KeepsBoundingBoxWhenExtraFootprintStaysInside)
{
  using autoware_perception_msgs::msg::Shape;

  const auto shape = create_shape(
    Shape::BOUNDING_BOX, 4.0, 2.0, {Point2d(0.5, 0.5), Point2d(-1.0, 0.0), Point2d(2.0, 1.0)});

  const auto polygon = create_base_polygon(shape, true);

  // The hull may start at another vertex than the primitive outline does, so the two are compared
  // as polygons rather than vertex by vertex.
  ASSERT_EQ(polygon.size(), 4U);
  expect_open_ring(polygon);
  EXPECT_TRUE(
    boost::geometry::equals(
      create_closed_ring(polygon), create_polygon(bounding_box_outline(4.0, 2.0))));
}

TEST(CreateBasePolygonTest, MergesExtraFootprintOfCylinderIntoConvexHull)
{
  using autoware_perception_msgs::msg::Shape;

  const std::vector<Point2d> extra_footprint = {Point2d(3.0, 0.0)};
  const auto shape = create_shape(Shape::CYLINDER, 2.0, 2.0, extra_footprint);

  const auto polygon = create_base_polygon(shape, true);

  expect_open_ring(polygon);
  const auto hull = create_closed_ring(polygon);
  expect_convex(hull);
  expect_covers_all(hull, extra_footprint);
  // The cylinder is approximated by its circumscribed square, which spans 2.0 x 2.0.
  expect_covers_all(hull, bounding_box_outline(2.0, 2.0));
  EXPECT_GT(std::abs(boost::geometry::area(hull)), 4.0);
}

// A POLYGON object is outlined by its perception footprint alone, which upstream already delivers
// convex. There is therefore nothing to merge in and nothing to make convex, and the footprint is
// handed over untouched whichever way the flag is set.
TEST(CreateBasePolygonTest, KeepsPerceptionPolygonUntouched)
{
  using autoware_perception_msgs::msg::Shape;

  // Deliberately an L shape, whose vertex at (1.0, 1.0) is reflex. Upstream does not emit such a
  // footprint; it is used here because it is what makes the pass-through observable at all.
  const std::vector<Point2d> concave_footprint = {Point2d(0.0, 0.0), Point2d(4.0, 0.0),
                                                  Point2d(4.0, 1.0), Point2d(1.0, 1.0),
                                                  Point2d(1.0, 3.0), Point2d(0.0, 3.0)};
  const auto shape = create_shape(Shape::POLYGON, 0.0, 0.0, concave_footprint);

  expect_same_vertices(create_base_polygon(shape, false), concave_footprint);
  expect_same_vertices(create_base_polygon(shape, true), concave_footprint);

  EXPECT_TRUE(create_base_polygon(create_shape(Shape::POLYGON, 0.0, 0.0), true).empty());
}

// A hull without area stays without area: a zero-sized bounding box must not be padded into a
// polygon that reports a collision the object cannot have. Degenerate rings are already handled by
// intersects_sat(), see IntersectsSatMatchesBoostForRingsAdmittedByTheEmptyGuard.
TEST(CreateBasePolygonTest, KeepsDegenerateHullsDegenerate)
{
  using autoware_perception_msgs::msg::Shape;

  const auto coincident_vertices = create_base_polygon(
    create_shape(Shape::BOUNDING_BOX, 0.0, 0.0, {Point2d(0.0, 0.0), Point2d(0.0, 0.0)}), true);
  ASSERT_FALSE(coincident_vertices.empty());
  for (const auto & vertex : coincident_vertices) {
    EXPECT_DOUBLE_EQ(vertex.x(), 0.0);
    EXPECT_DOUBLE_EQ(vertex.y(), 0.0);
  }

  const auto collinear_vertices = create_base_polygon(
    create_shape(
      Shape::BOUNDING_BOX, 0.0, 0.0, {Point2d(0.0, 0.0), Point2d(1.0, 0.0), Point2d(2.0, 0.0)}),
    true);
  ASSERT_FALSE(collinear_vertices.empty());
  EXPECT_DOUBLE_EQ(boost::geometry::area(create_closed_ring(collinear_vertices)), 0.0);
  for (const auto & vertex : collinear_vertices) {
    EXPECT_DOUBLE_EQ(vertex.y(), 0.0);
    EXPECT_GE(vertex.x(), 0.0);
    EXPECT_LE(vertex.x(), 2.0);
  }
}

TEST(ToPolygon2dTest, ClosesTheRingAndAppliesThePoseToTheMergedHull)
{
  using autoware_perception_msgs::msg::Shape;

  constexpr double yaw = M_PI / 6.0;
  const double center_x = 1.0;
  const double center_y = 2.0;

  const std::vector<Point2d> extra_footprint = {Point2d(3.0, 0.0)};
  const auto shape = create_shape(Shape::BOUNDING_BOX, 4.0, 2.0, extra_footprint);

  geometry_msgs::msg::Pose pose;
  pose.position.x = center_x;
  pose.position.y = center_y;
  pose.orientation = autoware::universe_utils::createQuaternionFromYaw(yaw);

  const auto polygon = to_polygon2d(pose, shape, true);

  ASSERT_FALSE(polygon.outer().empty());
  EXPECT_TRUE(boost::geometry::equals(polygon.outer().front(), polygon.outer().back()));
  EXPECT_TRUE(boost::geometry::is_valid(polygon));
  expect_convex(polygon);

  std::vector<Point2d> expected_points;
  for (const auto & vertex : bounding_box_outline(4.0, 2.0)) {
    expected_points.push_back(
      rotate_and_translate(vertex.x(), vertex.y(), center_x, center_y, yaw));
  }
  for (const auto & vertex : extra_footprint) {
    expected_points.push_back(
      rotate_and_translate(vertex.x(), vertex.y(), center_x, center_y, yaw));
  }
  expect_covers_all(polygon, expected_points);

  const auto without_extra = to_polygon2d(pose, shape, false);
  EXPECT_TRUE(
    boost::geometry::equals(without_extra, create_oriented_box(center_x, center_y, 4.0, 2.0, yaw)));
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
