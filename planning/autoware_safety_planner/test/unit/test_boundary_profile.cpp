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

#include "trajectory_planner/frenet_sampling_based_planner/compiled_constraints_utils.hpp"

#ifdef SAFETY_PLANNER_UNIT_TEST_PLOT
#include "test_plot_utils.hpp"
#endif

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

// example bus profile
VehicleInfo make_bus()
{
  VehicleInfo vehicle_info;
  vehicle_info.wheel_base_m = 4.76;
  vehicle_info.max_longitudinal_offset_m = 4.76 + 0.95;
  vehicle_info.min_longitudinal_offset_m = -1.53;
  vehicle_info.max_lateral_offset_m = 1.20;
  vehicle_info.min_lateral_offset_m = -1.23;
  return vehicle_info;
}

//! A reference path turning right on a circle of radius r: it starts at the origin heading +x, the
//! center is at (0, -r), and the left normal points away from the center
struct Arc
{
  double r;

  Pose2d pose(const double s, const double l) const
  {
    const double phi = s / r;
    return Pose2d{Point2d{(r + l) * std::sin(phi), -r + (r + l) * std::cos(phi)}, -phi};
  }

  SlPoint project(const Point2d & q) const
  {
    return {r * std::atan2(q.x(), q.y() + r), std::hypot(q.x(), q.y() + r) - r};
  }
};

//! A curb on the outside of the turn, ending in a sharp tip at (s_tip, l_tip) with a short hook,
//! as the curb at the corner of a junction: from far outside along the normal down to the tip
LateralBoundEntry make_tip_boundary(const Arc & arc, const double s_tip, const double l_tip)
{
  std::vector<Point2d> polyline{arc.pose(s_tip, l_tip + 20.0).position};
  const auto tip = arc.pose(s_tip, l_tip).position;
  const auto hook = arc.pose(s_tip + 0.4, l_tip + 0.1).position;
  // densified to 0.5 m, as the compiler does
  const auto far = polyline.front();
  for (int i = 1; i <= 40; ++i) {
    const double t = i / 40.0;
    polyline.emplace_back(far.x() + t * (tip.x() - far.x()), far.y() + t * (tip.y() - far.y()));
  }
  polyline.push_back(hook);

  LateralBoundEntry bound;
  bound.forbidden_side = Side::LEFT;
  std::vector<ProjectedVertex> piece;
  std::vector<SlPoint> sl_piece;
  for (const auto & p : polyline) {
    piece.push_back({p, arc.project(p)});
    sl_piece.push_back(arc.project(p));
  }
  bound.pieces = {piece};
  bound.polyline = make_lateral_envelope({sl_piece}, Side::LEFT);
  return bound;
}

Polygon2d footprint_polygon(const VehicleInfo & vehicle_info, const Pose2d & rear_axle)
{
  Polygon2d polygon;
  const double c = std::cos(rear_axle.yaw);
  const double s = std::sin(rear_axle.yaw);
  for (const auto & [x, y] :
       {std::pair{vehicle_info.max_longitudinal_offset_m, vehicle_info.max_lateral_offset_m},
        std::pair{vehicle_info.min_longitudinal_offset_m, vehicle_info.max_lateral_offset_m},
        std::pair{vehicle_info.min_longitudinal_offset_m, vehicle_info.min_lateral_offset_m},
        std::pair{vehicle_info.max_longitudinal_offset_m, vehicle_info.min_lateral_offset_m}}) {
    polygon.outer().emplace_back(
      rear_axle.position.x() + c * x - s * y, rear_axle.position.y() + s * x + c * y);
  }
  boost::geometry::correct(polygon);
  return polygon;
}

double point_segment_distance(const Point2d & p, const Point2d & a, const Point2d & b)
{
  const double dx = b.x() - a.x();
  const double dy = b.y() - a.y();
  const double len2 = dx * dx + dy * dy;
  const double t =
    len2 > 0.0 ? std::clamp(((p.x() - a.x()) * dx + (p.y() - a.y()) * dy) / len2, 0.0, 1.0) : 0.0;
  return std::hypot(p.x() - a.x() - t * dx, p.y() - a.y() - t * dy);
}

//! Between a polygon and a polyline that do not intersect, the distance is taken at a vertex of
//! one of them (boost::geometry::distance has no strategy for Point2d)
double distance(const Polygon2d & polygon, const LineString2d & line)
{
  const auto & ring = polygon.outer();
  double d = INF;
  for (std::size_t i = 0; i + 1 < ring.size(); ++i) {
    for (std::size_t j = 0; j + 1 < line.size(); ++j) {
      d = std::min(
        {d, point_segment_distance(ring[i], line[j], line[j + 1]),
         point_segment_distance(line[j], ring[i], ring[i + 1])});
    }
    d = std::min(d, point_segment_distance(line.back(), ring[i], ring[i + 1]));
  }
  return d;
}

//! The profile of the cell holding s, built as ConstraintTables does
BoundaryProfile profile_at(
  const Arc & arc, const LateralBoundEntry & bound, const VehicleInfo & vehicle_info,
  const double s)
{
  constexpr double RES = 0.5;
  const double s_cell = std::floor(s / RES) * RES;
  const double reach = std::hypot(
    std::max(vehicle_info.max_longitudinal_offset_m, -vehicle_info.min_longitudinal_offset_m),
    std::max(vehicle_info.max_lateral_offset_m, -vehicle_info.min_lateral_offset_m));
  const double x_min = -reach - 2.0;
  const double x_max = RES + reach + 2.0;
  return make_boundary_profile(
    {&bound}, arc.pose(s_cell, 0.0), x_min, x_max, s_cell + 2.0 * x_min, s_cell + 2.0 * x_max,
    0.25);
}

class BoundaryProfileTest : public ::testing::TestWithParam<double>
{
};

}  // namespace

// Random rear axle poses around a sharp tip on the outside of a curve: the rigid footprint checked
// on the profile never misses a contact with the boundary (boost intersects as the oracle), and
// rejects only poses that come within a bin or so of it or reach past the tip laterally. For
// reference, the misses of the (s, l) box on the envelope, which the planner used before, are
// counted as well
TEST_P(BoundaryProfileTest, FootprintNeverMissesASharpTip)
{
  const Arc arc{GetParam()};
  const auto vehicle_info = make_bus();
  const double s_tip = 20.0;
  const double l_tip = 2.5;
  const auto bound = make_tip_boundary(arc, s_tip, l_tip);
  LineString2d boundary_world;
  for (const auto & v : bound.pieces.front()) {
    boundary_world.push_back(v.position);
  }

  std::mt19937 rng(0);
  std::uniform_real_distribution<double> s_dist(s_tip - 10.0, s_tip + 3.0);
  std::uniform_real_distribution<double> l_dist(-1.0, 1.0);
  std::uniform_real_distribution<double> theta_dist(-0.3, 0.3);
  int contacts = 0;
  int misses = 0;
  int sl_box_misses = 0;
  int false_alarms = 0;
  std::vector<Polygon2d> missed_by_sl_box;
  for (int i = 0; i < 20000; ++i) {
    const double s = s_dist(rng);
    const double l = l_dist(rng);
    auto rear_axle = arc.pose(s, l);
    rear_axle.yaw += theta_dist(rng);
    const auto polygon = footprint_polygon(vehicle_info, rear_axle);
    const bool contact = boost::geometry::intersects(polygon, boundary_world);
    const bool hit = footprint_hits_boundary(
      profile_at(arc, bound, vehicle_info, s), vehicle_info, rear_axle, 0.0);
    contacts += contact;
    if (contact && !hit) {
      ++misses;
      ADD_FAILURE() << "missed at s = " << s << ", l = " << l;
    }
    if (!contact && hit) {
      ++false_alarms;
      // The profile keeps one side per bin, so a footprint that has already passed the tip
      // laterally, into the forbidden side, is rejected even where it does not touch the curb
      double l_max = -INF;
      for (const auto & p : polygon.outer()) {
        l_max = std::max(l_max, arc.project(p).l);
      }
      EXPECT_TRUE(distance(polygon, boundary_world) < 0.5 || l_max > l_tip)
        << "rejected far from the boundary at s = " << s << ", l = " << l;
    }
    if (contact && !violates_lateral_bound(bound, footprint_sl_box(vehicle_info, s, l))) {
      ++sl_box_misses;
      if (missed_by_sl_box.size() < 30) {
        missed_by_sl_box.push_back(polygon);
      }
    }
  }
  std::cout << "R = " << arc.r << ": contacts " << contacts << ", misses " << misses
            << ", false alarms " << false_alarms << ", misses of the (s, l) box " << sl_box_misses
            << std::endl;
  EXPECT_GT(contacts, 0);

#ifdef SAFETY_PLANNER_UNIT_TEST_PLOT
  auto plt = autoware::pyplot::import();
  auto [fig, ax] = plt.subplots(Kwargs("figsize"_a = py::make_tuple(10, 10)));
  std::vector<double> cx, cy;
  for (double s = 0.0; s <= s_tip + 10.0; s += 0.5) {
    const auto p = arc.pose(s, 0.0).position;
    cx.push_back(p.x());
    cy.push_back(p.y());
  }
  ax.plot(Args(cx, cy), Kwargs("color"_a = "tab:blue"));
  std::vector<double> bx, by;
  for (const auto & p : boundary_world) {
    bx.push_back(p.x());
    by.push_back(p.y());
  }
  ax.plot(Args(bx, by), Kwargs("color"_a = "tab:red", "linewidth"_a = 2.0));
  // Footprints in contact with the tip that the (s, l) box let through
  for (const auto & polygon : missed_by_sl_box) {
    std::vector<double> px, py;
    for (const auto & p : polygon.outer()) {
      px.push_back(p.x());
      py.push_back(p.y());
    }
    ax.plot(Args(px, py), Kwargs("color"_a = "tab:orange", "linewidth"_a = 0.5));
  }
  const auto tip = arc.pose(s_tip, l_tip).position;
  ax.set_xlim(Args(tip.x() - 12.0, tip.x() + 8.0));
  ax.set_ylim(Args(tip.y() - 14.0, tip.y() + 6.0));
  ax.set_aspect(Args("equal"));
  ax.set_title(
    Args("R = " + std::to_string(arc.r) + " m, orange: contacts missed by the (s, l) box"));
  save_figure(plt, "unit");
#endif
}

INSTANTIATE_TEST_SUITE_P(
  Radius, BoundaryProfileTest, ::testing::Values(1e4, 20.0, 10.0, 6.4),
  [](const ::testing::TestParamInfo<double> & info) {
    return "R" + std::to_string(static_cast<int>(info.param * 10.0));
  });

namespace
{

//! A road heading +y that turns right on a circle of radius r (center (r, 0)) and goes on along +x
struct RightTurn
{
  double r;
  double straight;  //!< [m] length of the straight before and after the turn

  double length() const { return 2.0 * straight + 0.5 * M_PI * r; }

  Pose2d pose(const double s) const
  {
    if (s < straight) {
      return Pose2d{Point2d{0.0, s - straight}, 0.5 * M_PI};
    }
    const double u = s - straight;
    if (u < 0.5 * M_PI * r) {
      const double phi = u / r;
      return Pose2d{Point2d{r - r * std::cos(phi), r * std::sin(phi)}, 0.5 * M_PI - phi};
    }
    return Pose2d{Point2d{r + u - 0.5 * M_PI * r, r}, 0.0};
  }
};

}  // namespace

// The corner of a junction (the first picture of the lateral_bounds issue): the curb on the left of
// the road after the turn comes down to a sharp tip near the corner. Seen from the frames before
// the turn it lies ahead and to the right, clear of the lane, so driving along the centerline must
// pass
TEST(BoundaryProfile, CurbOfTheRoadAfterTheTurnDoesNotBlockTheLaneBeforeIt)
{
  const RightTurn road{8.0, 30.0};
  const auto vehicle_info = make_bus();
  // The tip is 3 m left of the road after the turn and 7 m right of the road before it
  const Point2d tip{7.0, road.r + 3.0};
  LineString2d curb{{7.0, road.r + 30.0}, tip, {7.3, road.r + 3.1}};

  // The projection onto the centerline, by its nearest sample
  std::vector<Pose2d> centerline;
  for (double s = 0.0; s <= road.length(); s += 0.1) {
    centerline.push_back(road.pose(s));
  }
  const auto project = [&](const Point2d & q) {
    std::size_t best = 0;
    for (std::size_t i = 0; i < centerline.size(); ++i) {
      if (
        (centerline[i].position - q).squaredNorm() <
        (centerline[best].position - q).squaredNorm()) {
        best = i;
      }
    }
    const auto & c = centerline[best];
    const double dx = q.x() - c.position.x();
    const double dy = q.y() - c.position.y();
    return SlPoint{0.1 * static_cast<double>(best), std::cos(c.yaw) * dy - std::sin(c.yaw) * dx};
  };
  LateralBoundEntry bound;
  bound.forbidden_side = Side::LEFT;
  bound.pieces.emplace_back();
  for (std::size_t i = 0; i + 1 < curb.size(); ++i) {
    for (int k = 0; k < 20; ++k) {
      const double t = k / 20.0;
      const Point2d p{
        curb[i].x() + t * (curb[i + 1].x() - curb[i].x()),
        curb[i].y() + t * (curb[i + 1].y() - curb[i].y())};
      bound.pieces.back().push_back({p, project(p)});
    }
  }
  bound.pieces.back().push_back({curb.back(), project(curb.back())});

  constexpr double RES = 0.5;
  const double reach = std::hypot(
    std::max(vehicle_info.max_longitudinal_offset_m, -vehicle_info.min_longitudinal_offset_m),
    std::max(vehicle_info.max_lateral_offset_m, -vehicle_info.min_lateral_offset_m));
  const double x_min = -reach - 2.0;
  const double x_max = RES + reach + 2.0;
  for (double s = 0.0; s <= road.length() - 10.0; s += 0.25) {
    const double s_cell = std::floor(s / RES) * RES;
    const auto profile = make_boundary_profile(
      {&bound}, road.pose(s_cell), x_min, x_max, s_cell + 2.0 * x_min, s_cell + 2.0 * x_max, 0.25);
    const auto rear_axle = road.pose(s);
    ASSERT_FALSE(boost::geometry::intersects(footprint_polygon(vehicle_info, rear_axle), curb));
    EXPECT_FALSE(footprint_hits_boundary(profile, vehicle_info, rear_axle, 0.0)) << "s = " << s;
  }
}

// The other leg of a hairpin lies within the bins of a frame but far along the path; its segments
// are left out by the arc length window
TEST(BoundaryProfile, ArcLengthWindowKeepsTheOtherLegOut)
{
  LateralBoundEntry bound;
  bound.forbidden_side = Side::LEFT;
  bound.pieces = {{{Point2d{-5.0, 1.0}, {95.0, 1.0}}, {Point2d{5.0, 1.0}, {105.0, 1.0}}}};
  const auto profile = make_boundary_profile({&bound}, Pose2d{}, -5.0, 5.0, -10.0, 10.0, 0.25);
  EXPECT_EQ(profile.left_min, INF);

  const auto near = make_boundary_profile({&bound}, Pose2d{}, -5.0, 5.0, 90.0, 110.0, 0.25);
  EXPECT_DOUBLE_EQ(near.left_min, 1.0);
}

}  // namespace autoware::safety_planner
