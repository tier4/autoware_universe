// Copyright 2024 TIER IV, Inc.
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

#include "autoware/boundary_departure_checker/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <fmt/format.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <limits>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{
struct FootprintMargin
{
  double lon;
  double lat;
};

FootprintMargin calcFootprintMargin(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const double scale)
{
  const auto cov_in_map = covariance.covariance;
  Eigen::Matrix2d cov_xy_map;
  cov_xy_map << cov_in_map[0 * 6 + 0], cov_in_map[0 * 6 + 1], cov_in_map[1 * 6 + 0],
    cov_in_map[1 * 6 + 1];

  const double yaw_vehicle = tf2::getYaw(covariance.pose.orientation);

  // To get a position in a transformed coordinate, rotate the inverse direction
  Eigen::Matrix2d r_map2vehicle;
  r_map2vehicle << std::cos(-yaw_vehicle), -std::sin(-yaw_vehicle), std::sin(-yaw_vehicle),
    std::cos(-yaw_vehicle);
  // Rotate covariance E((X, Y)^t*(X, Y)) = E(R*(x,y)*(x,y)^t*R^t)
  // when Rotate point (X, Y)^t= R*(x, y)^t.
  const Eigen::Matrix2d cov_xy_vehicle = r_map2vehicle * cov_xy_map * r_map2vehicle.transpose();

  // The longitudinal/lateral length is represented
  // in cov_xy_vehicle(0,0), cov_xy_vehicle(1,1) respectively.
  return FootprintMargin{cov_xy_vehicle(0, 0) * scale, cov_xy_vehicle(1, 1) * scale};
}

lanelet::BoundingBox2d create_bbox(
  const geometry_msgs::msg::Point & point, const double search_distance)
{
  constexpr auto eps{1e-3};
  if (search_distance < eps) {
    return {
      lanelet::BasicPoint2d{point.x - eps, point.y - eps},
      lanelet::BasicPoint2d{point.x + eps, point.y + eps}};
  }
  return {
    lanelet::BasicPoint2d{point.x - search_distance, point.y - search_distance},
    lanelet::BasicPoint2d{point.x + search_distance, point.y + search_distance}};
}

std::vector<lanelet::PrimitiveLayer<lanelet::LineString3d>::ConstPrimitiveT> get_nearby_linestrings(
  const lanelet::LaneletMap & lanelet_map, const geometry_msgs::msg::Point & point,
  const double search_distance)
{
  const auto search_bbox = create_bbox(point, search_distance);
  return lanelet_map.lineStringLayer.search(search_bbox);
}

using autoware_utils::Segment2d;
using SegmentWithIdx = std::tuple<Segment2d, lanelet::Id, size_t>;
namespace bg = boost::geometry;
namespace bgi = bg::index;
using SegmentWithIdxRtree = boost::geometry::index::rtree<SegmentWithIdx, bgi::rstar<16>>;

SegmentWithIdxRtree build_uncrossable_boundaries_rtree(
  const std::vector<lanelet::PrimitiveLayer<lanelet::LineString3d>::ConstPrimitiveT> & linestrings,
  const std::vector<std::string> & boundary_types_to_detect)
{
  std::vector<Segment2d> uncrossable_segments;
  const auto is_uncrossable_type = [&boundary_types_to_detect](const auto & ls) {
    constexpr auto no_type = "";
    const auto type = ls.attributeOr(lanelet::AttributeName::Type, no_type);
    return (
      type != no_type &&
      std::find(boundary_types_to_detect.begin(), boundary_types_to_detect.end(), type) !=
        boundary_types_to_detect.end());
  };

  std::vector<SegmentWithIdx> segments;
  segments.reserve(linestrings.size());
  for (const auto & linestring : linestrings) {
    if (!is_uncrossable_type(linestring)) {
      continue;
    }
    const auto & ls = linestring.basicLineString();
    for (size_t i = 0; i + 1 < ls.size(); ++i) {
      const autoware_utils::Point2d p1 = {ls.at(i).x(), ls.at(i).y()};
      const autoware_utils::Point2d p2 = {ls.at(i + 1).x(), ls.at(i + 1).y()};
      const Segment2d segment = {p1, p2};
      segments.emplace_back(bg::return_envelope<Segment2d>(segment), linestring.id(), i);
    }
  }

  return {segments.begin(), segments.end()};
}

}  // namespace

namespace autoware::lane_departure_checker::utils
{
using autoware_utils::Segment2d;
using lane_departure_checker::Projection;
using SegmentWithIdx = std::tuple<Segment2d, lanelet::Id, size_t>;
namespace bg = boost::geometry;
namespace bgi = bg::index;
using SegmentWithIdxRtree = boost::geometry::index::rtree<SegmentWithIdx, bgi::rstar<16>>;

TrajectoryPoints cutTrajectory(const TrajectoryPoints & trajectory, const double length)
{
  if (trajectory.empty()) {
    return {};
  }

  TrajectoryPoints cut;

  double total_length = 0.0;
  auto last_point = autoware_utils::from_msg(trajectory.front().pose.position);
  auto end_it = std::next(trajectory.cbegin());
  for (; end_it != trajectory.cend(); ++end_it) {
    const auto remain_distance = length - total_length;

    // Over length
    if (remain_distance <= 0) {
      break;
    }

    const auto & new_pose = end_it->pose;
    const auto new_point = autoware_utils::from_msg(new_pose.position);
    const auto points_distance = boost::geometry::distance(last_point.to_2d(), new_point.to_2d());

    // Require interpolation
    if (remain_distance <= points_distance) {
      const Eigen::Vector3d p_interpolated =
        last_point + remain_distance * (new_point - last_point).normalized();

      TrajectoryPoint p;
      p.pose.position.x = p_interpolated.x();
      p.pose.position.y = p_interpolated.y();
      p.pose.position.z = p_interpolated.z();
      p.pose.orientation = new_pose.orientation;

      cut.push_back(p);
      break;
    }

    total_length += points_distance;
    last_point = new_point;
  }
  cut.insert(cut.begin(), trajectory.begin(), end_it);

  return cut;
}

TrajectoryPoints resampleTrajectory(const Trajectory & trajectory, const double interval)
{
  if (trajectory.points.size() < 2) {
    return trajectory.points;
  }

  TrajectoryPoints resampled;

  resampled.push_back(trajectory.points.front());
  auto prev_point = autoware_utils::from_msg(trajectory.points.front().pose.position);
  for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
    const auto & traj_point = trajectory.points.at(i);

    const auto next_point = autoware_utils::from_msg(traj_point.pose.position);

    if (boost::geometry::distance(prev_point.to_2d(), next_point.to_2d()) >= interval) {
      resampled.push_back(traj_point);
      prev_point = next_point;
    }
  }
  resampled.push_back(trajectory.points.back());

  return resampled;
}

std::vector<std::pair<LinearRing2d, Pose>> createVehicleFootprints(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const TrajectoryPoints & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const double footprint_margin_scale)
{
  // Calculate longitudinal and lateral margin based on covariance
  const auto margin = calcFootprintMargin(covariance, footprint_margin_scale);

  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint = vehicle_info.createFootprint(margin.lat, margin.lon);

  // Create vehicle footprint on each TrajectoryPoint
  std::vector<std::pair<LinearRing2d, Pose>> vehicle_footprints;
  std::transform(
    trajectory.begin(), trajectory.end(), std::back_inserter(vehicle_footprints),
    [&](const auto & p) -> std::pair<LinearRing2d, Pose> {
      using autoware_utils::transform_vector;
      using autoware_utils::pose2transform;
      return {transform_vector(local_vehicle_footprint, pose2transform(p.pose)), p.pose};
    });

  return vehicle_footprints;
}

std::vector<LinearRing2d> createVehicleFootprints(
  const PathWithLaneId & path, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const double footprint_extra_margin)
{
  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint = vehicle_info.createFootprint(footprint_extra_margin);

  // Create vehicle footprint on each Path point
  std::vector<LinearRing2d> vehicle_footprints;
  for (const auto & p : path.points) {
    vehicle_footprints.push_back(autoware_utils::transform_vector(
      local_vehicle_footprint, autoware_utils::pose2transform(p.point.pose)));
  }

  return vehicle_footprints;
}

lanelet::ConstLanelets getCandidateLanelets(
  const lanelet::ConstLanelets & route_lanelets,
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  lanelet::ConstLanelets candidate_lanelets;

  // Find lanes within the convex hull of footprints
  const auto footprint_hull = createHullFromFootprints(vehicle_footprints);

  for (const auto & route_lanelet : route_lanelets) {
    const auto poly = route_lanelet.polygon2d().basicPolygon();
    if (!boost::geometry::disjoint(poly, footprint_hull)) {
      candidate_lanelets.push_back(route_lanelet);
    }
  }

  return candidate_lanelets;
}

LinearRing2d createHullFromFootprints(const std::vector<LinearRing2d> & footprints)
{
  MultiPoint2d combined;
  for (const auto & footprint : footprints) {
    for (const auto & p : footprint) {
      combined.push_back(p);
    }
  }

  LinearRing2d hull;
  boost::geometry::convex_hull(combined, hull);

  return hull;
}

std::vector<LinearRing2d> createVehiclePassingAreas(
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  if (vehicle_footprints.empty()) {
    return {};
  }

  if (vehicle_footprints.size() == 1) {
    return {vehicle_footprints.front()};
  }

  std::vector<LinearRing2d> areas;
  areas.reserve(vehicle_footprints.size() - 1);

  for (size_t i = 0; i < vehicle_footprints.size() - 1; ++i) {
    const auto & footprint1 = vehicle_footprints.at(i);
    const auto & footprint2 = vehicle_footprints.at(i + 1);
    areas.push_back(createHullFromFootprints({footprint1, footprint2}));
  }

  return areas;
}

double calcMaxSearchLengthForBoundaries(
  const Trajectory & trajectory, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const double max_ego_lon_length = std::max(
    std::abs(vehicle_info.max_longitudinal_offset_m),
    std::abs(vehicle_info.min_longitudinal_offset_m));
  const double max_ego_lat_length = std::max(
    std::abs(vehicle_info.max_lateral_offset_m), std::abs(vehicle_info.min_lateral_offset_m));
  const double max_ego_search_length = std::hypot(max_ego_lon_length, max_ego_lat_length);
  return autoware::motion_utils::calcArcLength(trajectory.points) + max_ego_search_length;
}

bool point_between_projection(const Point2d & p, const Segment2d & segment)
{
  const auto & p1 = segment.first;
  const auto & p2 = segment.second;

  const Point2d p2_vec = {p2.x() - p1.x(), p2.y() - p1.y()};
  const Point2d p_vec = {p.x() - p1.x(), p.y() - p1.y()};

  const auto c1 = boost::geometry::dot_product(p_vec, p2_vec);
  if (c1 < 0.0) return false;

  const auto c2 = boost::geometry::dot_product(p2_vec, p2_vec);
  return (c1 <= c2);
}

std::optional<Projection> point_to_segment_projection(
  const Point2d & p, const Segment2d & segment, const bool swap_points = false)
{
  const auto & p1 = segment.first;
  const auto & p2 = segment.second;

  const Point2d p2_vec = {p2.x() - p1.x(), p2.y() - p1.y()};
  const Point2d p_vec = {p.x() - p1.x(), p.y() - p1.y()};

  const auto result = [&swap_points](const Point2d & orig, const Point2d & proj) {
    return swap_points ? Projection{proj, orig, boost::geometry::distance(proj, orig)}
                       : Projection{orig, proj, boost::geometry::distance(orig, proj)};
  };

  const auto c1 = boost::geometry::dot_product(p_vec, p2_vec);
  if (c1 < 0.0) return std::nullopt;
  if (c1 == 0.0) return result(p, p1);

  const auto c2 = boost::geometry::dot_product(p2_vec, p2_vec);
  if (c1 > c2) return std::nullopt;

  if (c1 == c2) return result(p, p2);

  const auto projection = p1 + (p2_vec * c1 / c2);
  const auto projection_point = Point2d{projection.x(), projection.y()};

  // const auto direction =
  //   (p2.x() - p1.x()) * (p.y() - p1.y()) - (p2.y() - p1.y()) * (p.x() - p1.x());
  return result(p, projection_point);
  // projected.dist = std::copysign(projected.dist, direction);
  // return projected;
}

std::optional<Projection> segment_to_segment_nearest_projection(
  const Segment2d & ego_seg, const Segment2d & lane_seg)
{
  std::vector<Projection> projections;

  if (
    const auto is_intersecting = autoware_utils::intersect(
      autoware_utils::to_msg(ego_seg.first.to_3d(0.0)),
      autoware_utils::to_msg(ego_seg.second.to_3d()),
      autoware_utils::to_msg(lane_seg.first.to_3d()),
      autoware_utils::to_msg(lane_seg.second.to_3d()))) {
    Point2d point(is_intersecting->x, is_intersecting->y);
    return Projection{point, point, 0.0};
  }
  constexpr bool swap_result = true;
  if (
    const auto projection_opt = point_to_segment_projection(ego_seg.first, lane_seg, swap_result)) {
    projections.push_back(*projection_opt);
  }

  if (
    const auto projection_opt =
      point_to_segment_projection(ego_seg.second, lane_seg, swap_result)) {
    projections.push_back(*projection_opt);
  }

  if (
    const auto projection_opt =
      point_to_segment_projection(lane_seg.first, ego_seg, !swap_result)) {
    projections.push_back(*projection_opt);
  }

  if (
    const auto projection_opt =
      point_to_segment_projection(lane_seg.second, ego_seg, !swap_result)) {
    projections.push_back(*projection_opt);
  }

  if (projections.empty()) return std::nullopt;
  if (projections.size() == 1) return projections.front();

  const auto min_elem = std::min_element(
    projections.begin(), projections.end(), [](const Projection & proj1, const Projection & proj2) {
      return std::abs(proj1.dist) < std::abs(proj2.dist);
    });

  return *min_elem;
}

std::optional<Projection> get_left_dist_to_boundary(
  const Segment2d & ego_left, const Segment2d & boundary_segment)
{
  auto left_opt = segment_to_segment_nearest_projection(ego_left, boundary_segment);

  if (!left_opt) return std::nullopt;

  // If projection is clearly on the right of the segment, discard
  if (std::abs(left_opt->dist) >= 1e-3 && !std::signbit(left_opt->dist)) {
    return std::nullopt;
  }

  return left_opt;
}

std::optional<Projection> get_right_dist_to_boundary(
  const Segment2d & ego_right, const Segment2d & boundary_segment)
{
  auto right_opt = segment_to_segment_nearest_projection(ego_right, boundary_segment);

  if (!right_opt) return std::nullopt;

  // If projection is clearly on the left of the segment, discard
  if (std::abs(right_opt->dist) >= 1e-3 && std::signbit(right_opt->dist)) {
    return std::nullopt;
  }

  return right_opt;
}

std::optional<Projection> get_dist_to_boundaries(
  const EgoSide & ego_footprint_side, const Segment2d & boundary_segment)
{
  auto left_opt = segment_to_segment_nearest_projection(ego_footprint_side.left, boundary_segment);
  auto right_opt =
    segment_to_segment_nearest_projection(ego_footprint_side.right, boundary_segment);

  if (!left_opt && !right_opt) {
    return std::nullopt;
  }

  if (left_opt && !right_opt) {
    if (std::abs(left_opt->dist) >= 1e-3 && !std::signbit(left_opt->dist)) {
      return std::nullopt;
    }
    return left_opt;
  }

  if (right_opt && !left_opt) {
    if (std::abs(right_opt->dist) >= 1e-3 && std::signbit(right_opt->dist)) {
      return std::nullopt;
    }
    return right_opt;
  }

  if (std::abs(right_opt->dist) < 1e-3) {
    return right_opt;
  }

  if (std::abs(left_opt->dist) < 1e-3) {
    return left_opt;
  }

  if (std::signbit(left_opt->dist) && std::signbit(right_opt->dist)) {
    // fmt::print("left: {:.3f}, right: {:.3f}. Return left\n", left_opt->dist, right_opt->dist);
    return left_opt;
  }
  if (!std::signbit(left_opt->dist) && !std::signbit(right_opt->dist)) {
    // fmt::print("left: {:.3f}, right: {:.3f}. Return right\n", left_opt->dist, right_opt->dist);
    return right_opt;
  }

  // fmt::print("left: {:.3f}, right: {:.3f}. Neutral\n", left_opt->dist, right_opt->dist);
  return right_opt;
}

SideToBoundPojections get_closest_boundary(
  const lanelet::LaneletMap & lanelet_map, const Pose & curr_pose,
  const EgoSides & ego_sides_from_footprints,
  const std::vector<std::string> & boundary_types_to_detect, const double max_global_vel,
  const double vehicle_length)
{
  if (ego_sides_from_footprints.empty()) {
    return {};
  }
  [[maybe_unused]] constexpr auto predicted_time_duration{10.0};
  [[maybe_unused]] const auto search_distance =
    std::max(
      max_global_vel * predicted_time_duration, ego_sides_from_footprints.back().dist_from_start) +
    vehicle_length;
  [[maybe_unused]] const auto nearby_linestring =
    get_nearby_linestrings(lanelet_map, curr_pose.position, search_distance);
  [[maybe_unused]] const auto uncrossable_bound_rtree =
    build_uncrossable_boundaries_rtree(nearby_linestring, boundary_types_to_detect);

  return {};
}

SideToBoundPojections get_closest_boundary_from_side(
  const lanelet::LaneletMap & lanelet_map, const EgoSides & ego_sides_from_footprints,
  const std::vector<std::string> & boundary_types_to_detect, const double max_lat_query_dist)
{
  const auto is_uncrossable_type = [&boundary_types_to_detect](const auto & ls) {
    constexpr auto no_type = "";
    const auto type = ls.attributeOr(lanelet::AttributeName::Type, no_type);
    return (
      type != no_type &&
      std::find(boundary_types_to_detect.begin(), boundary_types_to_detect.end(), type) !=
        boundary_types_to_detect.end());
  };

  const auto project_closest = [&](
                                 const Pose & curr_pose, const auto & ego_seg,
                                 const auto is_target_left, auto & output_side, const auto idx,
                                 const auto & compare_seg) {
    std::optional<Projection> closest_proj;
    Segment2d closest_seg;
    double min_dist = max_lat_query_dist;
    const lanelet::BasicPoint2d ego_pose{curr_pose.position.x, curr_pose.position.y};
    const lanelet::BasicPoint2d ego_start{ego_seg.first.x(), ego_seg.first.y()};
    const lanelet::BasicSegment2d ego_line{ego_seg.first, ego_seg.second};
    const lanelet::BasicSegment2d compare_side{compare_seg.first, compare_seg.second};

    lanelet_map.lineStringLayer.nearestUntil(
      ego_start, [&](const auto & bbox, const lanelet::ConstLineString3d & ls) {
        if (!is_uncrossable_type(ls)) return false;

        const double bbox_dist = lanelet::geometry::distance2d(bbox, ego_start);
        if (bbox_dist > min_dist) return true;

        const double dist = lanelet::geometry::distance2d(ego_pose, ls.basicLineString());
        if (dist >= min_dist) return false;

        const auto dist_output = lanelet::geometry::distance2d(ego_line, ls.basicLineString());
        const auto dist_compare = lanelet::geometry::distance2d(compare_side, ls.basicLineString());
        if (dist_compare < dist_output) return false;

        const auto & basic_ls = ls.basicLineString();
        for (size_t i = 0; i + 1 < basic_ls.size(); ++i) {
          const Point2d p1{basic_ls[i].x(), basic_ls[i].y()};
          const Point2d p2{basic_ls[i + 1].x(), basic_ls[i + 1].y()};
          const Segment2d lane_seg{p1, p2};

          if (const auto proj_opt = segment_to_segment_nearest_projection(ego_seg, lane_seg)) {
            const auto & [front, back] = ego_seg;
            const auto direction = (back.x() - front.x()) * (proj_opt->orig.y() - front.y()) -
                                   (back.y() - front.y()) * (proj_opt->orig.x() - front.x());
            const bool is_left_of_ego = std::signbit(direction);

            if (is_left_of_ego != is_target_left) continue;

            if ((!closest_proj || proj_opt->dist < closest_proj->dist)) {
              closest_proj = *proj_opt;
              closest_seg = lane_seg;
              min_dist = dist;
            }
          }
        }

        return false;
      });

    if (closest_proj) {
      ProjectionWithSegment output;
      output.projection = *closest_proj;
      output.nearest_segment = closest_seg;
      output.idx_from_ego_sides_from_footprints = idx;
      output_side.push_back(output);
    }
  };

  SideToBoundPojections side;
  side.left.reserve(ego_sides_from_footprints.size());
  side.right.reserve(ego_sides_from_footprints.size());

  for (size_t i = 0; i < ego_sides_from_footprints.size(); ++i) {
    const auto & ego = ego_sides_from_footprints[i];
    // Use the lambda for both left and right
    project_closest(ego.pose, ego.left, true, side.left, i, ego.right);    // Left side
    project_closest(ego.pose, ego.right, false, side.right, i, ego.left);  // Right side
  }

  return side;
}
}  // namespace autoware::lane_departure_checker::utils
