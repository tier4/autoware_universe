// Copyright 2020 Tier IV, Inc.
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

#include "autoware/behavior_velocity_intersection_module/util.hpp"

#include "autoware/behavior_velocity_intersection_module/interpolated_path_info.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::util
{
namespace bg = boost::geometry;

static std::optional<size_t> getDuplicatedPointIdx(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Point & point)
{
  for (size_t i = 0; i < path.points.size(); i++) {
    const auto & p = path.points.at(i).point.pose.position;

    constexpr double min_dist = 0.001;
    if (autoware_utils::calc_distance2d(p, point) < min_dist) {
      return i;
    }
  }

  return std::nullopt;
}

std::optional<size_t> insertPointIndex(
  const geometry_msgs::msg::Pose & in_pose,
  autoware_internal_planning_msgs::msg::PathWithLaneId * inout_path,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold)
{
  const auto duplicate_idx_opt = getDuplicatedPointIdx(*inout_path, in_pose.position);
  if (duplicate_idx_opt) {
    return duplicate_idx_opt.value();
  }

  const size_t closest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    inout_path->points, in_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  // vector.insert(i) inserts element on the left side of v[i]
  // the velocity need to be zero order hold(from prior point)
  int insert_idx = closest_idx;
  autoware_internal_planning_msgs::msg::PathPointWithLaneId inserted_point =
    inout_path->points.at(closest_idx);
  if (planning_utils::isAheadOf(in_pose, inout_path->points.at(closest_idx).point.pose)) {
    ++insert_idx;
  } else {
    // copy with velocity from prior point
    const size_t prior_ind = closest_idx > 0 ? closest_idx - 1 : 0;
    inserted_point.point.longitudinal_velocity_mps =
      inout_path->points.at(prior_ind).point.longitudinal_velocity_mps;
  }
  inserted_point.point.pose = in_pose;

  auto it = inout_path->points.begin() + insert_idx;
  inout_path->points.insert(it, inserted_point);

  return insert_idx;
}

bool hasLaneIds(
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId & p,
  const std::set<lanelet::Id> & ids)
{
  for (const auto & pid : p.lane_ids) {
    if (ids.find(pid) != ids.end()) {
      return true;
    }
  }
  return false;
}

std::optional<std::pair<size_t, size_t>> findLaneIdsInterval(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & p, const std::set<lanelet::Id> & ids)
{
  bool found = false;
  size_t start = 0;
  size_t end = p.points.size() > 0 ? p.points.size() - 1 : 0;
  if (start == end) {
    // there is only one point in the path
    return std::nullopt;
  }
  for (size_t i = 0; i < p.points.size(); ++i) {
    if (hasLaneIds(p.points.at(i), ids)) {
      if (!found) {
        // found interval for the first time
        found = true;
        start = i;
      }
    } else if (found) {
      // prior point was in the interval. interval ended
      end = i;
      break;
    }
  }
  start = start > 0 ? start - 1 : 0;  // the idx of last point before the interval
  return found ? std::make_optional(std::make_pair(start, end)) : std::nullopt;
}

std::optional<size_t> getFirstPointInsidePolygonByFootprint(
  const lanelet::CompoundPolygon3d & polygon, const InterpolatedPathInfo & interpolated_path_info,
  const autoware_utils::LinearRing2d & footprint, const double vehicle_length)
{
  const auto & path_ip = interpolated_path_info.path;
  const auto [lane_start, lane_end] = interpolated_path_info.lane_id_interval.value();
  const size_t vehicle_length_idx = static_cast<size_t>(vehicle_length / interpolated_path_info.ds);
  // NOTE(soblin): subtract vehicle_length_idx for the case where ego's footprint overlaps with
  // attention lane at lane_start already. In such case, the intersection stopline needs to be
  // generated inside the lanelet before the intersection
  const size_t start =
    static_cast<size_t>(std::max<int>(0, static_cast<int>(lane_start) - vehicle_length_idx));
  const auto area_2d = lanelet::utils::to2D(polygon).basicPolygon();
  for (auto i = start; i <= lane_end; ++i) {
    const auto & base_pose = path_ip.points.at(i).point.pose;
    const auto path_footprint =
      autoware_utils::transform_vector(footprint, autoware_utils::pose2transform(base_pose));
    const auto footprint_front_part = autoware_utils_geometry::LineString2d{
      path_footprint.at(vehicle_info_utils::VehicleInfo::FrontLeftIndex),
      path_footprint.at(vehicle_info_utils::VehicleInfo::FrontRightIndex)};
    if (
      bg::intersects(footprint_front_part, area_2d) || bg::within(footprint_front_part, area_2d)) {
      return std::make_optional<size_t>(i);
    }
  }
  return std::nullopt;
}

std::optional<std::pair<
  size_t /* the index of interpolated PathPoint*/, size_t /* the index of corresponding Polygon */>>
getFirstPointInsidePolygonsByFootprint(
  const std::vector<lanelet::CompoundPolygon3d> & polygons,
  const InterpolatedPathInfo & interpolated_path_info,
  const autoware_utils::LinearRing2d & footprint, const double vehicle_length)
{
  const auto & path_ip = interpolated_path_info.path;
  const auto [lane_start, lane_end] = interpolated_path_info.lane_id_interval.value();
  const size_t vehicle_length_idx = static_cast<size_t>(vehicle_length / interpolated_path_info.ds);
  const size_t start =
    static_cast<size_t>(std::max<int>(0, static_cast<int>(lane_start) - vehicle_length_idx));

  for (size_t i = start; i <= lane_end; ++i) {
    const auto & pose = path_ip.points.at(i).point.pose;
    const auto path_footprint =
      autoware_utils::transform_vector(footprint, autoware_utils::pose2transform(pose));
    for (size_t j = 0; j < polygons.size(); ++j) {
      const auto area_2d = lanelet::utils::to2D(polygons.at(j)).basicPolygon();
      const bool is_in_polygon = bg::intersects(area_2d, path_footprint);
      if (is_in_polygon) {
        return std::make_optional<std::pair<size_t, size_t>>(i, j);
      }
    }
  }
  return std::nullopt;
}

std::optional<size_t> getFirstPointInsidePolygon(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const std::pair<size_t, size_t> lane_interval, const lanelet::CompoundPolygon3d & polygon,
  const bool search_forward)
{
  // NOTE: if first point is already inside the polygon, returns nullopt
  const auto polygon_2d = lanelet::utils::to2D(polygon);
  if (search_forward) {
    const auto & p0 = path.points.at(lane_interval.first).point.pose.position;
    if (bg::within(to_bg2d(p0), polygon_2d)) {
      return std::nullopt;
    }
    for (size_t i = lane_interval.first; i <= lane_interval.second; ++i) {
      const auto & p = path.points.at(i).point.pose.position;
      const auto is_in_lanelet = bg::within(to_bg2d(p), polygon_2d);
      if (is_in_lanelet) {
        return std::make_optional<size_t>(i);
      }
    }
  } else {
    const auto & p0 = path.points.at(lane_interval.second).point.pose.position;
    if (bg::within(to_bg2d(p0), polygon_2d)) {
      return std::nullopt;
    }
    for (size_t i = lane_interval.second; i >= lane_interval.first; --i) {
      const auto & p = path.points.at(i).point.pose.position;
      const auto is_in_lanelet = bg::within(to_bg2d(p), polygon_2d);
      if (is_in_lanelet) {
        return std::make_optional<size_t>(i);
      }
      if (i == 0) {
        break;
      }
    }
  }
  return std::nullopt;
}

void retrievePathsBackward(
  const std::vector<std::vector<bool>> & adjacency, const size_t src_index,
  const std::vector<size_t> & visited_indices, std::vector<std::vector<size_t>> & paths)
{
  const auto & next_indices = adjacency.at(src_index);
  const bool is_terminal =
    (std::find(next_indices.begin(), next_indices.end(), true) == next_indices.end());
  if (is_terminal) {
    std::vector<size_t> path(visited_indices.begin(), visited_indices.end());
    path.push_back(src_index);
    paths.emplace_back(std::move(path));
    return;
  }
  for (size_t next = 0; next < next_indices.size(); next++) {
    if (!next_indices.at(next)) {
      continue;
    }
    if (std::find(visited_indices.begin(), visited_indices.end(), next) != visited_indices.end()) {
      // loop detected
      std::vector<size_t> path(visited_indices.begin(), visited_indices.end());
      path.push_back(src_index);
      paths.emplace_back(std::move(path));
      continue;
    }
    auto new_visited_indices = visited_indices;
    new_visited_indices.push_back(src_index);
    retrievePathsBackward(adjacency, next, new_visited_indices, paths);
  }
  return;
}

std::pair<lanelet::ConstLanelets, std::vector<lanelet::ConstLanelets>>
mergeLaneletsByTopologicalSort(
  const lanelet::ConstLanelets & lanelets, const lanelet::ConstLanelets & terminal_lanelets,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr)
{
  std::set<lanelet::Id> lanelet_Ids;
  std::unordered_map<lanelet::Id, size_t> Id2ind;
  std::unordered_map<size_t, lanelet::Id> ind2Id;
  std::unordered_map<lanelet::Id, lanelet::ConstLanelet> Id2lanelet;
  for (const auto & lanelet : lanelets) {
    size_t ind = ind2Id.size();
    const auto Id = lanelet.id();
    lanelet_Ids.insert(Id);
    Id2ind[Id] = ind;
    ind2Id[ind] = Id;
    Id2lanelet[Id] = lanelet;
  }
  std::set<size_t> terminal_indices;
  for (const auto & terminal_lanelet : terminal_lanelets) {
    if (Id2ind.count(terminal_lanelet.id()) > 0) {
      terminal_indices.insert(Id2ind[terminal_lanelet.id()]);
    }
  }

  // create adjacency matrix
  const auto n_node = lanelets.size();
  std::vector<std::vector<bool>> adjacency(n_node);
  for (size_t dst = 0; dst < n_node; ++dst) {
    adjacency[dst].resize(n_node);
    for (size_t src = 0; src < n_node; ++src) {
      adjacency[dst][src] = false;
    }
  }
  // NOTE: this function aims to traverse the detection lanelet in the lane direction, so if lane B
  // follows lane A on the routing_graph, adj[A][B] = true
  for (const auto & lanelet : lanelets) {
    const auto & followings = routing_graph_ptr->following(lanelet);
    const auto src = lanelet.id();
    for (const auto & following : followings) {
      if (const auto dst = following.id(); lanelet_Ids.find(dst) != lanelet_Ids.end()) {
        adjacency[(Id2ind[dst])][(Id2ind[src])] = true;
      }
    }
  }

  std::unordered_map<size_t, std::vector<std::vector<size_t>>> branches;
  for (const auto & terminal_ind : terminal_indices) {
    std::vector<std::vector<size_t>> paths;
    std::vector<size_t> visited;
    retrievePathsBackward(adjacency, terminal_ind, visited, paths);
    branches[terminal_ind] = std::move(paths);
  }

  for (auto it = branches.begin(); it != branches.end(); it++) {
    auto & paths = it->second;
    for (auto & path : paths) {
      std::reverse(path.begin(), path.end());
    }
  }
  lanelet::ConstLanelets merged;
  std::vector<lanelet::ConstLanelets> originals;
  for (const auto & [ind, sub_branches] : branches) {
    if (sub_branches.size() == 0) {
      continue;
    }
    for (const auto & sub_indices : sub_branches) {
      lanelet::ConstLanelets to_be_merged;
      originals.push_back(lanelet::ConstLanelets({}));
      auto & original = originals.back();
      for (const auto & sub_ind : sub_indices) {
        to_be_merged.push_back(Id2lanelet[ind2Id[sub_ind]]);
        original.push_back(Id2lanelet[ind2Id[sub_ind]]);
      }
      merged.push_back(lanelet::utils::combineLaneletsShape(to_be_merged));
    }
  }
  return {merged, originals};
}

bool isOverTargetIndex(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, const size_t closest_idx,
  const geometry_msgs::msg::Pose & current_pose, const size_t target_idx)
{
  if (closest_idx == target_idx) {
    const geometry_msgs::msg::Pose target_pose = path.points.at(target_idx).point.pose;
    return planning_utils::isAheadOf(current_pose, target_pose);
  }
  return static_cast<bool>(closest_idx > target_idx);
}

std::optional<autoware_utils::Polygon2d> getIntersectionArea(
  lanelet::ConstLanelet assigned_lane, lanelet::LaneletMapConstPtr lanelet_map_ptr)
{
  const std::string area_id_str = assigned_lane.attributeOr("intersection_area", "else");
  if (area_id_str == "else") return std::nullopt;
  if (!std::atoi(area_id_str.c_str())) return std::nullopt;

  const lanelet::Id area_id = std::atoi(area_id_str.c_str());
  const auto polygon_opt = lanelet_map_ptr->polygonLayer.find(area_id);
  if (polygon_opt == lanelet_map_ptr->polygonLayer.end()) return std::nullopt;

  const auto poly_3d = lanelet_map_ptr->polygonLayer.get(area_id);
  Polygon2d poly{};
  for (const auto & p : poly_3d) poly.outer().emplace_back(p.x(), p.y());
  return std::make_optional(poly);
}

std::optional<InterpolatedPathInfo> generateInterpolatedPath(
  const lanelet::Id lane_id, const std::set<lanelet::Id> & associative_lane_ids,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & input_path, const double ds,
  const rclcpp::Logger logger)
{
  InterpolatedPathInfo interpolated_path_info;
  if (!splineInterpolate(input_path, ds, interpolated_path_info.path, logger)) {
    return std::nullopt;
  }
  interpolated_path_info.ds = ds;
  interpolated_path_info.lane_id = lane_id;
  interpolated_path_info.associative_lane_ids = associative_lane_ids;
  interpolated_path_info.lane_id_interval =
    findLaneIdsInterval(interpolated_path_info.path, associative_lane_ids);
  return interpolated_path_info;
}

geometry_msgs::msg::Pose getObjectPoseWithVelocityDirection(
  const autoware_perception_msgs::msg::PredictedObjectKinematics & obj_state)
{
  if (obj_state.initial_twist_with_covariance.twist.linear.x >= 0) {
    return obj_state.initial_pose_with_covariance.pose;
  }

  // When the object velocity is negative, invert orientation (yaw)
  auto obj_pose = obj_state.initial_pose_with_covariance.pose;
  double yaw, pitch, roll;
  tf2::getEulerYPR(obj_pose.orientation, yaw, pitch, roll);
  tf2::Quaternion inv_q;
  inv_q.setRPY(roll, pitch, yaw + M_PI);
  obj_pose.orientation = tf2::toMsg(inv_q);
  return obj_pose;
}

std::vector<lanelet::CompoundPolygon3d> getPolygon3dFromLanelets(
  const lanelet::ConstLanelets & ll_vec)
{
  std::vector<lanelet::CompoundPolygon3d> polys;
  for (auto && ll : ll_vec) {
    polys.push_back(ll.polygon3d());
  }
  return polys;
}

static std::optional<size_t> find_maximum_footprint_overshoot_position_impl(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const autoware_utils::LinearRing2d & local_footprint,
  const lanelet::ConstLanelet & merging_target_lanelet, const double min_distance_threshold,
  const size_t search_start_idx, const size_t search_end_idx)
{
  if (search_start_idx >= path.points.size() || search_end_idx >= path.points.size()) {
    return std::nullopt;
  }
  const std::vector lane_boundaries = {
    lanelet::utils::to2D(merging_target_lanelet.rightBound().basicLineString()),
    lanelet::utils::to2D(merging_target_lanelet.leftBound().basicLineString())};

  std::optional<std::pair<size_t, lanelet::BasicLineString2d>>
    ego_entry_index_and_non_target_boundary{std::nullopt};
  for (unsigned i = search_start_idx; i <= search_end_idx; ++i) {
    const auto & base_pose = path.points.at(i).point.pose;
    const auto footprint =
      autoware_utils::transform_vector(local_footprint, autoware_utils::pose2transform(base_pose));
    if (boost::geometry::intersects(footprint, lane_boundaries.at(0))) {
      ego_entry_index_and_non_target_boundary = std::make_pair(i, lane_boundaries.at(1));
      break;
    }
    if (boost::geometry::intersects(footprint, lane_boundaries.at(1))) {
      ego_entry_index_and_non_target_boundary = std::make_pair(i, lane_boundaries.at(0));
      break;
    }
  }
  if (!ego_entry_index_and_non_target_boundary) {
    return std::nullopt;
  }

  const auto & [search_start_index, target_boundary] =
    ego_entry_index_and_non_target_boundary.value();
  double closest_dist = std::numeric_limits<double>::infinity();
  std::optional<size_t> closest_index{std::nullopt};
  for (unsigned i = search_start_index; i <= search_end_idx; ++i) {
    const auto & base_pose = path.points.at(i).point.pose;
    const auto footprint =
      autoware_utils::transform_vector(local_footprint, autoware_utils::pose2transform(base_pose));
    if (boost::geometry::intersects(footprint, target_boundary)) {
      return i;
    }

    double footprint_to_boundary_distance = std::numeric_limits<double>::infinity();
    for (const auto & p : footprint) {
      const double dist = boost::geometry::distance(p, target_boundary);
      if (dist < footprint_to_boundary_distance) {
        footprint_to_boundary_distance = dist;
      }
    }
    if (footprint_to_boundary_distance < closest_dist) {
      closest_dist = footprint_to_boundary_distance;
      closest_index = i;
    }
    if (footprint_to_boundary_distance < min_distance_threshold) {
      return closest_index;
    }
  }
  return closest_index;
}

std::optional<size_t> find_maximum_footprint_overshoot_position(
  const InterpolatedPathInfo & interpolated_path_info,
  const autoware_utils::LinearRing2d & local_footprint,
  const lanelet::ConstLanelet & merging_lanelet, const double min_distance_threshold,
  const std::string & turn_direction, const size_t search_start_idx)
{
  if (turn_direction != "left" && turn_direction != "right") {
    return std::nullopt;
  }

  const auto & path = interpolated_path_info.path;
  const auto & [_, intersection_end] = interpolated_path_info.lane_id_interval.value();
  return find_maximum_footprint_overshoot_position_impl(
    path, local_footprint, merging_lanelet, min_distance_threshold, search_start_idx,
    intersection_end);
}

std::string to_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +uuid.uuid[i];
  }
  return ss.str();
}

autoware_utils::Polygon2d createOneStepPolygon(
  const geometry_msgs::msg::Pose & prev_pose, const geometry_msgs::msg::Pose & next_pose,
  const autoware_perception_msgs::msg::Shape & shape)
{
  namespace bg = boost::geometry;
  const auto prev_poly = autoware_utils::to_polygon2d(prev_pose, shape);
  const auto next_poly = autoware_utils::to_polygon2d(next_pose, shape);

  autoware_utils::Polygon2d one_step_poly;
  for (const auto & point : prev_poly.outer()) {
    one_step_poly.outer().push_back(point);
  }
  for (const auto & point : next_poly.outer()) {
    one_step_poly.outer().push_back(point);
  }

  bg::correct(one_step_poly);

  autoware_utils::Polygon2d convex_one_step_poly;
  bg::convex_hull(one_step_poly, convex_one_step_poly);

  return convex_one_step_poly;
}

lanelet::ConstLanelets getPrevLanelets(
  const lanelet::ConstLanelets & lanelets_on_path, const std::set<lanelet::Id> & associative_ids)
{
  lanelet::ConstLanelets previous_lanelets;
  for (const auto & ll : lanelets_on_path) {
    if (associative_ids.find(ll.id()) != associative_ids.end()) {
      return previous_lanelets;
    }
    previous_lanelets.push_back(ll);
  }
  return previous_lanelets;
}

// end inclusive
lanelet::ConstLanelet generatePathLanelet(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, const size_t start_idx,
  const size_t end_idx, const double width, const double interval)
{
  lanelet::Points3d lefts;
  lanelet::Points3d rights;
  size_t prev_idx = start_idx;
  for (size_t i = start_idx; i <= end_idx; ++i) {
    const auto & p = path.points.at(i).point.pose;
    const auto & p_prev = path.points.at(prev_idx).point.pose;
    if (i != start_idx && autoware_utils::calc_distance2d(p_prev, p) < interval) {
      continue;
    }
    prev_idx = i;
    const double yaw = tf2::getYaw(p.orientation);
    const double x = p.position.x;
    const double y = p.position.y;
    // NOTE: maybe this is opposite
    const double left_x = x + width / 2 * std::sin(yaw);
    const double left_y = y - width / 2 * std::cos(yaw);
    const double right_x = x - width / 2 * std::sin(yaw);
    const double right_y = y + width / 2 * std::cos(yaw);
    lefts.emplace_back(lanelet::InvalId, left_x, left_y, p.position.z);
    rights.emplace_back(lanelet::InvalId, right_x, right_y, p.position.z);
  }
  lanelet::LineString3d left = lanelet::LineString3d(lanelet::InvalId, lefts);
  lanelet::LineString3d right = lanelet::LineString3d(lanelet::InvalId, rights);

  return lanelet::Lanelet(lanelet::InvalId, left, right);
}

std::optional<std::pair<size_t, const lanelet::CompoundPolygon3d &>> getFirstPointInsidePolygons(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const std::pair<size_t, size_t> lane_interval,
  const std::vector<lanelet::CompoundPolygon3d> & polygons, const bool search_forward)
{
  if (search_forward) {
    for (size_t i = lane_interval.first; i <= lane_interval.second; ++i) {
      bool is_in_lanelet = false;
      const auto & p = path.points.at(i).point.pose.position;
      for (const auto & polygon : polygons) {
        const auto polygon_2d = lanelet::utils::to2D(polygon);
        is_in_lanelet = bg::within(autoware::behavior_velocity_planner::to_bg2d(p), polygon_2d);
        if (is_in_lanelet) {
          return std::make_optional<std::pair<size_t, const lanelet::CompoundPolygon3d &>>(
            i, polygon);
        }
      }
      if (is_in_lanelet) {
        break;
      }
    }
  } else {
    for (size_t i = lane_interval.second; i >= lane_interval.first; --i) {
      bool is_in_lanelet = false;
      const auto & p = path.points.at(i).point.pose.position;
      for (const auto & polygon : polygons) {
        const auto polygon_2d = lanelet::utils::to2D(polygon);
        is_in_lanelet = bg::within(autoware::behavior_velocity_planner::to_bg2d(p), polygon_2d);
        if (is_in_lanelet) {
          return std::make_optional<std::pair<size_t, const lanelet::CompoundPolygon3d &>>(
            i, polygon);
        }
      }
      if (is_in_lanelet) {
        break;
      }
      if (i == 0) {
        break;
      }
    }
  }
  return std::nullopt;
}

double getHighestCurvature(const lanelet::ConstLineString3d & centerline)
{
  std::vector<geometry_msgs::msg::Point> points;
  for (auto point = centerline.begin(); point != centerline.end(); point++) {
    geometry_msgs::msg::Point ros_point;
    ros_point.x = point->x();
    ros_point.y = point->y();
    ros_point.z = point->z();
    points.push_back(ros_point);
  }

  autoware::interpolation::SplineInterpolationPoints2d interpolation(points);
  const std::vector<double> curvatures = interpolation.getSplineInterpolatedCurvatures();
  std::vector<double> curvatures_positive;
  for (const auto & curvature : curvatures) {
    curvatures_positive.push_back(std::fabs(curvature));
  }
  return *std::max_element(curvatures_positive.begin(), curvatures_positive.end());
}

}  // namespace autoware::behavior_velocity_planner::util
