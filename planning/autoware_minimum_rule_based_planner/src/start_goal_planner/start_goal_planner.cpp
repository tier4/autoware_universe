// Copyright 2022 TIER IV, Inc.
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

#include "start_goal_planner.hpp"

#include "../path_planner.hpp"
#include "../type_alias.hpp"
#include "clothoid_pull_generater.hpp"

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/trajectory/utils/crop.hpp>
#include <autoware/trajectory/utils/find_intervals.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils_debug/time_keeper.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
namespace
{
void interpolate_lane_ids(
  const PathPointTrajectory & trajectory, std::vector<PathPointWithLaneId> & points)
{
  for (auto & p : points) {
    const auto s = autoware::experimental::trajectory::closest(trajectory, p.point.pose);
    p.lane_ids = trajectory.compute(s).lane_ids;
  }
}

std::vector<PathPointWithLaneId> generate_trajectory_from_points(
  std::vector<geometry_msgs::msg::Point> points, PathPointWithLaneId goal)
{
  std::vector<PathPointWithLaneId> trajectory;
  if (points.size() < 2) {
    return trajectory;
  }
  for (size_t i = 1; i + 1 < points.size(); ++i) {
    const auto & prev = points[i - 1];
    const auto & curr = points[i];
    PathPointWithLaneId point = goal;
    point.point.pose.position = curr;
    point.point.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(std::atan2(curr.y - prev.y, curr.x - prev.x));
    trajectory.push_back(point);
  }
  return trajectory;
}

std::vector<PathPointWithLaneId> calc_goal_planner_start_poses(
  const PathPointTrajectory & trajectory, const Pose & goal_pose, const lanelet::Id & goal_lane_id,
  const StartGoalPlannerParams & params, const Pose & ego_pose)
{
  auto contain_goal_lane_id = [&](const PathPointWithLaneId & point) {
    const auto & ids = point.lane_ids;
    return std::find(ids.begin(), ids.end(), goal_lane_id) != ids.end();
  };

  std::vector<PathPointWithLaneId> candidates;
  const auto & search_radius_range = params.search_radius_range;
  const auto & pre_goal_offset = params.pre_goal_offset;

  auto closest_to_goal = autoware::experimental::trajectory::closest_with_constraint(
    trajectory, goal_pose, contain_goal_lane_id);

  // If no point with the goal lane ID exists in the trajectory (e.g. goal is on an adjacent
  // lane), fall back to the geometrically closest point so the goal connection still applies.
  if (!closest_to_goal) {
    closest_to_goal = autoware::experimental::trajectory::closest(trajectory, goal_pose);
  }

  auto cropped_path = autoware::experimental::trajectory::crop(trajectory, 0, *closest_to_goal);

  for (double s = search_radius_range; s > 0; s -= 2) {
    auto outside_circle = [&](const PathPointWithLaneId & point) {
      return autoware_utils::calc_distance2d(point.point.pose, goal_pose) > s;
    };

    auto intervals =
      autoware::experimental::trajectory::find_intervals(cropped_path, outside_circle, 10);

    std::vector<PathPointWithLaneId> goal_connected_trajectory_points;

    if (!intervals.empty()) {
      auto cropped =
        autoware::experimental::trajectory::crop(cropped_path, 0, intervals.back().end);
      goal_connected_trajectory_points = cropped.restore();
    } else if (cropped_path.length() > pre_goal_offset) {
      // If distance from start to goal is smaller than refine_goal_search_radius_range and start is
      // farther from goal than pre_goal, we just connect start, pre_goal, and goal.
      goal_connected_trajectory_points = {cropped_path.compute(0)};
    }

    candidates.push_back(
      goal_connected_trajectory_points.empty() ? cropped_path.compute(0)
                                               : goal_connected_trajectory_points.back());
  }

  auto ego_candidate =
    trajectory.compute(autoware::experimental::trajectory::closest(trajectory, ego_pose));
  ego_candidate.point.pose = ego_pose;
  candidates.push_back(ego_candidate);

  return candidates;
}

bool is_point_in_polygons(
  const autoware_utils_geometry::Point2d & point,
  const std::vector<lanelet::BasicPolygon2d> & polygons)
{
  return std::any_of(polygons.begin(), polygons.end(), [&](const auto & polygon) {
    return boost::geometry::covered_by(point, polygon);
  });
}

bool is_footprints_inside_polygons(
  const std::vector<PathPointWithLaneId> & points,
  const std::vector<lanelet::BasicPolygon2d> & polygons,
  const autoware_utils_geometry::LinearRing2d & base_footprint,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper);
  auto footprint_is_in_lanelet_or_areas = [&](const geometry_msgs::msg::Pose & pose) {
    const auto footprint =
      autoware_utils::transform_vector(base_footprint, autoware_utils::pose2transform(pose));
    for (const auto & point : footprint) {
      if (!is_point_in_polygons(point, polygons)) {
        return false;
      }
    }
    return true;
  };

  return std::none_of(points.begin(), points.end(), [&](const auto & point) {
    return !footprint_is_in_lanelet_or_areas(point.point.pose);
  });
}

lanelet::ConstPolygons3d find_polygons_intersecting_lanelet(
  const lanelet::PolygonLayer & polygon_layer, const lanelet::ConstLanelet & lanelet)
{
  lanelet::ConstPolygons3d result;

  const auto lane_2d = lanelet.polygon2d().basicPolygon();

  const auto bbox = lanelet::geometry::boundingBox2d(lanelet);
  const auto candidate_polygons = polygon_layer.search(bbox);

  for (const auto & polygon : candidate_polygons) {
    const auto poly_2d = lanelet::utils::to2D(polygon).basicPolygon();
    if (boost::geometry::intersects(lane_2d, poly_2d)) {
      result.push_back(polygon);
    }
  }
  return result;
}

std::pair<double, double> cal_curvature(
  const PathPointTrajectory & trajectory,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper);
  const auto ss = trajectory.get_underlying_bases();
  if (ss.size() < 3) {
    return {0.0, 0.0};
  }
  const auto curvature_vec = trajectory.curvature(ss);
  double max_curvature = 0.0;
  double curvature_integral = 0.0;
  for (size_t i = 0; i < ss.size() - 1; ++i) {
    const double ds = ss[i + 1] - ss[i];
    const double curvature =
      (curvature_vec[i] * curvature_vec[i] + curvature_vec[i + 1] * curvature_vec[i + 1]) * 0.5;
    if (max_curvature < curvature) {
      max_curvature = curvature;
    }
    curvature_integral += curvature * ds;
  }
  return {curvature_integral, max_curvature};
}

bool has_turn_point(const std::vector<PathPointWithLaneId> & points)
{
  std::optional<double> theta_prev = std::nullopt;

  if (points.size() < 2) {
    return false;
  }

  for (size_t i = 0; i < points.size() - 1; ++i) {
    const auto dx = points[i + 1].point.pose.position.x - points[i].point.pose.position.x;
    const auto dy = points[i + 1].point.pose.position.y - points[i].point.pose.position.y;
    const auto theta = std::atan2(dy, dx);
    if (theta_prev.has_value()) {
      const double d_theta = std::abs(autoware_utils::normalize_radian(theta - *theta_prev));
      constexpr double turn_point_th = M_PI / 2;
      if (d_theta > turn_point_th) {
        return true;
      }
    }
    theta_prev = theta;
  }
  return false;
}

std::vector<PathPointWithLaneId> downsample_trajectory_points(
  const PathPointTrajectory & trajectory, const size_t num_points)
{
  std::vector<PathPointWithLaneId> points;
  points.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    const double s =
      trajectory.length() * static_cast<double>(i) / static_cast<double>(num_points - 1);
    points.push_back(trajectory.compute(s));
  }
  return points;
}

double cal_trajectory_diff(
  const std::vector<PathPointWithLaneId> & points_a, const double length_a,
  const std::vector<PathPointWithLaneId> & points_b, const double length_b,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper);
  if (points_a.empty() || points_b.empty()) {
    return 0.0;
  }

  const bool a_is_longer = length_a >= length_b;
  const auto & base_traj_points = a_is_longer ? points_a : points_b;
  const auto & short_traj_points = a_is_longer ? points_b : points_a;

  if (short_traj_points.size() == 1) {
    return std::abs(
      autoware::motion_utils::calcLateralOffset(
        base_traj_points, short_traj_points.front().point.pose.position));
  }

  std::vector<double> dist_vec;
  dist_vec.reserve(short_traj_points.size());
  for (const auto & point : short_traj_points) {
    const double offset =
      autoware::motion_utils::calcLateralOffset(base_traj_points, point.point.pose.position);
    dist_vec.push_back(std::isnan(offset) ? 0.0 : std::abs(offset));
  }

  double dist_integral = 0.0;
  for (size_t i = 0; i < short_traj_points.size() - 1; ++i) {
    const double ds = autoware_utils::calc_distance2d(
      short_traj_points[i].point.pose.position, short_traj_points[i + 1].point.pose.position);
    const double dist = (dist_vec[i] * dist_vec[i] + dist_vec[i + 1] * dist_vec[i + 1]) * 0.5;
    dist_integral += dist * ds;
  }
  return dist_integral;
}

}  // namespace

StartGoalPlanner::StartGoalPlanner(
  const rclcpp::Logger & logger, std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
  const StartGoalPlannerParams & params, const VehicleInfo & vehicle_info)
: logger_(logger),
  time_keeper_(std::move(time_keeper)),
  params_(params),
  vehicle_info_(vehicle_info)
{
}

void StartGoalPlanner::set_route_data(const RouteData & route_data)
{
  route_data_ = route_data;
}

void StartGoalPlanner::update_params(const StartGoalPlannerParams & params)
{
  params_ = params;
}

std::optional<PathPointTrajectory> StartGoalPlanner::plan(
  const PathPointTrajectory & trajectory, lanelet::ConstLanelet & current_lanelet,
  const double & s_path_end, const geometry_msgs::msg::Pose & ego_pose)
{
  const auto available_area = get_available_area(trajectory);

  judge_goal_planner_act(trajectory, s_path_end, available_area);
  judge_start_planner_act(current_lanelet, ego_pose);

  if (!start_planner_act_ && !goal_planner_act_) {
    return std::nullopt;  // StartGoalPlanner is not applied. normal termination
  }

  const auto goal_pose_candidates = get_goal_pose(trajectory, ego_pose);
  const auto start_pose_candidates = get_start_pose(trajectory, ego_pose);
  if (!start_pose_candidates || !goal_pose_candidates) {
    return generate_fallback_trajectory(trajectory);
  }

  if (available_area.empty()) {
    return generate_fallback_trajectory(trajectory);
  }

  const auto candidate_trajectories =
    generate_pull_trajectories(*start_pose_candidates, *goal_pose_candidates);
  if (!candidate_trajectories) {
    return generate_fallback_trajectory(trajectory);
  }

  const auto pull_trajectory =
    evaluate_trajectory(*candidate_trajectories, available_area, ego_pose);
  if (!pull_trajectory) {
    return generate_fallback_trajectory(trajectory);
  }

  const auto refined_trajectory = connect_pull_trajectory(trajectory, *pull_trajectory);
  if (!refined_trajectory) {
    return generate_fallback_trajectory(trajectory);
  }
  generated_trajectory_ = refined_trajectory;
  return generated_trajectory_;
}

void StartGoalPlanner::judge_start_planner_act(
  const lanelet::ConstLanelet & current_lanelet, const geometry_msgs::msg::Pose & ego_pose)
{
  const double lateral_offset =
    autoware::experimental::lanelet2_utils::get_lateral_distance_to_centerline(
      current_lanelet, ego_pose);
  constexpr double start_planner_end_th_m = 1;

  const bool reset_condition_1 = goal_planner_act_;
  const bool reset_condition_2 = lateral_offset < start_planner_end_th_m;
  const bool set_condition_1 = [&]() {
    lanelet::ConstLanelets candidates = route_data_.start_lanelets;
    for (const auto & ll : route_data_.start_lanelets) {
      for (const auto & right :
           route_data_.lanelet_map_ptr->laneletLayer.findUsages(ll.rightBound())) {
        if (lanelet::geometry::inside(right, {ego_pose.position.x, ego_pose.position.y})) {
          return true;
        }
      }
      for (const auto & left :
           route_data_.lanelet_map_ptr->laneletLayer.findUsages(ll.leftBound())) {
        if (lanelet::geometry::inside(left, {ego_pose.position.x, ego_pose.position.y})) {
          return true;
        }
      }
    }
    return false;
  }();
  const bool set_condition_2 = [&]() {
    for (const auto & right :
         route_data_.lanelet_map_ptr->laneletLayer.findUsages(current_lanelet.rightBound())) {
      if (lanelet::geometry::inside(right, {ego_pose.position.x, ego_pose.position.y})) {
        return true;
      }
    }
    for (const auto & left :
         route_data_.lanelet_map_ptr->laneletLayer.findUsages(current_lanelet.leftBound())) {
      if (lanelet::geometry::inside(left, {ego_pose.position.x, ego_pose.position.y})) {
        return true;
      }
    }
    return false;
  }();

  if (reset_condition_1 || reset_condition_2) {
    if (start_planner_act_) {
      generated_trajectory_ = std::nullopt;
    }
    start_planner_act_ = false;
  } else if (set_condition_1 && set_condition_2) {
    start_planner_act_ = true;
  }
}

void StartGoalPlanner::judge_goal_planner_act(
  const PathPointTrajectory & trajectory, const double & s_path_end,
  const std::vector<lanelet::BasicPolygon2d> & available_area)
{
  const auto s_path_end_clamped = std::min(trajectory.length(), s_path_end);
  const auto distance_to_goal =
    autoware_utils::calc_distance2d(trajectory.compute(s_path_end_clamped), route_data_.goal_pose);
  const bool goal_pose_in_available_area = is_point_in_polygons(
    autoware_utils_geometry::Point2d{
      route_data_.goal_pose.position.x, route_data_.goal_pose.position.y},
    available_area);

  if (!goal_pose_prev_.has_value()) {
    goal_pose_prev_ = route_data_.goal_pose;
  }

  if (autoware_utils_geometry::calc_distance2d(*goal_pose_prev_, route_data_.goal_pose) > 1e-3) {
    goal_planner_act_ = false;
    generated_trajectory_ = std::nullopt;
  } else if (!goal_planner_act_ || !generated_trajectory_.has_value()) {
    goal_planner_act_ =
      distance_to_goal < params_.search_radius_range && goal_pose_in_available_area;
  }
  goal_pose_prev_ = route_data_.goal_pose;
}

std::vector<lanelet::BasicPolygon2d> StartGoalPlanner::get_available_area(
  const PathPointTrajectory & trajectory)
{
  const auto base_lanelets =
    utils::extract_lanelets_from_trajectory(trajectory, route_data_.lanelet_map_ptr);
  const auto & lanelet_map_ptr = route_data_.lanelet_map_ptr;
  const auto & routing_graph_ptr = route_data_.routing_graph_ptr;

  lanelet::ConstLanelets lanelets = base_lanelets;
  std::unordered_set<lanelet::Id> added_area_ids;
  std::vector<lanelet::BasicPolygon2d> polygons;

  for (const auto & ll : base_lanelets) {
    polygons.push_back(ll.polygon2d().basicPolygon());
  }

  auto try_add_lanelet = [&](const lanelet::ConstLanelet & candidate) {
    if (std::find(lanelets.begin(), lanelets.end(), candidate) == lanelets.end()) {
      lanelets.push_back(candidate);
      polygons.push_back(candidate.polygon2d().basicPolygon());
    }
  };

  auto try_add_area = [&](const lanelet::ConstPolygon3d & candidate) {
    if (added_area_ids.insert(candidate.id()).second) {
      polygons.push_back(lanelet::utils::to2D(candidate).basicPolygon());
    }
  };

  // Lanelets reachable from base_lanelets by a lane change.
  for (const auto & ll : base_lanelets) {
    for (const auto & neighbor : routing_graph_ptr->besides(ll)) {
      try_add_lanelet(neighbor);
    }
  }

  // Road-shoulder lanelets sharing a boundary linestring with the lanelets collected so far.
  const lanelet::ConstLanelets lanelets_after_lane_change = lanelets;
  for (const auto & ll : lanelets_after_lane_change) {
    for (const auto & other : lanelet_map_ptr->laneletLayer.findUsages(ll.leftBound())) {
      if (
        other.rightBound() == ll.leftBound() &&
        autoware::experimental::lanelet2_utils::is_shoulder_lane(other)) {
        try_add_lanelet(other);
      }
    }
    for (const auto & other : lanelet_map_ptr->laneletLayer.findUsages(ll.rightBound())) {
      if (
        other.leftBound() == ll.rightBound() &&
        autoware::experimental::lanelet2_utils::is_shoulder_lane(other)) {
        try_add_lanelet(other);
      }
    }
  }

  // Freespace areas sharing a boundary linestring with the lanelets collected so far.
  for (const auto & ll : lanelets) {
    for (const auto & area :
         find_polygons_intersecting_lanelet(lanelet_map_ptr->polygonLayer, ll)) {
      const std::string type = area.hasAttribute(lanelet::AttributeName::Type)
                                 ? area.attribute(lanelet::AttributeName::Type).value()
                                 : "";
      if (available_area_type.count(type)) {
        try_add_area(area);
      }
    }
  }

  return polygons;
}

std::optional<std::vector<PathPointWithLaneId>> StartGoalPlanner::get_start_pose(
  const PathPointTrajectory & trajectory, const geometry_msgs::msg::Pose & ego_pose)
{
  if (goal_planner_act_) {
    const auto goal_lane_id = route_data_.preferred_lanelets.back().id();
    auto candidates = calc_goal_planner_start_poses(
      trajectory, route_data_.goal_pose, goal_lane_id, params_, ego_pose);
    if (candidates.empty()) {
      return std::nullopt;
    }
    return candidates;
  }
  if (start_planner_act_) {
    auto start_point =
      trajectory.compute(autoware::experimental::trajectory::closest(trajectory, ego_pose));
    start_point.point.pose = ego_pose;
    return std::vector{start_point};
  }
  return std::nullopt;
}

std::optional<std::vector<PathPointWithLaneId>> StartGoalPlanner::get_goal_pose(
  const PathPointTrajectory & trajectory, const geometry_msgs::msg::Pose & ego_pose)
{
  if (goal_planner_act_) {
    const auto pre_goal_pose =
      autoware_utils::calc_offset_pose(route_data_.goal_pose, -params_.pre_goal_offset, 0.0, 0.0);
    auto pre_goal =
      trajectory.compute(autoware::experimental::trajectory::closest(trajectory, pre_goal_pose));
    pre_goal.point.pose = pre_goal_pose;
    return std::vector<PathPointWithLaneId>{pre_goal};
  }
  if (start_planner_act_) {
    double start_s = autoware::experimental::trajectory::closest(trajectory, ego_pose);
    std::vector<PathPointWithLaneId> goal_pose_candidates;
    for (int num_points = 1; num_points < 20; ++num_points) {
      goal_pose_candidates.push_back(trajectory.compute(start_s));
      start_s += 1.5;
    }
    if (!goal_pose_candidates.empty()) {
      return goal_pose_candidates;
    }
    return std::nullopt;
  }
  return std::nullopt;
}

std::optional<std::vector<PathPointTrajectory>> StartGoalPlanner::generate_pull_trajectories(
  const std::vector<PathPointWithLaneId> & start_pose_candidates,
  const std::vector<PathPointWithLaneId> & goal_pose_candidates)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto max_steer_angles_rad = generate_candidate_steer_angles_rad(
    vehicle_info_.max_steer_angle_rad, params_.clothoid_steer_angle_trial_count);
  const auto max_steer_angle_rate_rad_per_sec =
    autoware_utils::deg2rad(params_.clothoid_max_steer_angle_rate_deg_per_sec);

  std::vector<PathPointTrajectory> candidate_trajectories;

  for (const auto & start_point : start_pose_candidates) {
    for (const auto & goal_point : goal_pose_candidates) {
      for (const auto max_steering_angle : max_steer_angles_rad) {
        const auto clothoid_paths = plan_clothoid_pull(
          start_point.point.pose, goal_point.point.pose, vehicle_info_.wheel_base_m,
          max_steering_angle, max_steer_angle_rate_rad_per_sec,
          params_.clothoid_reference_velocity);

        if (!clothoid_paths.has_value()) {
          continue;
        }

        for (const auto & clothoid_points : *clothoid_paths) {
          std::vector<PathPointWithLaneId> trajectory = {start_point};
          const auto interior_points = generate_trajectory_from_points(clothoid_points, goal_point);
          trajectory.insert(trajectory.end(), interior_points.begin(), interior_points.end());
          trajectory.push_back(goal_point);

          if (has_turn_point(trajectory)) {
            continue;
          }

          if (const auto output = autoware::experimental::trajectory::pretty_build(trajectory)) {
            candidate_trajectories.push_back(*output);
          }
        }
      }
    }
  }

  if (candidate_trajectories.empty()) {
    return std::nullopt;
  }
  return candidate_trajectories;
}

std::optional<PathPointTrajectory> StartGoalPlanner::evaluate_trajectory(
  const std::vector<PathPointTrajectory> & candidate_trajectories,
  const std::vector<lanelet::BasicPolygon2d> & available_area,
  const geometry_msgs::msg::Pose & ego_pose)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const double feasible_curvature =
    std::tan(vehicle_info_.max_steer_angle_rad) / vehicle_info_.wheel_base_m;
  const auto base_footprint = vehicle_info_.createFootprint();

  constexpr size_t num_sample_trajectory_diff = 5;
  constexpr size_t num_sample_footprint_check = 5;
  const auto traj_points_prev =
    generated_trajectory_.has_value()
      ? downsample_trajectory_points(*generated_trajectory_, num_sample_trajectory_diff)
      : std::vector<PathPointWithLaneId>{};

  double best_score = std::numeric_limits<double>::max();
  std::optional<PathPointTrajectory> best_trajectory = std::nullopt;
  for (const auto & candidate : candidate_trajectories) {
    const double start_angle = tf2::getYaw(candidate.compute(0.0).point.pose.orientation);
    const double ego_yaw = tf2::getYaw(ego_pose.orientation);
    if (std::abs(autoware_utils::normalize_radian(start_angle - ego_yaw)) > M_PI / 2) {
      continue;
    }

    const auto [curvature_integral, max_curvature] = cal_curvature(candidate, time_keeper_);
    if (max_curvature > feasible_curvature) {
      continue;
    }

    if (!is_footprints_inside_polygons(
          downsample_trajectory_points(candidate, num_sample_footprint_check), available_area,
          base_footprint, time_keeper_)) {
      continue;
    }

    const double arc_length = candidate.length();
    if (arc_length < 1e-9) {
      continue;
    }

    const double trajectory_diff =
      generated_trajectory_.has_value()
        ? cal_trajectory_diff(
            traj_points_prev, generated_trajectory_->length(),
            downsample_trajectory_points(candidate, num_sample_trajectory_diff), candidate.length(),
            time_keeper_)
        : 0.0;

    const double score = curvature_integral / feasible_curvature * feasible_curvature * 0.4 +
                         arc_length / params_.search_radius_range * 0.3 + trajectory_diff * 0.3;
    if (score < best_score) {
      best_score = score;
      best_trajectory = candidate;
    }
  }
  return best_trajectory;
}

std::optional<PathPointTrajectory> StartGoalPlanner::connect_pull_trajectory(
  const PathPointTrajectory & trajectory, const PathPointTrajectory & pull_trajectory)
{
  if (goal_planner_act_) {
    return connect_goal_planner_trajectory(trajectory, pull_trajectory);
  }
  if (start_planner_act_) {
    return connect_start_planner_trajectory(trajectory, pull_trajectory);
  }
  return std::nullopt;
}

std::optional<PathPointTrajectory> StartGoalPlanner::connect_start_planner_trajectory(
  const PathPointTrajectory & trajectory, const PathPointTrajectory & pull_trajectory)
{
  auto pull_points = pull_trajectory.restore();
  interpolate_lane_ids(trajectory, pull_points);
  const auto pull_end_pose = pull_points.back().point.pose;
  const auto s_closest = autoware::experimental::trajectory::closest(trajectory, pull_end_pose);
  auto base_points =
    autoware::experimental::trajectory::crop(trajectory, s_closest, trajectory.length()).restore();
  constexpr double kDuplicatePointEpsilon = 1e-3;
  if (
    !base_points.empty() &&
    autoware_utils::calc_distance2d(base_points.front().point.pose, pull_end_pose) <
      kDuplicatePointEpsilon) {
    pull_points.pop_back();
  }
  pull_points.insert(pull_points.end(), base_points.begin(), base_points.end());
  auto connected_trajectory = autoware::experimental::trajectory::pretty_build(pull_points);
  if (!connected_trajectory) {
    return std::nullopt;
  }
  connected_trajectory->align_orientation_with_trajectory_direction();
  return connected_trajectory;
}

std::optional<PathPointTrajectory> StartGoalPlanner::connect_goal_planner_trajectory(
  const PathPointTrajectory & trajectory, const PathPointTrajectory & pull_trajectory)
{
  auto pull_points = pull_trajectory.restore();
  interpolate_lane_ids(trajectory, pull_points);

  auto goal = trajectory.compute(
    autoware::experimental::trajectory::closest(trajectory, route_data_.goal_pose));
  goal.point.pose = route_data_.goal_pose;
  goal.point.longitudinal_velocity_mps = 0.0;
  pull_points.push_back(goal);

  const auto pull_start_pose = pull_trajectory.compute(0.0).point.pose;
  const auto s_closest = autoware::experimental::trajectory::closest(trajectory, pull_start_pose);

  auto base_points = autoware::experimental::trajectory::crop(trajectory, 0, s_closest).restore();

  constexpr double kDuplicatePointEpsilon = 1e-3;
  if (
    !base_points.empty() &&
    autoware_utils::calc_distance2d(base_points.back().point.pose, pull_start_pose) <
      kDuplicatePointEpsilon) {
    base_points.pop_back();
  }

  base_points.insert(base_points.end(), pull_points.begin(), pull_points.end());

  auto connected_trajectory = autoware::experimental::trajectory::pretty_build(base_points);
  if (!connected_trajectory) {
    return std::nullopt;
  }
  connected_trajectory->align_orientation_with_trajectory_direction();
  return connected_trajectory;
}

std::optional<PathPointTrajectory> StartGoalPlanner::generate_fallback_trajectory(
  const PathPointTrajectory & trajectory)
{
  if (goal_planner_act_) {
    return generated_trajectory_;
  }

  if (start_planner_act_) {
    if (generated_trajectory_.has_value()) {
      return generated_trajectory_;
    }

    auto points = trajectory.restore();
    for (auto & point : points) {
      point.point.longitudinal_velocity_mps = 0.0;
      point.point.lateral_velocity_mps = 0.0;
    }

    return autoware::experimental::trajectory::pretty_build(points);
  }
  return std::nullopt;
}

}  // namespace autoware::minimum_rule_based_planner
