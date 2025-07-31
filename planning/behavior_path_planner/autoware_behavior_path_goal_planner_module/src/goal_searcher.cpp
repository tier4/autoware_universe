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

#include "autoware/behavior_path_goal_planner_module/goal_searcher.hpp"

#include "autoware/behavior_path_goal_planner_module/util.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware_lanelet2_extension/regulatory_elements/bus_stop_area.hpp"
#include "autoware_lanelet2_extension/regulatory_elements/no_parking_area.hpp"
#include "autoware_lanelet2_extension/regulatory_elements/no_stopping_area.hpp"
#include "autoware_lanelet2_extension/utility/query.hpp"
#include "autoware_lanelet2_extension/utility/utilities.hpp"
#include "autoware_utils/geometry/boost_polygon_utils.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <boost/geometry/algorithms/union.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{

std::vector<Pose> resampleBoundaryWithinRange(
  const lanelet::ConstLineString3d & boundary, const Pose & reference_pose,
  const double backward_length, const double forward_length, const double interval)
{
  using autoware::motion_utils::findNearestIndex;
  using autoware_utils::calc_azimuth_angle;
  using autoware_utils::calc_distance2d;
  using autoware_utils::create_quaternion_from_yaw;

  if (boundary.size() < 2) {
    return {};
  }

  // Convert linestring to pose vector for easier manipulation
  std::vector<Pose> boundary_poses;
  boundary_poses.reserve(boundary.size());

  for (size_t i = 0; i < boundary.size(); ++i) {
    Pose pose;
    pose.position.x = boundary[i].x();
    pose.position.y = boundary[i].y();
    pose.position.z = boundary[i].z();

    // Calculate orientation based on direction to next point
    if (i < boundary.size() - 1) {
      geometry_msgs::msg::Point next_point;
      next_point.x = boundary[i + 1].x();
      next_point.y = boundary[i + 1].y();
      next_point.z = boundary[i + 1].z();

      const auto azimuth = calc_azimuth_angle(pose.position, next_point);
      pose.orientation = create_quaternion_from_yaw(azimuth);
    } else if (i > 0) {
      // Use previous segment's direction for the last point
      geometry_msgs::msg::Point prev_point;
      prev_point.x = boundary[i - 1].x();
      prev_point.y = boundary[i - 1].y();
      prev_point.z = boundary[i - 1].z();

      const auto azimuth = calc_azimuth_angle(prev_point, pose.position);
      pose.orientation = create_quaternion_from_yaw(azimuth);
    }

    boundary_poses.push_back(std::move(pose));
  }

  // Find exact projection point on boundary
  const auto projection_opt = goal_planner_utils::calcClosestPose(boundary, reference_pose.position);
  if (!projection_opt) {
    return {};
  }
  const auto & projection_pose = projection_opt.value();

  // Find which segment contains the projection
  size_t segment_idx = 0;
  double min_dist_to_segment = std::numeric_limits<double>::max();
  
  for (size_t i = 0; i < boundary_poses.size() - 1; ++i) {
    const auto & p1 = boundary_poses[i].position;
    const auto & p2 = boundary_poses[i + 1].position;
    
    // Check if projection is between p1 and p2
    const double seg_length = calc_distance2d(p1, p2);
    if (seg_length > 0.0) {
      const double dot = ((projection_pose.position.x - p1.x) * (p2.x - p1.x) + 
                          (projection_pose.position.y - p1.y) * (p2.y - p1.y)) / (seg_length * seg_length);
      if (dot >= 0.0 && dot <= 1.0) {
        // Projection is within this segment
        segment_idx = i;
        break;
      }
    }
    
    // If not within segment, check distance to endpoints
    const double dist_to_p1 = calc_distance2d(projection_pose.position, p1);
    if (dist_to_p1 < min_dist_to_segment) {
      min_dist_to_segment = dist_to_p1;
      segment_idx = i;
    }
  }

  // Calculate arc length from start to projection point
  double arc_length_to_projection = 0.0;
  for (size_t i = 0; i < segment_idx; ++i) {
    arc_length_to_projection += calc_distance2d(boundary_poses[i].position, boundary_poses[i + 1].position);
  }
  
  // Add partial segment length
  arc_length_to_projection += calc_distance2d(boundary_poses[segment_idx].position, projection_pose.position);

  // Calculate total arc length of boundary
  double total_boundary_length = 0.0;
  for (size_t i = 0; i < boundary_poses.size() - 1; ++i) {
    total_boundary_length += calc_distance2d(boundary_poses[i].position, boundary_poses[i + 1].position);
  }

  // Calculate start and end arc lengths for the range
  const double start_arc_length = std::max(0.0, arc_length_to_projection - backward_length);
  const double end_arc_length = std::min(total_boundary_length, arc_length_to_projection + forward_length);

  if (end_arc_length <= start_arc_length) {
    return {};
  }

  // Resample at regular intervals within the range
  std::vector<Pose> resampled_poses;
  const double range_length = end_arc_length - start_arc_length;
  const size_t num_samples = static_cast<size_t>(std::ceil(range_length / interval)) + 1;
  resampled_poses.reserve(num_samples);

  // Build cumulative arc lengths for easier interpolation
  std::vector<double> cumulative_lengths;
  cumulative_lengths.reserve(boundary_poses.size());
  cumulative_lengths.push_back(0.0);
  
  for (size_t i = 1; i < boundary_poses.size(); ++i) {
    cumulative_lengths.push_back(
      cumulative_lengths.back() + 
      calc_distance2d(boundary_poses[i - 1].position, boundary_poses[i].position));
  }

  for (double s = 0.0; s <= range_length; s += interval) {
    const double target_arc_length = start_arc_length + s;

    // Find segment containing target arc length
    const auto seg_it = std::lower_bound(
      cumulative_lengths.begin(), cumulative_lengths.end(), target_arc_length);
    
    if (seg_it == cumulative_lengths.end()) {
      // Beyond the end, use last pose
      resampled_poses.push_back(boundary_poses.back());
      continue;
    }
    
    const size_t seg_idx = std::distance(cumulative_lengths.begin(), seg_it);
    if (seg_idx == 0) {
      // At or before start, use first pose
      resampled_poses.push_back(boundary_poses.front());
      continue;
    }

    // Interpolate within the segment
    const size_t idx1 = seg_idx - 1;
    const size_t idx2 = seg_idx;
    const double seg_start = cumulative_lengths[idx1];
    const double seg_end = cumulative_lengths[idx2];
    const double seg_length = seg_end - seg_start;

    if (seg_length > 0.0) {
      const double ratio = (target_arc_length - seg_start) / seg_length;

      // Interpolate pose
      Pose interpolated_pose;
      const auto & p1 = boundary_poses[idx1];
      const auto & p2 = boundary_poses[idx2];

      interpolated_pose.position.x = p1.position.x + ratio * (p2.position.x - p1.position.x);
      interpolated_pose.position.y = p1.position.y + ratio * (p2.position.y - p1.position.y);
      interpolated_pose.position.z = p1.position.z + ratio * (p2.position.z - p1.position.z);

      // Use the segment's orientation
      interpolated_pose.orientation = p1.orientation;

      resampled_poses.push_back(std::move(interpolated_pose));
    } else {
      resampled_poses.push_back(boundary_poses[idx1]);
    }
  }

  return resampled_poses;
}

// Helper function to calculate goal pose with minimum margin from boundary
std::optional<Pose> calculateGoalPoseWithMinMargin(
  const Pose & front_pose, const Pose & rear_pose, const lanelet::ConstLineString3d & boundary,
  const double base_link2front, const double margin_from_boundary, const bool left_side_parking)
{
  using autoware_utils::calc_offset_pose;
  // Calculate signed distance from rear pose to boundary in y direction
  // Project rear pose onto boundary to find the perpendicular distance
  const auto rear_to_boundary_pose_opt =
    goal_planner_utils::calcClosestPose(boundary, rear_pose.position);
  if (!rear_to_boundary_pose_opt) {
    return std::nullopt;
  }
  const auto & rear_to_boundary_pose = rear_to_boundary_pose_opt.value();

  // Transform boundary position to rear pose coordinate system
  const auto boundary_position_in_rear_frame =
    autoware_utils::inverse_transform_point(rear_to_boundary_pose.position, rear_pose);

  // In rear pose coordinate system, y component is the lateral distance to boundary
  const double rear_pose_to_boundary_distance = boundary_position_in_rear_frame.y;

  // Calculate goal pose based on parking side and rear pose position
  Pose pose_with_min_margin_from_boundary;

  if (left_side_parking) {
    if (rear_pose_to_boundary_distance < 0) {
      // Case 1-1: Left boundary is on the right side of rear pose
      // Need to offset right by (-rear_pose_to_boundary_distance + margin_from_boundary)
      const double offset_y = -(-rear_pose_to_boundary_distance + margin_from_boundary);
      pose_with_min_margin_from_boundary =
        calc_offset_pose(front_pose, -base_link2front, offset_y, 0.0);
    } else {
      // Case 1-2: Left boundary is on the left side of rear pose
      // Front pose is closer to boundary, offset by -margin_from_boundary
      pose_with_min_margin_from_boundary =
        calc_offset_pose(front_pose, -base_link2front, -margin_from_boundary, 0.0);
    }
  } else {
    if (rear_pose_to_boundary_distance < 0) {
      // Case 2-1: Right boundary is on the right side of rear pose
      // Front pose is closer to boundary, offset by margin_from_boundary
      pose_with_min_margin_from_boundary =
        calc_offset_pose(front_pose, -base_link2front, margin_from_boundary, 0.0);
    } else {
      // Case 2-2: Right boundary is on the left side of rear pose
      // Need to offset left by (rear_pose_to_boundary_distance + margin_from_boundary)
      const double offset_y = rear_pose_to_boundary_distance + margin_from_boundary;
      pose_with_min_margin_from_boundary =
        calc_offset_pose(front_pose, -base_link2front, offset_y, 0.0);
    }
  }

  return pose_with_min_margin_from_boundary;
}

using autoware_utils::calc_offset_pose;
using lanelet::autoware::NoParkingArea;
using lanelet::autoware::NoStoppingArea;

// Sort with smaller longitudinal distances taking precedence over smaller lateral distances.
struct SortByLongitudinalDistance
{
  bool prioritize_goals_before_objects{false};
  explicit SortByLongitudinalDistance(bool prioritize_goals_before_objects)
  : prioritize_goals_before_objects(prioritize_goals_before_objects)
  {
  }

  bool operator()(const GoalCandidate & a, const GoalCandidate & b) const noexcept
  {
    if (prioritize_goals_before_objects) {
      if (a.num_objects_to_avoid != b.num_objects_to_avoid) {
        return a.num_objects_to_avoid < b.num_objects_to_avoid;
      }
    }

    const double diff = a.distance_from_original_goal - b.distance_from_original_goal;
    constexpr double eps = 0.01;
    // If the longitudinal distances are approximately equal, sort based on lateral offset.
    if (std::abs(diff) < eps) {
      return a.lateral_offset < b.lateral_offset;
    }
    return a.distance_from_original_goal < b.distance_from_original_goal;
  }
};

// Sort with the weighted sum of the longitudinal distance and the lateral distance weighted by
// lateral_cost.
struct SortByWeightedDistance
{
  double lateral_cost{0.0};
  bool prioritize_goals_before_objects{false};

  SortByWeightedDistance(double cost, bool prioritize_goals_before_objects)
  : lateral_cost(cost), prioritize_goals_before_objects(prioritize_goals_before_objects)
  {
  }

  bool operator()(const GoalCandidate & a, const GoalCandidate & b) const noexcept
  {
    if (prioritize_goals_before_objects) {
      if (a.num_objects_to_avoid != b.num_objects_to_avoid) {
        return a.num_objects_to_avoid < b.num_objects_to_avoid;
      }
    }

    return a.distance_from_original_goal + lateral_cost * a.lateral_offset <
           b.distance_from_original_goal + lateral_cost * b.lateral_offset;
  }
};

GoalSearcher::GoalSearcher(
  const GoalPlannerParameters & parameters, const LinearRing2d & vehicle_footprint,
  const bool left_side_parking, const lanelet::ConstLanelets & pull_over_lanes,
  const lanelet::BasicPolygons2d & no_parking_area_polygons,
  const lanelet::BasicPolygons2d & no_stopping_area_polygons,
  const lanelet::BasicPolygons2d & bus_stop_area_polygons)
: parameters_(parameters),
  vehicle_footprint_(vehicle_footprint),
  left_side_parking_(left_side_parking),
  pull_over_lanes_(pull_over_lanes),
  no_parking_area_polygons_(no_parking_area_polygons),
  no_stopping_area_polygons_(no_stopping_area_polygons),
  bus_stop_area_polygons_(bus_stop_area_polygons)
{
}

GoalSearcher GoalSearcher::create(
  const GoalPlannerParameters & parameters, const LinearRing2d & vehicle_footprint,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  const auto left_side_parking = parameters.parking_policy == ParkingPolicy::LEFT_SIDE;
  const auto pull_over_lanes = goal_planner_utils::getPullOverLanes(
    *planner_data->route_handler, left_side_parking, parameters.backward_goal_search_length,
    parameters.forward_goal_search_length);

  const auto no_parking_areas = lanelet::utils::query::noParkingAreas(pull_over_lanes);
  lanelet::BasicPolygons2d no_parking_area_polygons;
  for (const auto & no_parking_area : no_parking_areas) {
    for (const auto & area : no_parking_area->noParkingAreas()) {
      no_parking_area_polygons.push_back(lanelet::utils::to2D(area).basicPolygon());
    }
  }

  const auto no_stopping_areas = lanelet::utils::query::noStoppingAreas(pull_over_lanes);
  lanelet::BasicPolygons2d no_stopping_area_polygons;
  for (const auto & no_stopping_area : no_stopping_areas) {
    for (const auto & area : no_stopping_area->noStoppingAreas()) {
      no_stopping_area_polygons.push_back(lanelet::utils::to2D(area).basicPolygon());
    }
  }

  const auto bus_stop_areas = lanelet::utils::query::busStopAreas(pull_over_lanes);
  lanelet::BasicPolygons2d bus_stop_area_polygons;
  for (const auto & bus_stop_area : bus_stop_areas) {
    for (const auto & area : bus_stop_area->busStopAreas()) {
      bus_stop_area_polygons.push_back(lanelet::utils::to2D(area).basicPolygon());
    }
  }

  return GoalSearcher(
    parameters, vehicle_footprint, left_side_parking, pull_over_lanes, no_parking_area_polygons,
    no_stopping_area_polygons, bus_stop_area_polygons);
}

GoalCandidates GoalSearcher::search(
  const std::shared_ptr<const PlannerData> & planner_data, const bool use_bus_stop_area)
{
  GoalCandidates goal_candidates{};

  const auto reference_goal_pose_opt = goal_planner_utils::calcRefinedGoal(
    planner_data->route_handler->getOriginalGoalPose(), planner_data->route_handler,
    left_side_parking_, planner_data->parameters.vehicle_width,
    planner_data->parameters.base_link2front, planner_data->parameters.base_link2rear, parameters_);

  if (!reference_goal_pose_opt) {
    return goal_candidates;
  }
  const auto & reference_goal_pose = reference_goal_pose_opt.value();

  const auto & route_handler = planner_data->route_handler;
  const double forward_length = parameters_.forward_goal_search_length;
  const double backward_length = parameters_.backward_goal_search_length;
  const double margin_from_boundary = parameters_.margin_from_boundary;

  const double lateral_offset_interval = use_bus_stop_area
                                           ? parameters_.bus_stop_area.lateral_offset_interval
                                           : parameters_.lateral_offset_interval;
  const double max_lateral_offset = use_bus_stop_area ? 10.0 : parameters_.max_lateral_offset;
  const double vehicle_length = planner_data->parameters.vehicle_length;
  const double base_link2front = planner_data->parameters.base_link2front;

  const auto departure_check_lane = goal_planner_utils::createDepartureCheckLanelet(
    pull_over_lanes_, *route_handler, left_side_parking_);

  // Combine lanelets to get the boundary
  const auto pull_over_lanelet = lanelet::utils::combineLaneletsShape(pull_over_lanes_);
  const auto boundary =
    left_side_parking_ ? pull_over_lanelet.leftBound() : pull_over_lanelet.rightBound();

  // Create a pose offset forward by base_link2front from reference goal pose
  const Pose front_ref_pose = calc_offset_pose(reference_goal_pose, base_link2front, 0.0, 0.0);

  // used in createAreaPolygon for search area visualization.
  std::vector<Pose> min_margin_from_boundary_goal_poses{};

  // Resample boundary within search range
  const double longitudinal_interval = use_bus_stop_area
                                         ? parameters_.bus_stop_area.goal_search_interval
                                         : parameters_.goal_search_interval;

  const auto boundary_poses = resampleBoundaryWithinRange(
    boundary, front_ref_pose, backward_length, forward_length, longitudinal_interval);

  size_t goal_id = 0;
  for (const auto & front_pose : boundary_poses) {
    // Calculate rear pose by offsetting backward along the orientation
    const Pose rear_pose = calc_offset_pose(front_pose, -vehicle_length, 0.0, 0.0);

    // Calculate goal pose with minimum margin from boundary
    const auto pose_with_min_margin_opt = calculateGoalPoseWithMinMargin(
      front_pose, rear_pose, boundary, base_link2front, margin_from_boundary, left_side_parking_);

    if (!pose_with_min_margin_opt) {
      continue;
    }
    const auto & pose_with_min_margin_from_boundary = pose_with_min_margin_opt.value();

    const double longitudinal_distance_from_original_goal =
      autoware_utils::inverse_transform_point(
        pose_with_min_margin_from_boundary.position, reference_goal_pose)
        .x;

    // search goal_pose in lateral direction
    const double sign = left_side_parking_ ? -1.0 : 1.0;
    bool has_added_min_margin_goal = false;
    for (double dy = 0.0; dy <= max_lateral_offset; dy += lateral_offset_interval) {
      const Pose goal_pose =
        calc_offset_pose(pose_with_min_margin_from_boundary, 0.0, sign * dy, 0.0);

      const auto transformed_vehicle_footprint = autoware_utils::transform_vector(
        vehicle_footprint_, autoware_utils::pose2transform(goal_pose));

      if (
        use_bus_stop_area && !goal_planner_utils::isWithinAreas(
                               transformed_vehicle_footprint, bus_stop_area_polygons_)) {
        continue;
      }

      if (goal_planner_utils::isIntersectingAreas(
            transformed_vehicle_footprint, no_parking_area_polygons_)) {
        // break here to exclude goals located laterally in no_parking_areas
        // break;
      }

      if (goal_planner_utils::isIntersectingAreas(
            transformed_vehicle_footprint, no_stopping_area_polygons_)) {
        // break here to exclude goals located laterally in no_stopping_areas
        // break;
      }

      if (!boost::geometry::within(
            transformed_vehicle_footprint, departure_check_lane.polygon2d().basicPolygon())) {
        // continue;
      }

      if (!has_added_min_margin_goal) {
        min_margin_from_boundary_goal_poses.push_back(goal_pose);
        has_added_min_margin_goal = true;
      }

      GoalCandidate goal_candidate{};
      goal_candidate.goal_pose = goal_pose;
      goal_candidate.lateral_offset = dy;
      goal_candidate.id = goal_id;
      goal_id++;
      // use longitudinal_distance as distance_from_original_goal
      goal_candidate.distance_from_original_goal = longitudinal_distance_from_original_goal;
      goal_candidates.push_back(goal_candidate);
    }
  }
  createAreaPolygons(min_margin_from_boundary_goal_poses, planner_data);

  return goal_candidates;
}

void GoalSearcher::countObjectsToAvoid(
  GoalCandidates & goal_candidates, const PredictedObjects & objects,
  const std::shared_ptr<const PlannerData> & planner_data, const Pose & reference_goal_pose) const
{
  const auto & route_handler = planner_data->route_handler;
  const double forward_length = parameters_.forward_goal_search_length;
  const double backward_length = parameters_.backward_goal_search_length;

  // calculate search start/end pose in pull over lanes
  const auto search_start_end_poses = std::invoke([&]() -> std::pair<Pose, Pose> {
    const auto goal_arc_coords =
      lanelet::utils::getArcCoordinates(pull_over_lanes_, reference_goal_pose);
    const double s_start = std::max(0.0, goal_arc_coords.length - backward_length);
    const double s_end = goal_arc_coords.length + forward_length;
    const auto center_line_path = utils::resamplePathWithSpline(
      route_handler->getCenterLinePath(pull_over_lanes_, s_start, s_end),
      parameters_.goal_search_interval);
    return std::make_pair(
      center_line_path.points.front().point.pose, center_line_path.points.back().point.pose);
  });
  const auto search_start_pose = std::get<0>(search_start_end_poses);
  const auto search_end_pose = std::get<1>(search_start_end_poses);

  // generate current lane center line path to check collision with objects
  const auto current_lanes = utils::getExtendedCurrentLanes(
    planner_data, parameters_.backward_goal_search_length, parameters_.forward_goal_search_length,
    /*forward_only_in_route*/ false);
  const auto current_center_line_path = std::invoke([&]() -> PathWithLaneId {
    const double s_start =
      lanelet::utils::getArcCoordinates(current_lanes, search_start_pose).length;
    const double s_end = lanelet::utils::getArcCoordinates(current_lanes, search_end_pose).length;
    return utils::resamplePathWithSpline(
      route_handler->getCenterLinePath(current_lanes, s_start, s_end), 1.0);
  });

  // reset num_objects_to_avoid
  for (auto & goal_candidate : goal_candidates) {
    goal_candidate.num_objects_to_avoid = 0;
  }

  // count number of objects to avoid
  for (const auto & object : objects.objects) {
    for (const auto & p : current_center_line_path.points) {
      const auto transformed_vehicle_footprint = autoware_utils::transform_vector(
        vehicle_footprint_, autoware_utils::pose2transform(p.point.pose));
      const auto obj_polygon = autoware_utils::to_polygon2d(object);
      const double distance = boost::geometry::distance(obj_polygon, transformed_vehicle_footprint);
      if (distance > parameters_.object_recognition_collision_check_hard_margins.back()) {
        continue;
      }
      const Pose & object_pose = object.kinematics.initial_pose_with_covariance.pose;
      const double s_object = lanelet::utils::getArcCoordinates(current_lanes, object_pose).length;
      for (auto & goal_candidate : goal_candidates) {
        const Pose & goal_pose = goal_candidate.goal_pose;
        const double s_goal = lanelet::utils::getArcCoordinates(current_lanes, goal_pose).length;
        if (s_object < s_goal) {
          goal_candidate.num_objects_to_avoid++;
        }
      }
      break;
    }
  }
}

void GoalSearcher::update(
  GoalCandidates & goal_candidates,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
  const std::shared_ptr<const PlannerData> & planner_data, const PredictedObjects & objects) const
{
  const auto refined_goal_opt = goal_planner_utils::calcRefinedGoal(
    planner_data->route_handler->getOriginalGoalPose(), planner_data->route_handler,
    left_side_parking_, planner_data->parameters.vehicle_width,
    planner_data->parameters.base_link2front, planner_data->parameters.base_link2rear, parameters_);

  if (!refined_goal_opt) {
    return;
  }

  const auto & refined_goal = refined_goal_opt.value();
  if (parameters_.prioritize_goals_before_objects) {
    countObjectsToAvoid(goal_candidates, objects, planner_data, refined_goal);
  }

  if (parameters_.goal_priority == "minimum_weighted_distance") {
    std::sort(
      goal_candidates.begin(), goal_candidates.end(),
      SortByWeightedDistance(
        parameters_.minimum_weighted_distance_lateral_weight,
        parameters_.prioritize_goals_before_objects));
  } else if (parameters_.goal_priority == "minimum_longitudinal_distance") {
    std::sort(
      goal_candidates.begin(), goal_candidates.end(),
      SortByLongitudinalDistance(parameters_.prioritize_goals_before_objects));
  }

  // update is_safe
  for (auto & goal_candidate : goal_candidates) {
    const Pose goal_pose = goal_candidate.goal_pose;

    // check collision with footprint
    if (checkCollision(goal_pose, objects, occupancy_grid_map)) {
      goal_candidate.is_safe = false;
      continue;
    }

    // check longitudinal margin with pull over lane objects
    constexpr bool filter_inside = true;
    const auto target_objects = goal_planner_utils::filterObjectsByLateralDistance(
      goal_pose, planner_data->parameters.vehicle_width, objects,
      parameters_.object_recognition_collision_check_hard_margins.back(), filter_inside);
    if (checkCollisionWithLongitudinalDistance(
          goal_pose, target_objects, occupancy_grid_map, planner_data)) {
      goal_candidate.is_safe = false;
      continue;
    }

    goal_candidate.is_safe = true;
  }
}

// Note: this function is not just return goal_candidate.is_safe but check collision with
// current planner_data_ and margin scale factor.
// And is_safe is not updated in this function.
bool GoalSearcher::isSafeGoalWithMarginScaleFactor(
  const GoalCandidate & goal_candidate, const double margin_scale_factor,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
  const std::shared_ptr<const PlannerData> & planner_data, const PredictedObjects & objects) const
{
  const Pose goal_pose = goal_candidate.goal_pose;
  const double margin =
    parameters_.object_recognition_collision_check_hard_margins.back() * margin_scale_factor;

  if (utils::checkCollisionBetweenFootprintAndObjects(
        vehicle_footprint_, goal_pose, objects, margin)) {
    return false;
  }

  // check longitudinal margin with pull over lane objects
  constexpr bool filter_inside = true;
  const auto target_objects = goal_planner_utils::filterObjectsByLateralDistance(
    goal_pose, planner_data->parameters.vehicle_width, objects, margin, filter_inside);
  if (checkCollisionWithLongitudinalDistance(
        goal_pose, target_objects, occupancy_grid_map, planner_data)) {
    return false;
  }

  return true;
}

bool GoalSearcher::checkCollision(
  const Pose & pose, const PredictedObjects & objects,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map) const
{
  if (parameters_.use_occupancy_grid_for_goal_search) {
    const Pose pose_grid_coords = global2local(occupancy_grid_map->getMap(), pose);
    const auto idx = pose2index(
      occupancy_grid_map->getMap(), pose_grid_coords, occupancy_grid_map->getParam().theta_size);
    const bool check_out_of_range = false;
    if (occupancy_grid_map->detectCollision(idx, check_out_of_range)) {
      return true;
    }
  }

  if (utils::checkCollisionBetweenFootprintAndObjects(
        vehicle_footprint_, pose, objects,
        parameters_.object_recognition_collision_check_hard_margins.back())) {
    return true;
  }
  return false;
}

bool GoalSearcher::checkCollisionWithLongitudinalDistance(
  const Pose & ego_pose, const PredictedObjects & objects,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  if (
    parameters_.use_occupancy_grid_for_goal_search &&
    parameters_.use_occupancy_grid_for_goal_longitudinal_margin) {
    constexpr bool check_out_of_range = false;
    const double offset = std::max(
      parameters_.longitudinal_margin - parameters_.occupancy_grid_collision_check_margin, 0.0);

    // check forward collision
    const Pose ego_pose_moved_forward = calc_offset_pose(ego_pose, offset, 0, 0);
    const Pose forward_pose_grid_coords =
      global2local(occupancy_grid_map->getMap(), ego_pose_moved_forward);
    const auto forward_idx = pose2index(
      occupancy_grid_map->getMap(), forward_pose_grid_coords,
      occupancy_grid_map->getParam().theta_size);
    if (occupancy_grid_map->detectCollision(forward_idx, check_out_of_range)) {
      return true;
    }

    // check backward collision
    const Pose ego_pose_moved_backward = calc_offset_pose(ego_pose, -offset, 0, 0);
    const Pose backward_pose_grid_coords =
      global2local(occupancy_grid_map->getMap(), ego_pose_moved_backward);
    const auto backward_idx = pose2index(
      occupancy_grid_map->getMap(), backward_pose_grid_coords,
      occupancy_grid_map->getParam().theta_size);
    if (occupancy_grid_map->detectCollision(backward_idx, check_out_of_range)) {
      return true;
    }
  }

  if (
    utils::calcLongitudinalDistanceFromEgoToObjects(
      ego_pose, planner_data->parameters.base_link2front, planner_data->parameters.base_link2rear,
      objects) < parameters_.longitudinal_margin) {
    return true;
  }
  return false;
}

void GoalSearcher::createAreaPolygons(
  std::vector<Pose> original_search_poses, const std::shared_ptr<const PlannerData> & planner_data)
{
  using autoware_utils::MultiPolygon2d;
  using autoware_utils::Point2d;
  using autoware_utils::Polygon2d;

  const double vehicle_width = planner_data->parameters.vehicle_width;
  const double base_link2front = planner_data->parameters.base_link2front;
  const double base_link2rear = planner_data->parameters.base_link2rear;
  const double max_lateral_offset = parameters_.max_lateral_offset;

  const auto appendPointToPolygon =
    [](Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point) {
      Point2d point{};
      point.x() = geom_point.x;
      point.y() = geom_point.y;
      boost::geometry::append(polygon.outer(), point);
    };

  boost::geometry::clear(area_polygons_);
  for (const auto p : original_search_poses) {
    Polygon2d footprint{};

    const double left_front_offset =
      left_side_parking_ ? vehicle_width / 2 : vehicle_width / 2 + max_lateral_offset;
    const Point p_left_front = calc_offset_pose(p, base_link2front, left_front_offset, 0).position;
    appendPointToPolygon(footprint, p_left_front);

    const double right_front_offset =
      left_side_parking_ ? -vehicle_width / 2 - max_lateral_offset : -vehicle_width / 2;
    const Point p_right_front =
      calc_offset_pose(p, base_link2front, right_front_offset, 0).position;
    appendPointToPolygon(footprint, p_right_front);

    const double right_back_offset =
      left_side_parking_ ? -vehicle_width / 2 - max_lateral_offset : -vehicle_width / 2;
    const Point p_right_back = calc_offset_pose(p, -base_link2rear, right_back_offset, 0).position;
    appendPointToPolygon(footprint, p_right_back);

    const double left_back_offset =
      left_side_parking_ ? vehicle_width / 2 : vehicle_width / 2 + max_lateral_offset;
    const Point p_left_back = calc_offset_pose(p, -base_link2rear, left_back_offset, 0).position;
    appendPointToPolygon(footprint, p_left_back);

    appendPointToPolygon(footprint, p_left_front);

    MultiPolygon2d current_result{};
    boost::geometry::union_(footprint, area_polygons_, current_result);
    area_polygons_ = current_result;
  }
}

std::optional<GoalCandidate> GoalSearcher::getClosestGoalCandidateAlongLanes(
  const GoalCandidates & goal_candidates,
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  const auto current_lanes = utils::getExtendedCurrentLanes(
    planner_data, parameters_.backward_goal_search_length, parameters_.forward_goal_search_length,
    /*forward_only_in_route*/ false);

  // Define a lambda function to compute the arc length for a given goal candidate.
  auto getGoalArcLength = [&current_lanes](const auto & candidate) {
    return lanelet::utils::getArcCoordinates(current_lanes, candidate.goal_pose).length;
  };

  // Find the closest goal candidate by comparing the arc lengths of each candidate.
  const auto closest_goal_candidate = std::min_element(
    goal_candidates.begin(), goal_candidates.end(),
    [&getGoalArcLength](const auto & a, const auto & b) {
      return getGoalArcLength(a) < getGoalArcLength(b);
    });

  if (closest_goal_candidate == goal_candidates.end()) {
    return {};  // return empty GoalCandidate in case no valid candidates are found.
  }

  return *closest_goal_candidate;
}

}  // namespace autoware::behavior_path_planner
