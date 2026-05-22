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

#include "assessment.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::collision_timing_assessment
{
bool is_target_trajectory_type(
  const AssessmentTrajectories & options, const std::string & trajectory_type)
{
  if (trajectory_type.find("diffusion_based_trajectory") != std::string::npos) {
    return options.diffusion_based;
  }
  if (trajectory_type.find("constant_curvature_path") != std::string::npos) {
    return options.constant_curvature;
  }
  if (trajectory_type.find("map_based_predicted_path") != std::string::npos) {
    return options.map_based;
  }
  return false;
}

RiskLevel to_pet_risk_level(double pet, const PetThreshold & error_th, const PetThreshold & warn_th)
{
  const bool is_error =
    pet <= error_th.ego_first_passing_time_gap && pet >= -error_th.object_first_passing_time_gap;
  if (is_error) {
    return RiskLevel::ERROR;
  }

  const bool is_warn =
    pet <= warn_th.ego_first_passing_time_gap && pet >= -warn_th.object_first_passing_time_gap;
  return is_warn ? RiskLevel::WARN : RiskLevel::SAFE;
}

RiskLevel to_drac_risk_level(const std::optional<double> & acc, const DracParams & drac_params)
{
  if (!acc.has_value() || acc.value() < drac_params.error_threshold.ego_acceleration) {
    return RiskLevel::ERROR;
  }
  if (acc.value() < drac_params.warn_threshold.ego_acceleration) {
    return RiskLevel::WARN;
  }
  return RiskLevel::SAFE;
}

bool has_enabled_collision_edge(
  const EgoFootprintEdgeIntersections & edges, const GlobalParams & global_params)
{
  return (global_params.collision_detection_target_edges.front && edges.front) ||
         (global_params.collision_detection_target_edges.side && (edges.left || edges.right)) ||
         (global_params.collision_detection_target_edges.rear && edges.rear);
}

constexpr double kCollisionTimeRefinementPrecision = 0.01;

geometry_msgs::msg::Pose interpolate_pose_at_time(const TrajectoryData & trajectory, const double t)
{
  const auto & times = trajectory.getTimes();
  const auto & poses = trajectory.getPoses();
  if (poses.size() == 1 || t <= times.front()) {
    return poses.front();
  }
  if (t >= times.back()) {
    return poses.back();
  }

  const size_t upper_index = trajectory.get_same_or_later_time_index(t);
  const size_t lower_index = upper_index == 0 ? 0 : upper_index - 1;
  const double lower_time = times.at(lower_index);
  const double upper_time = times.at(upper_index);
  const double duration = upper_time - lower_time;
  const double ratio = duration > TIME_INDEX_EPSILON ? (t - lower_time) / duration : 0.0;
  return autoware_utils::calc_interpolated_pose(
    poses.at(lower_index), poses.at(upper_index), ratio, false);
}

Polygon2d interpolate_footprint_at_time(const TrajectoryData & trajectory, const double t)
{
  const auto & poses = trajectory.getPoses();
  const auto & footprints = trajectory.getFootprints();
  if (footprints.size() == 1 || t <= trajectory.getTimes().front()) {
    return footprints.front();
  }
  if (t >= trajectory.getTimes().back()) {
    return footprints.back();
  }

  const size_t upper_index = trajectory.get_same_or_later_time_index(t);
  const size_t lower_index = upper_index == 0 ? 0 : upper_index - 1;
  const auto interpolated_pose = interpolate_pose_at_time(trajectory, t);
  const auto & base_pose = poses.at(lower_index);
  const auto & base_footprint = footprints.at(lower_index);

  Polygon2d interpolated;
  interpolated.outer().reserve(base_footprint.outer().size());
  for (const auto & point : base_footprint.outer()) {
    geometry_msgs::msg::Point global_point;
    global_point.x = point.x();
    global_point.y = point.y();
    const auto local_point = autoware_utils::inverse_transform_point(global_point, base_pose);
    const auto transformed_point = autoware_utils::transform_point(local_point, interpolated_pose);
    interpolated.outer().emplace_back(transformed_point.x, transformed_point.y);
  }
  return interpolated;
}

struct PreciseCollisionSample
{
  double ref_time;
  Polygon2d ego_polygon;
  Polygon2d object_polygon;
  EgoFootprintEdgeIntersections edges;
};

std::optional<PreciseCollisionSample> evaluate_collision_sample(
  const TrajectoryData & ref_trajectory, const TrajectoryData & test_trajectory,
  const double ref_time, const double test_time, const GlobalParams & global_params)
{
  const auto ego_footprint = interpolate_footprint_at_time(ref_trajectory, ref_time);
  const auto object_footprint = interpolate_footprint_at_time(test_trajectory, test_time);
  if (!boost::geometry::intersects(ego_footprint, object_footprint)) {
    return std::nullopt;
  }

  const auto edge_intersections =
    geometry::intersecting_ego_footprint_edges(ego_footprint, object_footprint);
  if (!has_enabled_collision_edge(edge_intersections, global_params)) {
    return std::nullopt;
  }

  return PreciseCollisionSample{ref_time, ego_footprint, object_footprint, edge_intersections};
}

std::optional<PreciseCollisionSample> find_first_collision_in_interval(
  const TrajectoryData & ref_trajectory, const TrajectoryData & test_trajectory,
  const TimeRange & time_range, const GlobalParams & global_params)
{
  const auto ref_start_footprint = interpolate_footprint_at_time(ref_trajectory, time_range.first);
  const auto ref_end_footprint = interpolate_footprint_at_time(ref_trajectory, time_range.second);
  const auto test_start_footprint =
    interpolate_footprint_at_time(test_trajectory, time_range.first);
  const auto test_end_footprint = interpolate_footprint_at_time(test_trajectory, time_range.second);

  const auto ref_swept_hull = compute_convex_hull(
    FootprintTrajectory{ref_start_footprint, ref_end_footprint}, IndexRange{0, 1});
  const auto test_swept_hull = compute_convex_hull(
    FootprintTrajectory{test_start_footprint, test_end_footprint}, IndexRange{0, 1});
  if (!geometry::intersects_sat(ref_swept_hull, test_swept_hull)) {
    return std::nullopt;
  }

  if (time_range.second - time_range.first <= kCollisionTimeRefinementPrecision) {
    constexpr std::array<double, 5> sample_ratios{{0.0, 0.25, 0.5, 0.75, 1.0}};
    for (const double ratio : sample_ratios) {
      const double ref_time = time_range.first + (time_range.second - time_range.first) * ratio;
      const double test_time = time_range.first + (time_range.second - time_range.first) * ratio;
      const auto collision = evaluate_collision_sample(
        ref_trajectory, test_trajectory, ref_time, test_time, global_params);
      if (collision.has_value()) {
        return collision;
      }
    }
    return std::nullopt;
  }

  const double mid_time = 0.5 * (time_range.first + time_range.second);

  if (const auto first_half_collision = find_first_collision_in_interval(
        ref_trajectory, test_trajectory, TimeRange{time_range.first, mid_time}, global_params);
      first_half_collision.has_value()) {
    return first_half_collision;
  }

  return find_first_collision_in_interval(
    ref_trajectory, test_trajectory, TimeRange{mid_time, time_range.second}, global_params);
}

std::optional<CollisionDetail> find_collision_timing(
  const TrajectoryData & ref_trajectory, const TrajectoryData & test_trajectory,
  PetThreshold pet_find_range, double time_resolution, const GlobalParams & global_params)
{
  const double max_pet_threshold = std::max(
    pet_find_range.ego_first_passing_time_gap, pet_find_range.object_first_passing_time_gap);

  if (!boost::geometry::intersects(
        ref_trajectory.get_or_compute_overall_envelope(),
        test_trajectory.get_or_compute_envelope(TimeRange{
          ref_trajectory.getTimes().front() - pet_find_range.object_first_passing_time_gap,
          ref_trajectory.getTimes().back() + pet_find_range.ego_first_passing_time_gap}))) {
    return std::nullopt;
  }

  struct CandidateFinding
  {
    double ttc;
    double pet;
    IndexRange ref_index_range;
    TimeRange test_time_range;
    Polygon2d ego_polygon_at_first_collision;
    Polygon2d object_polygon_at_first_collision;
    EgoFootprintEdgeIntersections ego_collision_edges_at_first_collision;
  };

  const auto make_finding = [&](const CandidateFinding & candidate) -> CollisionDetail {
    return CollisionDetail{
      test_trajectory.getObjectIdentification(),
      candidate.pet,
      candidate.ttc,
      ref_trajectory.getPoses(),
      test_trajectory.getPoses(),
      ref_trajectory.get_or_compute_convex(candidate.ref_index_range),
      test_trajectory.get_or_compute_convex(candidate.test_time_range),
      candidate.ego_polygon_at_first_collision,
      candidate.object_polygon_at_first_collision,
      candidate.ego_collision_edges_at_first_collision};
  };

  std::optional<CandidateFinding> candidate_finding{};
  for (size_t i = 0; i < ref_trajectory.size(); ++i) {
    size_t prev_i = (i == 0) ? 0 : i - 1;
    const double ref_start_time = ref_trajectory.getTimes().at(prev_i);
    const double ref_end_time = ref_trajectory.getTimes().at(i);

    const IndexRange ref_index_range{prev_i, i};
    const Box2d & ref_envelope = ref_trajectory.get_or_compute_envelope(ref_index_range);
    const Polygon2d & ref_convex = ref_trajectory.get_or_compute_convex(ref_index_range);

    const double current_pet_limit =
      candidate_finding.has_value() ? std::abs(candidate_finding->pet) : max_pet_threshold;

    if (!boost::geometry::intersects(
          ref_envelope, test_trajectory.get_or_compute_envelope(TimeRange{
                          ref_start_time - current_pet_limit,
                          ref_trajectory.getTimes().back() + current_pet_limit}))) {
      continue;
    }

    struct RefinedCollision
    {
      double ref_time;
      Polygon2d ego_polygon;
      Polygon2d object_polygon;
      EgoFootprintEdgeIntersections edge_intersections;
    };

    const auto has_intersects =
      [&](const TimeRange & time_range) -> std::optional<RefinedCollision> {
      if (!boost::geometry::intersects(
            ref_envelope, test_trajectory.get_or_compute_envelope(time_range))) {
        return std::nullopt;
      }

      const auto & test_convex = test_trajectory.get_or_compute_convex(time_range);
      if (!geometry::intersects_sat(ref_convex, test_convex)) {
        return std::nullopt;
      }

      const auto precise_collision = find_first_collision_in_interval(
        ref_trajectory, test_trajectory, time_range, global_params);
      if (!precise_collision.has_value()) {
        return std::nullopt;
      }

      return RefinedCollision{
        precise_collision->ref_time, precise_collision->ego_polygon,
        precise_collision->object_polygon, precise_collision->edges};
    };

    for (double pet_range = 0.0; pet_range < current_pet_limit; pet_range += time_resolution) {
      const TimeRange test_time_range_before{ref_start_time - pet_range, ref_end_time - pet_range};
      const auto edge_intersections_before = has_intersects(test_time_range_before);

      const TimeRange test_time_range_after{ref_start_time + pet_range, ref_end_time + pet_range};
      const auto edge_intersections_after = has_intersects(test_time_range_after);

      if (!edge_intersections_before.has_value() && !edge_intersections_after.has_value()) {
        continue;
      }

      const bool has_intersects_before = edge_intersections_before.has_value();
      const double pet = has_intersects_before ? -pet_range : pet_range;
      const TimeRange test_time_range =
        has_intersects_before ? test_time_range_before : test_time_range_after;
      const auto collision = has_intersects_before ? edge_intersections_before.value()
                                                   : edge_intersections_after.value();

      candidate_finding = CandidateFinding{
        collision.ref_time,
        pet,
        ref_index_range,
        test_time_range,
        collision.ego_polygon,
        collision.object_polygon,
        collision.edge_intersections};
      break;
    }
    if (candidate_finding.has_value() && candidate_finding->pet == 0.0) {
      return make_finding(candidate_finding.value());
    }
  }

  if (!candidate_finding.has_value()) {
    return std::nullopt;
  }

  return make_finding(candidate_finding.value());
}

PetArtifact assess_planned_speed_collision_timing(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const PetParamMap & pet_param_map, const GlobalParams & global_params,
  const VehicleInfo & vehicle_info, const std::vector<TrajectoryData> & object_trajectories)
{
  // todo (takagi): ego_trajectory options can not set for each object type because cache structure
  // is not implemented yet.
  const auto & ego_pet_params = pet_param_map.at(kCollisionCheckParamBaseKey);
  const double ego_time_horizon_for_pet = std::abs(context.odometry->twist.twist.linear.x) * 0.5 /
                                            -ego_pet_params.ego_assumed_acceleration +
                                          ego_pet_params.ego_total_braking_delay;
  auto ego_trajectory = trajectory::generate_ego_trajectory(
    traj_points, context, ego_time_horizon_for_pet, vehicle_info, global_params);

  std::vector<CollisionEvaluation> collision_evaluations{};
  for (const auto & object_trajectory : object_trajectories) {
    const auto & pet_params =
      pet_param_map.at(object_trajectory.getObjectIdentification().classification);
    if (!is_target_trajectory_type(
          pet_params.assessment_trajectories,
          object_trajectory.getObjectIdentification().trajectory_type)) {
      continue;
    }

    auto collision = find_collision_timing(
      ego_trajectory, object_trajectory, pet_params.warn_threshold, global_params.time_resolution,
      global_params);

    if (collision.has_value()) {
      const auto risk_level =
        to_pet_risk_level(collision->pet, pet_params.error_threshold, pet_params.warn_threshold);
      if (risk_level == RiskLevel::SAFE) {
        continue;
      }
      collision_evaluations.push_back(
        CollisionEvaluation{risk_level, std::move(collision.value())});
    }
  }

  return {calc_worst_risk(collision_evaluations), std::move(collision_evaluations)};
}

DracArtifact assess_drac(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const DracParamMap & drac_param_map, const VehicleInfo & vehicle_info,
  const std::vector<TrajectoryData> & object_trajectories, const GlobalParams & global_params)
{
  const double ego_time_horizon = rclcpp::Duration(traj_points.back().time_from_start).seconds();

  // todo (takagi): ego_trajectory options can not set for each object type because cache structure
  // is not implemented yet.
  const auto & ego_drac_params = drac_param_map.at(kCollisionCheckParamBaseKey);
  constexpr double default_ego_deceleration_step = 1.0;
  constexpr double default_max_ego_deceleration = 6.0;
  constexpr PetThreshold error_pet_th{0.6, 0.3};

  std::vector<CollisionEvaluation> last_collision_evaluations{};

  for (double ego_dec = 0.0; ego_dec < default_max_ego_deceleration + 1e-3;
       ego_dec += default_ego_deceleration_step) {
    const auto ego_deceleration_trajectory = [&]() {
      if (ego_dec == 0.0) {
        return trajectory::generate_ego_trajectory(
          traj_points, context, ego_time_horizon, vehicle_info, global_params);
      } else if (ego_dec > default_max_ego_deceleration - 1e-3) {
        return trajectory::generate_ego_trajectory(
          context.odometry->twist.twist, 0.0, -ego_dec, ego_time_horizon, traj_points, vehicle_info,
          global_params);
      }
      return trajectory::generate_ego_trajectory(
        context.odometry->twist.twist, ego_drac_params.ego_total_braking_delay, -ego_dec,
        ego_time_horizon, traj_points, vehicle_info, global_params);
    }();

    std::vector<CollisionEvaluation> collision_evaluations{};
    for (const auto & object_trajectory : object_trajectories) {
      const auto & drac_params =
        drac_param_map.at(object_trajectory.getObjectIdentification().classification);
      if (!is_target_trajectory_type(
            drac_params.assessment_trajectories,
            object_trajectory.getObjectIdentification().trajectory_type)) {
        continue;
      }

      constexpr double drac_params_collision_time_threshold = 1.0;
      auto finding_nominal_object_motion = find_collision_timing(
        ego_deceleration_trajectory, object_trajectory, error_pet_th, global_params.time_resolution,
        global_params);

      const RiskLevel nominal_motion_risk_level =
        finding_nominal_object_motion.has_value()
          ? to_pet_risk_level(finding_nominal_object_motion->pet, error_pet_th, error_pet_th)
          : RiskLevel::SAFE;
      if (nominal_motion_risk_level != RiskLevel::ERROR) {
        continue;
      }

      const auto & traj_type_str = object_trajectory.getObjectIdentification().trajectory_type;
      const auto & object_id = object_trajectory.getObjectIdentification().uuid;

      const auto object_deceleration_trajectory = trajectory::generate_object_trajectory(
        context, object_id, traj_type_str, -ego_dec, global_params.time_resolution,
        ego_time_horizon + drac_params_collision_time_threshold);

      auto finding_dec_object_motion = find_collision_timing(
        ego_deceleration_trajectory, object_deceleration_trajectory, error_pet_th,
        global_params.time_resolution, global_params);

      const RiskLevel dec_motion_risk_level =
        finding_dec_object_motion.has_value()
          ? to_pet_risk_level(finding_dec_object_motion->pet, error_pet_th, error_pet_th)
          : RiskLevel::SAFE;

      if (dec_motion_risk_level != RiskLevel::ERROR) {
        continue;
      }

      collision_evaluations.push_back(CollisionEvaluation{
        nominal_motion_risk_level, std::move(finding_nominal_object_motion.value())});
      collision_evaluations.push_back(
        CollisionEvaluation{dec_motion_risk_level, std::move(finding_dec_object_motion.value())});
    }
    if (collision_evaluations.empty()) {
      return DracArtifact{
        to_drac_risk_level(-ego_dec, ego_drac_params), -ego_dec,
        std::move(last_collision_evaluations)};
    }

    last_collision_evaluations = std::move(collision_evaluations);
  }

  return DracArtifact{
    to_drac_risk_level(std::nullopt, ego_drac_params), std::nullopt,
    std::move(last_collision_evaluations)};
}

std::vector<TrajectoryData> generate_object_trajectories(
  const FilterContext & context, double required_time_horizon, double object_assumed_acceleration,
  double time_resolution, const DracParamMap & drac_param_map, const PetParamMap & pet_param_map)
{
  std::vector<TrajectoryData> object_trajectories{};

  if (context.predicted_objects) {
    // const auto trajectory_num_per_object =
    //   static_cast<size_t>(options.predicted_path_trajectory) +
    //   static_cast<size_t>(options.constant_curvature_trajectory);
    // object_trajectories.reserve(
    //   object_trajectories.size() +
    //   context.predicted_objects->objects.size() * trajectory_num_per_object);
    const rclcpp::Duration objects_reference_time =
      rclcpp::Time(context.predicted_objects->header.stamp) -
      rclcpp::Time(context.odometry->header.stamp);
    for (const auto & object : context.predicted_objects->objects) {
      const auto & drac_param = drac_param_map.at(to_type_string(object.classification));
      const auto & pet_param = pet_param_map.at(to_type_string(object.classification));
      const bool is_require_map_based =
        drac_param.assessment_trajectories.map_based || pet_param.assessment_trajectories.map_based;
      if (is_require_map_based && !object.kinematics.predicted_paths.empty()) {
        object_trajectories.push_back(trajectory::generate_predicted_path_trajectory(
          object, 0.0, object_assumed_acceleration, objects_reference_time, required_time_horizon,
          context.predicted_objects->header.stamp, time_resolution));
      }

      if (
        drac_param.assessment_trajectories.constant_curvature ||
        pet_param.assessment_trajectories.constant_curvature) {
        object_trajectories.push_back(trajectory::generate_constant_curvature_trajectory(
          object, 0.0, object_assumed_acceleration, objects_reference_time, required_time_horizon,
          context.predicted_objects->header.stamp, time_resolution));
      }
    }
  }

  if (context.neural_network_predicted_objects) {
    // object_trajectories.reserve(
    //   object_trajectories.size() + context.neural_network_predicted_objects->objects.size());
    const rclcpp::Duration neural_network_objects_reference_time =
      rclcpp::Time(context.neural_network_predicted_objects->header.stamp) -
      rclcpp::Time(context.odometry->header.stamp);
    for (const auto & object : context.neural_network_predicted_objects->objects) {
      if (object.kinematics.predicted_paths.empty()) {
        continue;
      }
      const auto & drac_param = drac_param_map.at(to_type_string(object.classification));
      const auto & pet_param = pet_param_map.at(to_type_string(object.classification));
      const bool is_require_diffusion_based = drac_param.assessment_trajectories.diffusion_based ||
                                              pet_param.assessment_trajectories.diffusion_based;
      if (is_require_diffusion_based) {
        object_trajectories.push_back(trajectory::generate_diffusion_based_trajectory(
          object, neural_network_objects_reference_time, required_time_horizon,
          context.neural_network_predicted_objects->header.stamp, time_resolution));
      }
    }
  }
  return object_trajectories;
}

std::pair<PetArtifact, DracArtifact> assess(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const PetParamMap & pet_param_map, const DracParamMap & drac_param_map,
  const GlobalParams & global_params, const VehicleInfo & vehicle_info)
{
  const double required_time_horizon =
    rclcpp::Duration(traj_points.back().time_from_start).seconds();
  const auto nominal_speed_object_trajectories = generate_object_trajectories(
    context, required_time_horizon, -1.0, global_params.time_resolution, drac_param_map,
    pet_param_map);

  PetArtifact pet_artifact{};
  pet_artifact = assess_planned_speed_collision_timing(
    traj_points, context, pet_param_map, global_params, vehicle_info,
    nominal_speed_object_trajectories);

  DracArtifact drac_artifact{};
  drac_artifact = assess_drac(
    traj_points, context, drac_param_map, vehicle_info, nominal_speed_object_trajectories,
    global_params);
  return {pet_artifact, drac_artifact};
}
}  // namespace autoware::trajectory_validator::plugin::safety::collision_timing_assessment

namespace autoware::trajectory_validator::plugin::safety::rss_deceleration
{
std::optional<double> compute_distance_to_collision(
  const TrajectoryData & ego_trajectory,
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  const auto object_footprint =
    geometry::to_polygon2d(object.kinematics.initial_pose_with_covariance.pose, object.shape);
  const auto object_envelope = boost::geometry::return_envelope<Box2d>(object_footprint);

  if (!boost::geometry::intersects(
        ego_trajectory.get_or_compute_overall_envelope(), object_envelope)) {
    return std::nullopt;
  }

  for (size_t i = 0; i < ego_trajectory.size(); ++i) {
    const auto prev_i = (i == 0) ? 0 : (i - 1);
    const auto & ego_footprint = ego_trajectory.get_or_compute_convex(IndexRange{prev_i, i});
    if (geometry::intersects_sat(ego_footprint, object_footprint)) {
      return ego_trajectory.getDistances().at(i);
    }
  }

  return std::nullopt;
}

TrajectoryData generate_rss_ego_trajectory(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const GlobalParams & global_params, const VehicleInfo & vehicle_info)
{
  const double ego_time_horizon_for_rss =
    rclcpp::Duration(traj_points.back().time_from_start).seconds();

  return trajectory::generate_ego_trajectory(
    traj_points, context, ego_time_horizon_for_rss, vehicle_info, global_params);
}

RssDetail assess_required_deceleration(
  const TrajectoryData & ego_trajectory, const geometry_msgs::msg::Twist & ego_twist,
  const autoware_perception_msgs::msg::PredictedObject & object, const RssParams & rss_params,
  const builtin_interfaces::msg::Time & stamp)
{
  const auto ego_long_vel = ego_twist.linear.x;
  if (ego_long_vel <= 0.0) {
    return RssDetail{TrajectoryIdentification{object, stamp}, 0.0};
  }

  const auto distance_to_collision = compute_distance_to_collision(ego_trajectory, object);
  if (!distance_to_collision.has_value()) {
    return RssDetail{TrajectoryIdentification{object, stamp}, 0.0};
  }

  const double obj_long_vel =
    std::clamp(compute_longitudinal_velocity(ego_trajectory.getPoses(), object), 0.0, 30.0);
  const double safe_distance =
    distance_to_collision.value() - rss_params.stop_distance_margin +
    obj_long_vel * obj_long_vel * 0.5 / -rss_params.object_assumed_acceleration -
    ego_long_vel * rss_params.ego_total_braking_delay;

  const double required_deceleration = safe_distance <= 0.0
                                         ? std::numeric_limits<double>::infinity()
                                         : ego_long_vel * ego_long_vel * 0.5 / safe_distance;

  return RssDetail{TrajectoryIdentification{object, stamp}, required_deceleration};
}

RssArtifact assess(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const RssParamMap & rss_param_map, const GlobalParams & global_params,
  const VehicleInfo & vehicle_info)
{
  if (!context.predicted_objects || context.predicted_objects->objects.empty()) {
    return {};
  }

  const auto ego_trajectory =
    generate_rss_ego_trajectory(traj_points, context, global_params, vehicle_info);

  std::vector<RssEvaluation> rss_evaluations{};
  rss_evaluations.reserve(context.predicted_objects->objects.size());

  for (const auto & object : context.predicted_objects->objects) {
    const auto & rss_params = rss_param_map.at(to_type_string(object.classification));
    if (!rss_params.enable_assessment) {
      continue;
    }
    const auto rss_detail = assess_required_deceleration(
      ego_trajectory, context.odometry->twist.twist, object, rss_params,
      context.predicted_objects->header.stamp);
    const auto risk_level =
      rss_detail.rss_acceleration < rss_params.error_threshold.ego_acceleration ? RiskLevel::ERROR
                                                                                : RiskLevel::SAFE;
    rss_evaluations.push_back(RssEvaluation{risk_level, rss_detail});
  }

  return RssArtifact{calc_worst_risk(rss_evaluations), std::move(rss_evaluations)};
}
}  // namespace autoware::trajectory_validator::plugin::safety::rss_deceleration
