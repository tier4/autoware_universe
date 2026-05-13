#include "assessment.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::collision_timing_assessment
{
namespace
{
bool is_target_trajectory_type(
  const ObjectTrajectoryGenerationOptions & options, const std::string & trajectory_type)
{
  if (trajectory_type.find("diffusion_based_trajectory") != std::string::npos) {
    return options.diffusion_based_trajectory;
  }
  if (trajectory_type.find("constant_curvature_path") != std::string::npos) {
    return options.constant_curvature_trajectory;
  }
  if (trajectory_type.find("map_based_predicted_path") != std::string::npos) {
    return options.predicted_path_trajectory;
  }
  return false;
}

struct DracAssessment
{
  std::optional<double> drac{0.0};
  std::vector<Finding> findings;
};

std::optional<Finding> find_collision_timing(
  const TrajectoryData & ref_trajectory, const TrajectoryData & test_trajectory,
  double positive_pet_threshold, double negative_pet_threshold, double time_resolution)
{
  const double before_pet_threshold = std::abs(negative_pet_threshold);
  const double after_pet_threshold = std::max(0.0, positive_pet_threshold);
  const double max_pet_threshold = std::max(before_pet_threshold, after_pet_threshold);

  if (!boost::geometry::intersects(
        ref_trajectory.get_or_compute_overall_envelope(),
        test_trajectory.get_or_compute_envelope(TimeRange{
          ref_trajectory.getTimes().front() - before_pet_threshold,
          ref_trajectory.getTimes().back() + after_pet_threshold}))) {
    return std::nullopt;
  }

  struct CandidateFinding
  {
    double ttc;
    double pet;
    IndexRange ref_index_range;
    TimeRange test_time_range;
  };

  const auto make_finding = [&](const CandidateFinding & candidate) -> Finding {
    const auto & object_identification = test_trajectory.getObjectIdentification();
    return Finding{
      object_identification,
      candidate.pet,
      candidate.ttc,
      ref_trajectory.getPoses(),
      test_trajectory.getPoses(),
      ref_trajectory.get_or_compute_convex(candidate.ref_index_range),
      test_trajectory.get_or_compute_convex(candidate.test_time_range)};
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

    const auto has_intersects = [&](const TimeRange & time_range) -> bool {
      if (!boost::geometry::intersects(
            ref_envelope, test_trajectory.get_or_compute_envelope(time_range))) {
        return false;
      }

      return geometry::intersects_sat(
        ref_convex, test_trajectory.get_or_compute_convex(time_range));
    };

    for (double pet_range = 0.0; pet_range < current_pet_limit; pet_range += time_resolution) {
      const TimeRange test_time_range_before{ref_start_time - pet_range, ref_end_time - pet_range};
      const bool has_intersects_before = has_intersects(test_time_range_before);

      const TimeRange test_time_range_after{ref_start_time + pet_range, ref_end_time + pet_range};
      const bool has_intersects_after = has_intersects(test_time_range_after);

      if (!has_intersects_before && !has_intersects_after) {
        continue;
      }

      const double pet = has_intersects_before ? -pet_range : pet_range;
      const TimeRange test_time_range =
        has_intersects_before ? test_time_range_before : test_time_range_after;

      candidate_finding = CandidateFinding{ref_start_time, pet, ref_index_range, test_time_range};
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

std::vector<Finding> assess_planned_speed_collision_timing(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const PetCollisionParams & pet_collision_params, double time_resolution,
  VehicleInfo & vehicle_info, const std::vector<TrajectoryData> & object_trajectories)
{
  const double ego_time_horizon_for_pet = std::abs(context.odometry->twist.twist.linear.x) * 0.5 /
                                            -pet_collision_params.ego_assumed_acceleration +
                                          pet_collision_params.ego_total_braking_delay;
  auto ego_trajectory = trajectory::generate_ego_trajectory(
    traj_points, context, ego_time_horizon_for_pet, time_resolution, vehicle_info);

  std::vector<Finding> findings{};
  findings.reserve(object_trajectories.size());

  for (const auto & object_trajectory : object_trajectories) {
    if (!is_target_trajectory_type(
          ObjectTrajectoryGenerationOptions{pet_collision_params},
          object_trajectory.getObjectIdentification().trajectory_type)) {
      continue;
    }

    auto finding = find_collision_timing(
      ego_trajectory, object_trajectory,
      pet_collision_params.warn_threshold.ego_first_passing_time_gap,
      -pet_collision_params.warn_threshold.object_first_passing_time_gap, time_resolution);
    if (finding.has_value()) {
      findings.push_back(std::move(finding.value()));
    }
  }

  return findings;
}

DracAssessment assess_drac(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const DracParams & drac_params, VehicleInfo & vehicle_info,
  const std::vector<TrajectoryData> & object_trajectories, const GlobalParams & global_params)
{
  const double ego_time_horizon = rclcpp::Duration(traj_points.back().time_from_start).seconds();

  constexpr double default_ego_deceleration_step = 1.0;
  constexpr double default_max_ego_deceleration = 6.0;
  std::vector<Finding> last_findings{};

  for (double ego_dec = 0.0; ego_dec < default_max_ego_deceleration + 1e-3;
       ego_dec += default_ego_deceleration_step) {
    const auto ego_deceleration_trajectory = [&]() {
      if (ego_dec == 0.0) {
        return trajectory::generate_ego_trajectory(
          traj_points, context, ego_time_horizon, global_params.time_resolution, vehicle_info);
      } else if (ego_dec > default_max_ego_deceleration - 1e-3) {
        return trajectory::generate_ego_trajectory(
          context.odometry->twist.twist, 0.0, -ego_dec, ego_time_horizon,
          global_params.time_resolution, traj_points, vehicle_info);
      }
      return trajectory::generate_ego_trajectory(
        context.odometry->twist.twist, drac_params.ego_total_braking_delay, -ego_dec,
        ego_time_horizon, global_params.time_resolution, traj_points, vehicle_info);
    }();

    std::vector<Finding> findings{};
    findings.reserve(object_trajectories.size());
    for (const auto & object_trajectory : object_trajectories) {
      if (!is_target_trajectory_type(
            ObjectTrajectoryGenerationOptions{drac_params},
            object_trajectory.getObjectIdentification().trajectory_type)) {
        continue;
      }

      constexpr double drac_params_collision_time_threshold = 1.0;
      auto finding_nominal_object_motion = find_collision_timing(
        ego_deceleration_trajectory, object_trajectory, drac_params_collision_time_threshold,
        -drac_params_collision_time_threshold, global_params.time_resolution);
      if (!finding_nominal_object_motion.has_value()) {
        continue;
      }

      const auto & traj_type_str = object_trajectory.getObjectIdentification().trajectory_type;
      const auto & object_id = object_trajectory.getObjectIdentification().uuid;

      const auto object_deceleration_trajectory = trajectory::generate_object_trajectory(
        context, object_id, traj_type_str, -ego_dec, global_params.time_resolution,
        ego_time_horizon + drac_params_collision_time_threshold);

      auto finding_dec_object_motion = find_collision_timing(
        ego_deceleration_trajectory, object_deceleration_trajectory,
        drac_params_collision_time_threshold, -drac_params_collision_time_threshold,
        global_params.time_resolution);
      if (!finding_dec_object_motion.has_value()) {
        continue;
      }

      findings.push_back(std::move(finding_nominal_object_motion.value()));
      findings.push_back(std::move(finding_dec_object_motion.value()));
    }
    if (findings.empty()) {
      return DracAssessment{ego_dec, std::move(last_findings)};
    }

    last_findings = std::move(findings);
  }

  return DracAssessment{std::nullopt, std::move(last_findings)};
}
}  // namespace

std::vector<TrajectoryData> generate_object_trajectories(
  const FilterContext & context, double required_time_horizon, double object_assumed_acceleration,
  double time_resolution, const ObjectTrajectoryGenerationOptions & options)
{
  std::vector<TrajectoryData> object_trajectories{};

  if (context.predicted_objects) {
    const auto trajectory_num_per_object =
      static_cast<size_t>(options.predicted_path_trajectory) +
      static_cast<size_t>(options.constant_curvature_trajectory);
    object_trajectories.reserve(
      object_trajectories.size() +
      context.predicted_objects->objects.size() * trajectory_num_per_object);
    const rclcpp::Duration objects_reference_time =
      rclcpp::Time(context.predicted_objects->header.stamp) -
      rclcpp::Time(context.odometry->header.stamp);
    for (const auto & object : context.predicted_objects->objects) {
      if (options.predicted_path_trajectory && !object.kinematics.predicted_paths.empty()) {
        object_trajectories.push_back(trajectory::generate_predicted_path_trajectory(
          object, 0.0, object_assumed_acceleration, objects_reference_time, required_time_horizon,
          context.predicted_objects->header.stamp, time_resolution));
      }

      if (options.constant_curvature_trajectory) {
        object_trajectories.push_back(trajectory::generate_constant_curvature_trajectory(
          object, 0.0, object_assumed_acceleration, objects_reference_time, required_time_horizon,
          context.predicted_objects->header.stamp, time_resolution));
      }
    }
  }

  if (options.diffusion_based_trajectory && context.neural_network_predicted_objects) {
    object_trajectories.reserve(
      object_trajectories.size() + context.neural_network_predicted_objects->objects.size());
    const rclcpp::Duration neural_network_objects_reference_time =
      rclcpp::Time(context.neural_network_predicted_objects->header.stamp) -
      rclcpp::Time(context.odometry->header.stamp);
    for (const auto & object : context.neural_network_predicted_objects->objects) {
      if (object.kinematics.predicted_paths.empty()) {
        continue;
      }
      object_trajectories.push_back(trajectory::generate_diffusion_based_trajectory(
        object, neural_network_objects_reference_time, required_time_horizon,
        context.neural_network_predicted_objects->header.stamp, time_resolution));
    }
  }
  return object_trajectories;
}

Result assess(
  const TrajectoryPoints & traj_points, const FilterContext & context,
  const PetCollisionParams & pet_collision_params, const DracParams & drac_params,
  const GlobalParams & global_params, VehicleInfo & vehicle_info)
{
  ObjectTrajectoryGenerationOptions required_trajectory_types;
  if (pet_collision_params.enable_assessment) {
    required_trajectory_types.merge_with(ObjectTrajectoryGenerationOptions{pet_collision_params});
  }
  if (drac_params.enable_assessment) {
    required_trajectory_types.merge_with(ObjectTrajectoryGenerationOptions{drac_params});
  }

  const double required_time_horizon =
    rclcpp::Duration(traj_points.back().time_from_start).seconds() +
    pet_collision_params.warn_threshold.ego_first_passing_time_gap;
  const auto nominal_speed_object_trajectories = generate_object_trajectories(
    context, required_time_horizon, -1.0, global_params.time_resolution, required_trajectory_types);

  Result result{};
  if (!pet_collision_params.enable_assessment) {
    result.planned_speed_findings = {};
  } else {
    result.planned_speed_findings = assess_planned_speed_collision_timing(
      traj_points, context, pet_collision_params, global_params.time_resolution, vehicle_info,
      nominal_speed_object_trajectories);
  }

  if (!drac_params.enable_assessment) {
    DracAssessment drac_assessment{0.0, {}};
    result.drac_findings = drac_assessment.findings;
    result.drac = drac_assessment.drac;
  } else {
    const auto drac_assessment = assess_drac(
      traj_points, context, drac_params, vehicle_info, nominal_speed_object_trajectories,
      global_params);
    result.drac_findings = drac_assessment.findings;
    result.drac = drac_assessment.drac;
  }
  return result;
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
  const TrajectoryPoints & traj_points, const FilterContext & context, double time_resolution,
  VehicleInfo & vehicle_info)
{
  const double ego_time_horizon_for_rss =
    rclcpp::Duration(traj_points.back().time_from_start).seconds();

  return trajectory::generate_ego_trajectory(
    traj_points, context, ego_time_horizon_for_rss, time_resolution, vehicle_info);
}

Assessment assess_required_deceleration(
  const TrajectoryData & ego_trajectory, const geometry_msgs::msg::Twist & ego_twist,
  const autoware_perception_msgs::msg::PredictedObject & object, const RssParams & rss_params,
  const builtin_interfaces::msg::Time & stamp)
{
  const auto ego_long_vel = ego_twist.linear.x;
  if (ego_long_vel <= 0.0) {
    return Assessment{TrajectoryIdentification{object, stamp}, 0.0};
  }

  const auto distance_to_collision = compute_distance_to_collision(ego_trajectory, object);
  if (!distance_to_collision.has_value()) {
    return Assessment{TrajectoryIdentification{object, stamp}, 0.0};
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

  return Assessment{TrajectoryIdentification{object, stamp}, required_deceleration};
}

Result assess(
  const TrajectoryPoints & traj_points, const FilterContext & context, const RssParams & rss_params,
  double time_resolution, VehicleInfo & vehicle_info)
{
  if (!context.predicted_objects || context.predicted_objects->objects.empty()) {
    return {};
  }

  const auto ego_trajectory =
    generate_rss_ego_trajectory(traj_points, context, time_resolution, vehicle_info);

  Result result{};
  result.violations.reserve(context.predicted_objects->objects.size());

  for (const auto & object : context.predicted_objects->objects) {
    const auto assessment = assess_required_deceleration(
      ego_trajectory, context.odometry->twist.twist, object, rss_params,
      context.predicted_objects->header.stamp);

    if (
      !result.worst_assessment.has_value() ||
      assessment.required_deceleration > result.worst_assessment->required_deceleration) {
      result.worst_assessment = assessment;
    }

    if (assessment.required_deceleration > -rss_params.error_threshold.ego_acceleration) {
      result.has_violation = true;
      result.violations.push_back(assessment);
    }
  }

  return result;
}
}  // namespace autoware::trajectory_validator::plugin::safety::rss_deceleration
