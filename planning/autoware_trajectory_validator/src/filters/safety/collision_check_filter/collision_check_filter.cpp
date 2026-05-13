// Copyright 2025 TIER IV, Inc.
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

#include "collision_check_filter.hpp"

#include "assessment.hpp"

#include <autoware_internal_planning_msgs/msg/control_point.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor_array.hpp>

#include <fmt/core.h>

#include <any>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
namespace
{
struct EvaluationArtifacts
{
  bool is_feasible{true};
  std::string error_msg{};
  std::vector<MetricReport> metrics{};
  autoware_internal_planning_msgs::msg::PlanningFactorArray planning_factors{};
};
}  // namespace

void CollisionCheckFilter::update_parameters(const validator::Params & node_params)
{
  std::cerr << __LINE__ << std::endl; 

  global_params_ = GlobalParams(node_params.collision_check.global_setting);

  std::cerr << __LINE__ << std::endl; 

  drac_param_map_ = create_param_map_per_object<DracParams>(node_params);
  pet_collision_param_map_ = create_param_map_per_object<PetCollisionParams>(node_params);
  rss_param_map_ = create_param_map_per_object<RssParams>(node_params);

  std::cerr << __LINE__ << std::endl; 

  pet_collision_params_ = pet_collision_param_map_.at(kCollisionCheckParamBaseKey);
  rss_params_ = rss_param_map_.at(kCollisionCheckParamBaseKey);
  drac_params_ = drac_param_map_.at(kCollisionCheckParamBaseKey);

  std::cerr << __LINE__ << std::endl; 
}

autoware_internal_planning_msgs::msg::SafetyFactorArray make_safety_factor_array(
  const builtin_interfaces::msg::Time & stamp, const collision_timing_assessment::Finding & finding,
  const std::string & collision_type, double time_resolution)
{
  using autoware_internal_planning_msgs::msg::SafetyFactor;
  using autoware_internal_planning_msgs::msg::SafetyFactorArray;

  SafetyFactor safety_factor;
  safety_factor.type = SafetyFactor::OBJECT;
  safety_factor.object_id = finding.object_identification.uuid;
  safety_factor.ttc_begin = static_cast<float>(finding.ttc);
  safety_factor.ttc_end = static_cast<float>(finding.ttc + time_resolution);
  safety_factor.is_safe = false;
  if (!finding.object_trajectory.empty()) {
    safety_factor.points.push_back(finding.object_trajectory.front().position);
  }

  SafetyFactorArray safety_factors;
  safety_factors.header.stamp = stamp;
  safety_factors.header.frame_id = "map";
  safety_factors.factors.push_back(std::move(safety_factor));
  safety_factors.is_safe = false;
  safety_factors.detail = collision_type;
  return safety_factors;
}

void CollisionCheckFilter::clear_detection_times()
{
  pet_continuous_times_.clear();
  rss_continuous_times_.clear();
  drac_continuous_times_.clear();
}

using AddDebugMarkers = std::function<void(
  const rclcpp::Time &, const std::string &, const std::string &, const PoseTrajectory &,
  const PoseTrajectory &, const Polygon2d &, const Polygon2d &)>;

using AddPlanningFactor = std::function<void(
  const builtin_interfaces::msg::Time &, const geometry_msgs::msg::Pose &,
  const collision_timing_assessment::Finding &, const std::string &,
  autoware_internal_planning_msgs::msg::PlanningFactorArray &)>;

void add_collision_planning_factor(
  const double time_resolution, const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::Pose & ego_pose, const collision_timing_assessment::Finding & finding,
  const std::string & collision_type,
  autoware_internal_planning_msgs::msg::PlanningFactorArray & planning_factors)
{
  const auto safety_factors =
    make_safety_factor_array(stamp, finding, collision_type, time_resolution);
  const auto control_point =
    autoware_internal_planning_msgs::build<autoware_internal_planning_msgs::msg::ControlPoint>()
      .pose(ego_pose)
      .velocity(0.0)
      .shift_length(0.0)
      .distance(0.0);
  auto factor =
    autoware_internal_planning_msgs::build<autoware_internal_planning_msgs::msg::PlanningFactor>()
      .module("")
      .is_driving_forward(true)
      .control_points({control_point})
      .behavior(autoware_internal_planning_msgs::msg::PlanningFactor::STOP)
      .detail(collision_type)
      .safety_factors(safety_factors);
  planning_factors.factors.push_back(std::move(factor));
}

void process_pet_findings(
  const std::string & validator_name, const std::string & validator_category,
  const PetCollisionParams & pet_collision_params,
  reporter::ContinuousDetectionTimes & pet_continuous_times, const rclcpp::Time & current_time,
  const builtin_interfaces::msg::Time & stamp, const geometry_msgs::msg::Pose & ego_pose,
  const std::vector<collision_timing_assessment::Finding> & findings,
  EvaluationArtifacts & artifacts, const AddDebugMarkers & add_debug_markers,
  const AddPlanningFactor & add_planning_factor)
{
  pet_continuous_times.update(current_time, findings, [](const auto & finding) {
    return finding.object_identification.trajectory_id_string();
  });

  std::string log_messages{};
  std::string marker_messages{};
  uint8_t log_level = MetricReport::WARN;
  for (const auto & finding : findings) {
    const auto & obj_id = finding.object_identification;
    const bool is_error =
      finding.pet <= pet_collision_params.error_threshold.ego_first_passing_time_gap &&
      finding.pet >= -pet_collision_params.error_threshold.object_first_passing_time_gap;
    const uint8_t metric_level = is_error ? MetricReport::ERROR : MetricReport::WARN;
    if (is_error) {
      artifacts.is_feasible = false;
      log_level = MetricReport::ERROR;
    }

    artifacts.metrics.push_back(
      autoware_trajectory_validator::build<MetricReport>()
        .validator_name(validator_name)
        .validator_category(validator_category)
        .metric_name(
          fmt::format("check_PET_{}_{}", obj_id.trajectory_id_string(), obj_id.classification))
        .metric_value(finding.pet)
        .level(metric_level));
    artifacts.metrics.push_back(
      autoware_trajectory_validator::build<MetricReport>()
        .validator_name(validator_name)
        .validator_category(validator_category)
        .metric_name(
          fmt::format("check_TTC_{}_{}", obj_id.trajectory_id_string(), obj_id.classification))
        .metric_value(finding.ttc)
        .level(metric_level));

    const auto finding_msg = fmt::format(
      "PET collision, classification: {}, ID: {}, PET: {}, TTC: {}, duration: {}, stamp: {}.{};",
      obj_id.classification, obj_id.trajectory_id_string(), finding.pet, finding.ttc,
      pet_continuous_times.get_time(obj_id.trajectory_id_string()), obj_id.stamp.sec,
      obj_id.stamp.nanosec);
    log_messages += finding_msg;
    reporter::append_text_marker_message(marker_messages, finding_msg);
    add_debug_markers(
      stamp, "planned_speed_collision", obj_id.trajectory_id_string(), finding.ego_trajectory,
      finding.object_trajectory, finding.ego_hull, finding.object_hull);
    if (is_error) {
      add_planning_factor(stamp, ego_pose, finding, "PET", artifacts.planning_factors);
    }
  }

  artifacts.error_msg += marker_messages;
  reporter::log_collision_messages(log_level, log_messages);
}

void process_drac_findings(
  const std::string & validator_name, const std::string & validator_category,
  const DracParams & drac_params, reporter::ContinuousDetectionTimes & drac_continuous_times,
  const rclcpp::Time & current_time, const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::Pose & ego_pose,
  const collision_timing_assessment::Result & collision_timing_result,
  EvaluationArtifacts & artifacts, const AddDebugMarkers & add_debug_markers,
  const AddPlanningFactor & add_planning_factor)
{
  drac_continuous_times.update(
    current_time, collision_timing_result.drac_findings,
    [](const auto & finding) { return finding.object_identification.trajectory_id_string(); });

  const bool is_warn =
    collision_timing_result.drac == std::nullopt ||
    collision_timing_result.drac.value() >= -drac_params.warn_threshold.ego_acceleration;
  const bool is_error =
    collision_timing_result.drac == std::nullopt ||
    collision_timing_result.drac.value() >= -drac_params.error_threshold.ego_acceleration;
  if (!is_warn) {
    return;
  }

  std::string log_messages{};
  std::string marker_messages{};
  const uint8_t metric_level = is_error ? MetricReport::ERROR : MetricReport::WARN;
  for (const auto & finding : collision_timing_result.drac_findings) {
    const auto & obj_id = finding.object_identification;
    if (is_error) {
      artifacts.is_feasible = false;
    }

    artifacts.metrics.push_back(autoware_trajectory_validator::build<MetricReport>()
                                  .validator_name(validator_name)
                                  .validator_category(validator_category)
                                  .metric_name(fmt::format(
                                    "check_DRAC_{}_{}_{}", obj_id.trajectory_id_string(),
                                    obj_id.classification, obj_id.trajectory_type))
                                  .metric_value(collision_timing_result.drac.value_or(0.0))
                                  .level(metric_level));

    const auto finding_msg = fmt::format(
      "DRAC collision, ID: {}, PET: {}, TTC: {}, DRAC: {}, stamp: {}.{};",
      obj_id.trajectory_id_string(), finding.pet, finding.ttc,
      collision_timing_result.drac.has_value()
        ? std::to_string(collision_timing_result.drac.value())
        : "Cant be avoided",
      obj_id.stamp.sec, obj_id.stamp.nanosec);
    log_messages += finding_msg;
    reporter::append_text_marker_message(marker_messages, finding_msg);
    add_debug_markers(
      stamp, "drac_collision", obj_id.trajectory_id_string(), finding.ego_trajectory,
      finding.object_trajectory, finding.ego_hull, finding.object_hull);
    if (is_error) {
      add_planning_factor(stamp, ego_pose, finding, "DRAC", artifacts.planning_factors);
    }
  }

  artifacts.error_msg += marker_messages;
  reporter::log_collision_messages(metric_level, log_messages);
}

void process_rss_violations(
  const std::string & validator_name, const std::string & validator_category,
  const GlobalParams & global_params, const RssParams & rss_params,
  const TrajectoryPoints & traj_points, const FilterContext & context, VehicleInfo & vehicle_info,
  reporter::ContinuousDetectionTimes & rss_continuous_times, const rclcpp::Time & current_time,
  EvaluationArtifacts & artifacts)
{
  if (!rss_params.enable_assessment) {
    return;
  }

  const auto rss_result = rss_deceleration::assess(
    traj_points, context, rss_params, global_params.time_resolution, vehicle_info);
  rss_continuous_times.update(current_time, rss_result.violations, [](const auto & violation) {
    return violation.object.object_id_string();
  });

  std::string log_messages{};
  std::string marker_messages{};
  for (const auto & violation : rss_result.violations) {
    const auto object_id = violation.object.object_id_string();
    artifacts.is_feasible = false;
    artifacts.metrics.push_back(
      autoware_trajectory_validator::build<MetricReport>()
        .validator_name(validator_name)
        .validator_category(validator_category)
        .metric_name(fmt::format("check_RSS_{}_{}", violation.object.classification, object_id))
        .metric_value(violation.required_deceleration)
        .level(MetricReport::ERROR));

    const auto finding_msg = fmt::format(
      "RSS collision, classification: {}, ID: {}, duration: {}, required deceleration: {}, "
      "stamp: {}.{};",
      violation.object.classification, object_id, rss_continuous_times.get_time(object_id),
      violation.required_deceleration, violation.object.stamp.sec, violation.object.stamp.nanosec);
    log_messages += finding_msg;
    reporter::append_text_marker_message(marker_messages, finding_msg);
  }

  artifacts.error_msg += marker_messages;
  reporter::log_collision_messages(MetricReport::ERROR, log_messages);
}

CollisionCheckFilter::result_t CollisionCheckFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  if (
    (!context.predicted_objects || context.predicted_objects->objects.empty()) &&
    (!context.neural_network_predicted_objects ||
     context.neural_network_predicted_objects->objects.empty())) {
    clear_detection_times();
    return {};  // No objects to check collision with
  }

  if (traj_points.empty()) {
    clear_detection_times();
    return {};  // No trajectory to check
  }

  EvaluationArtifacts artifacts{};
  const rclcpp::Time current_time = context.odometry->header.stamp;
  const auto collision_timing_result = collision_timing_assessment::assess(
    traj_points, context, pet_collision_params_, drac_params_, global_params_, *vehicle_info_ptr_);
  const auto add_debug_markers_cb =
    [this](
      const rclcpp::Time & stamp, const std::string & ns, const std::string & trajectory_id,
      const PoseTrajectory & ego_trajectory, const PoseTrajectory & object_trajectory,
      const Polygon2d & ego_hull, const Polygon2d & object_hull) {
      reporter::add_debug_markers(
        debug_markers_, stamp, ns, trajectory_id, ego_trajectory, object_trajectory, ego_hull,
        object_hull);
    };
  const auto add_planning_factor_cb =
    [this](
      const builtin_interfaces::msg::Time & stamp, const geometry_msgs::msg::Pose & ego_pose,
      const collision_timing_assessment::Finding & finding, const std::string & collision_type,
      autoware_internal_planning_msgs::msg::PlanningFactorArray & planning_factors) {
      add_collision_planning_factor(
        global_params_.time_resolution, stamp, ego_pose, finding, collision_type, planning_factors);
    };
  process_pet_findings(
    get_name(), category(), pet_collision_params_, pet_continuous_times_, current_time,
    context.odometry->header.stamp, context.odometry->pose.pose,
    collision_timing_result.planned_speed_findings, artifacts, add_debug_markers_cb,
    add_planning_factor_cb);
  process_drac_findings(
    get_name(), category(), drac_params_, drac_continuous_times_, current_time,
    context.odometry->header.stamp, context.odometry->pose.pose, collision_timing_result, artifacts,
    add_debug_markers_cb, add_planning_factor_cb);
  process_rss_violations(
    get_name(), category(), global_params_, rss_params_, traj_points, context, *vehicle_info_ptr_,
    rss_continuous_times_, current_time, artifacts);
  if (!artifacts.error_msg.empty()) {
    reporter::add_error_text_marker(
      debug_markers_, context.odometry->header.stamp, context.odometry->pose.pose,
      artifacts.error_msg);
  }

  return ValidationResult{
    artifacts.is_feasible, std::move(artifacts.metrics), std::move(artifacts.planning_factors)};
}

}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::CollisionCheckFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
