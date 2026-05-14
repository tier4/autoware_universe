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
  global_params_ = GlobalParams(node_params.collision_check.global_setting);

  drac_param_map_ = create_param_map_per_object<DracParams>(node_params);
  pet_param_map_ = create_param_map_per_object<PetParams>(node_params);
  rss_param_map_ = create_param_map_per_object<RssParams>(node_params);
}

autoware_internal_planning_msgs::msg::SafetyFactorArray make_safety_factor_array(
  const builtin_interfaces::msg::Time & stamp, const CollisionDetail & collision_detail,
  const std::string & collision_type, double time_resolution)
{
  using autoware_internal_planning_msgs::msg::SafetyFactor;
  using autoware_internal_planning_msgs::msg::SafetyFactorArray;

  SafetyFactor safety_factor;
  safety_factor.type = SafetyFactor::OBJECT;
  safety_factor.object_id = collision_detail.object_identification.uuid;
  safety_factor.ttc_begin = static_cast<float>(collision_detail.ttc);
  safety_factor.ttc_end = static_cast<float>(collision_detail.ttc + time_resolution);
  safety_factor.is_safe = false;
  if (!collision_detail.object_trajectory.empty()) {
    safety_factor.points.push_back(collision_detail.object_trajectory.front().position);
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

void add_collision_planning_factor(
  const double time_resolution, const builtin_interfaces::msg::Time & stamp,
  const geometry_msgs::msg::Pose & ego_pose, const CollisionDetail & collision_detail,
  const std::string & collision_type,
  autoware_internal_planning_msgs::msg::PlanningFactorArray & planning_factors)
{
  const auto safety_factors =
    make_safety_factor_array(stamp, collision_detail, collision_type, time_resolution);
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

void process_pet_artifacts(
  const std::string & validator_name, const std::string & validator_category,
  reporter::ContinuousDetectionTimes & pet_continuous_times, const rclcpp::Time & current_time,
  const builtin_interfaces::msg::Time & stamp, const geometry_msgs::msg::Pose & ego_pose,
  const PetArtifact & pet_artifact, EvaluationArtifacts & artifacts,
  visualization_msgs::msg::MarkerArray & debug_markers, double time_resolution)
{
  pet_continuous_times.update(
    current_time, pet_artifact.object_evaluations, [](const auto & evaluation) {
      return evaluation.detail.object_identification.trajectory_id_string();
    });

  if (pet_artifact.risk == RiskLevel::SAFE || pet_artifact.object_evaluations.empty()) {
    return;
  }

  std::string log_messages{};
  std::string marker_messages{};
  uint8_t log_level = MetricReport::WARN;
  for (const auto & evaluation : pet_artifact.object_evaluations) {
    if (evaluation.risk == RiskLevel::SAFE) {
      continue;
    }

    const auto & timing = evaluation.detail;
    const auto & obj_id = timing.object_identification;
    const bool is_error = evaluation.risk == RiskLevel::ERROR;
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
        .metric_value(timing.pet)
        .level(metric_level));
    artifacts.metrics.push_back(
      autoware_trajectory_validator::build<MetricReport>()
        .validator_name(validator_name)
        .validator_category(validator_category)
        .metric_name(
          fmt::format("check_TTC_{}_{}", obj_id.trajectory_id_string(), obj_id.classification))
        .metric_value(timing.ttc)
        .level(metric_level));

    const auto finding_msg = fmt::format(
      "PET collision, classification: {}, ID: {}, PET: {}, TTC: {}, duration: {}, stamp: {}.{};",
      obj_id.classification, obj_id.trajectory_id_string(), timing.pet, timing.ttc,
      pet_continuous_times.get_time(obj_id.trajectory_id_string()), obj_id.stamp.sec,
      obj_id.stamp.nanosec);
    log_messages += finding_msg;
    reporter::append_text_marker_message(marker_messages, finding_msg);
    reporter::add_debug_markers(
      debug_markers, rclcpp::Time{stamp}, "planned_speed_collision", obj_id.trajectory_id_string(),
      timing.ego_trajectory, timing.object_trajectory, timing.ego_hull, timing.object_hull);
    if (is_error) {
      add_collision_planning_factor(
        time_resolution, stamp, ego_pose, timing, "PET", artifacts.planning_factors);
    }
  }

  artifacts.error_msg += marker_messages;
  reporter::log_collision_messages(log_level, log_messages);
}

void process_drac_artifacts(
  const std::string & validator_name, const std::string & validator_category,
  reporter::ContinuousDetectionTimes & drac_continuous_times, const rclcpp::Time & current_time,
  const builtin_interfaces::msg::Time & stamp, const geometry_msgs::msg::Pose & ego_pose,
  const DracArtifact & drac_artifact, EvaluationArtifacts & artifacts,
  visualization_msgs::msg::MarkerArray & debug_markers, double time_resolution)
{
  drac_continuous_times.update(
    current_time, drac_artifact.object_evaluations, [](const auto & evaluation) {
      return evaluation.detail.object_identification.trajectory_id_string();
    });

  if (drac_artifact.risk == RiskLevel::SAFE || drac_artifact.object_evaluations.empty()) {
    return;
  }

  std::string log_messages{};
  std::string marker_messages{};
  const bool has_error = drac_artifact.risk == RiskLevel::ERROR;
  const uint8_t log_level = has_error ? MetricReport::ERROR : MetricReport::WARN;
  for (const auto & evaluation : drac_artifact.object_evaluations) {
    const auto & timing = evaluation.detail;
    const auto & obj_id = timing.object_identification;
    if (has_error) {
      artifacts.is_feasible = false;
    }

    artifacts.metrics.push_back(autoware_trajectory_validator::build<MetricReport>()
                                  .validator_name(validator_name)
                                  .validator_category(validator_category)
                                  .metric_name(fmt::format(
                                    "check_DRAC_{}_{}_{}", obj_id.trajectory_id_string(),
                                    obj_id.classification, obj_id.trajectory_type))
                                  .metric_value(drac_artifact.required_acceleration.value_or(0.0))
                                  .level(log_level));

    const auto finding_msg = fmt::format(
      "DRAC collision, ID: {}, PET: {}, TTC: {}, DRAC: {}, stamp: {}.{};",
      obj_id.trajectory_id_string(), timing.pet, timing.ttc,
      drac_artifact.required_acceleration.has_value()
        ? std::to_string(drac_artifact.required_acceleration.value())
        : "Cant be avoided",
      obj_id.stamp.sec, obj_id.stamp.nanosec);
    log_messages += finding_msg;
    reporter::append_text_marker_message(marker_messages, finding_msg);
    reporter::add_debug_markers(
      debug_markers, rclcpp::Time{stamp}, "drac_collision", obj_id.trajectory_id_string(),
      timing.ego_trajectory, timing.object_trajectory, timing.ego_hull, timing.object_hull);
    if (has_error) {
      add_collision_planning_factor(
        time_resolution, stamp, ego_pose, timing, "DRAC", artifacts.planning_factors);
    }
  }

  artifacts.error_msg += marker_messages;
  reporter::log_collision_messages(log_level, log_messages);
}

void process_rss_artifacts(
  const std::string & validator_name, const std::string & validator_category,
  const RssArtifact & rss_artifact, const TrajectoryPoints & traj_points,
  const FilterContext & context, VehicleInfo & vehicle_info,
  reporter::ContinuousDetectionTimes & rss_continuous_times, const rclcpp::Time & current_time,
  EvaluationArtifacts & artifacts)
{
  std::vector<RssEvaluation> violations{};
  violations.reserve(rss_artifact.object_evaluations.size());
  for (const auto & evaluation : rss_artifact.object_evaluations) {
    if (evaluation.risk == RiskLevel::SAFE) {
      continue;
    }
    violations.push_back(evaluation);
  }
  rss_continuous_times.update(current_time, violations, [](const auto & violation) {
    return violation.detail.object_identification.object_id_string();
  });

  if (rss_artifact.risk == RiskLevel::SAFE || violations.empty()) {
    return;
  }

  std::string log_messages{};
  std::string marker_messages{};
  for (const auto & violation : violations) {
    const auto & detail = violation.detail;
    const auto object_id = detail.object_identification.object_id_string();
    artifacts.is_feasible = false;
    artifacts.metrics.push_back(
      autoware_trajectory_validator::build<MetricReport>()
        .validator_name(validator_name)
        .validator_category(validator_category)
        .metric_name(
          fmt::format("check_RSS_{}_{}", detail.object_identification.classification, object_id))
        .metric_value(detail.rss_acceleration)
        .level(MetricReport::ERROR));

    const auto finding_msg = fmt::format(
      "RSS collision, classification: {}, ID: {}, duration: {}, required deceleration: {}, "
      "stamp: {}.{};",
      detail.object_identification.classification, object_id,
      rss_continuous_times.get_time(object_id), detail.rss_acceleration,
      detail.object_identification.stamp.sec, detail.object_identification.stamp.nanosec);
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

  const auto [pet_artifact, drac_artifact] = collision_timing_assessment::assess(
    traj_points, context, pet_param_map_, drac_param_map_, global_params_, *vehicle_info_ptr_);
  const auto rss_artifact = rss_deceleration::assess(
    traj_points, context, rss_param_map_, global_params_.time_resolution, *vehicle_info_ptr_);

  EvaluationArtifacts artifacts{};
  const rclcpp::Time current_time = context.odometry->header.stamp;
  process_pet_artifacts(
    get_name(), category(), pet_continuous_times_, current_time, context.odometry->header.stamp,
    context.odometry->pose.pose, pet_artifact, artifacts, debug_markers_,
    global_params_.time_resolution);
  process_drac_artifacts(
    get_name(), category(), drac_continuous_times_, current_time, context.odometry->header.stamp,
    context.odometry->pose.pose, drac_artifact, artifacts, debug_markers_,
    global_params_.time_resolution);
  process_rss_artifacts(
    get_name(), category(), rss_artifact, traj_points, context, *vehicle_info_ptr_,
    rss_continuous_times_, current_time, artifacts);
  if (!artifacts.error_msg.empty()) {
    reporter::add_error_text_marker(
      debug_markers_, context.odometry->header.stamp, context.odometry->pose.pose,
      artifacts.error_msg);
  }

  return ValidationResult{
    calc_worst_risk({pet_artifact.risk, drac_artifact.risk, rss_artifact.risk}) != RiskLevel::ERROR,
    std::move(artifacts.metrics), std::move(artifacts.planning_factors)};
}

}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::CollisionCheckFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
