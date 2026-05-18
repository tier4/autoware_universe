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

#include <fmt/core.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
void CollisionCheckFilter::update_parameters(const validator::Params & node_params)
{
  global_params_ = GlobalParams(node_params.collision_check.global_setting);

  drac_param_map_ = create_param_map_per_object<DracParams>(node_params);
  pet_param_map_ = create_param_map_per_object<PetParams>(node_params);
  rss_param_map_ = create_param_map_per_object<RssParams>(node_params);
}

void CollisionCheckFilter::clear_detection_times()
{
  pet_continuous_times_.clear();
  rss_continuous_times_.clear();
  drac_continuous_times_.clear();
}

std::vector<MetricReport> CollisionCheckFilter::generate_metric_reports(
  const DracArtifact & drac_artifact, const PetArtifact & pet_artifact,
  const RssArtifact & rss_artifact) const
{
  std::vector<MetricReport> reports;

  const auto convert_metrics_level = [](const RiskLevel risk_level) {
    switch (risk_level) {
      case RiskLevel::SAFE:
        return MetricReport::OK;
      case RiskLevel::WARN:
        return MetricReport::WARN;
      case RiskLevel::ERROR:
        return MetricReport::ERROR;
      default:
        throw std::runtime_error("invalid argument");
    }
  };

  const auto add_report =
    [&](const std::string_view metric_name, double metric_value, RiskLevel risk) {
      reports.push_back(autoware_trajectory_validator::build<MetricReport>()
                          .validator_name(get_name())
                          .validator_category(category())
                          .metric_name(std::string(metric_name))
                          .metric_value(metric_value)
                          .level(convert_metrics_level(risk)));
    };

  static constexpr std::array<const char *, 3> kCanonicalTrajectoryTypes = {
    "map_based_predicted_path",
    "constant_curvature_path",
    "diffusion_based_trajectory",
  };

  const auto canonical_type_index = [&](const std::string & raw) -> int {
    for (size_t i = 0; i < kCanonicalTrajectoryTypes.size(); ++i) {
      if (raw.find(kCanonicalTrajectoryTypes[i]) != std::string::npos) {
        return static_cast<int>(i);
      }
    }
    return -1;
  };

  const auto pet_risk_level = [](double pet, const PetParams & params) {
    const bool is_error = pet <= params.error_threshold.ego_first_passing_time_gap &&
                          pet >= -params.error_threshold.object_first_passing_time_gap;
    if (is_error) return RiskLevel::ERROR;
    const bool is_warn = pet <= params.warn_threshold.ego_first_passing_time_gap &&
                         pet >= -params.warn_threshold.object_first_passing_time_gap;
    return is_warn ? RiskLevel::WARN : RiskLevel::SAFE;
  };

  const auto is_drac_type_enabled = [&](size_t i) {
    const auto & at = drac_param_map_.at(kCollisionCheckParamBaseKey).assessment_trajectories;
    if (i == 0) return at.map_based;
    if (i == 1) return at.constant_curvature;
    if (i == 2) return at.diffusion_based;
    return false;
  };

  const auto & pet_params_base = pet_param_map_.at(kCollisionCheckParamBaseKey);
  std::array<std::optional<double>, 3> pet_worst{};
  for (const auto & ev : pet_artifact.object_evaluations) {
    const int idx = canonical_type_index(ev.detail.object_identification.trajectory_type);
    if (idx < 0) continue;
    auto & cur = pet_worst[idx];
    if (!cur.has_value() || std::abs(ev.detail.pet) < std::abs(cur.value())) {
      cur = ev.detail.pet;
    }
  }
  for (size_t i = 0; i < kCanonicalTrajectoryTypes.size(); ++i) {
    const double value =
      pet_worst[i].has_value() ? pet_worst[i].value() : std::numeric_limits<double>::infinity();
    const RiskLevel level = pet_worst[i].has_value()
                              ? pet_risk_level(pet_worst[i].value(), pet_params_base)
                              : RiskLevel::SAFE;
    add_report(fmt::format("check_PET_{}", kCanonicalTrajectoryTypes[i]), value, level);
  }

  const auto & drac_params_base = drac_param_map_.at(kCollisionCheckParamBaseKey);
  std::array<bool, 3> drac_has_finding{false, false, false};
  for (const auto & ev : drac_artifact.object_evaluations) {
    const int idx = canonical_type_index(ev.detail.object_identification.trajectory_type);
    if (idx < 0) continue;
    drac_has_finding[idx] = true;
  }
  for (size_t i = 0; i < kCanonicalTrajectoryTypes.size(); ++i) {
    const bool known = drac_has_finding[i] && is_drac_type_enabled(i);
    double value;
    RiskLevel level;
    if (!known) {
      value = std::numeric_limits<double>::quiet_NaN();
      level = RiskLevel::SAFE;
    } else if (!drac_artifact.required_acceleration.has_value()) {
      value = std::numeric_limits<double>::max();
      level = RiskLevel::ERROR;
    } else {
      const double v = drac_artifact.required_acceleration.value();
      value = std::max(0.0, -v);
      if (v >= 0.0) {
        level = RiskLevel::SAFE;
      } else if (v < drac_params_base.error_threshold.ego_acceleration) {
        level = RiskLevel::ERROR;
      } else if (v < drac_params_base.warn_threshold.ego_acceleration) {
        level = RiskLevel::WARN;
      } else {
        level = RiskLevel::SAFE;
      }
    }
    add_report(fmt::format("check_DRAC_{}", kCanonicalTrajectoryTypes[i]), value, level);
  }

  double rss_worst_value = 0.0;
  bool rss_has_violation = false;
  for (const auto & ev : rss_artifact.object_evaluations) {
    if (ev.detail.rss_acceleration > rss_worst_value) {
      rss_worst_value = ev.detail.rss_acceleration;
    }
    if (ev.risk == RiskLevel::ERROR) {
      rss_has_violation = true;
    }
  }
  add_report(
    "check_RSS", rss_worst_value, rss_has_violation ? RiskLevel::ERROR : RiskLevel::SAFE);

  return reports;
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

  auto planning_factors = reporter::process_collision_artifacts(
    *context.odometry, pet_artifact, pet_continuous_times_, drac_artifact,
    drac_continuous_times_, rss_artifact, rss_continuous_times_, debug_markers_,
    global_params_.time_resolution);

  return ValidationResult{
    calc_worst_risk({pet_artifact.risk, drac_artifact.risk, rss_artifact.risk}) != RiskLevel::ERROR,
    generate_metric_reports(drac_artifact, pet_artifact, rss_artifact),
    std::move(planning_factors)};
}

}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::CollisionCheckFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
