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

#include "metric.hpp"

#include "assessment.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::metric
{
namespace
{
using collision_timing_assessment::kCanonicalTrajectoryTypes;

std::string canonical_trajectory_type(const std::string & raw)
{
  for (const auto * type : kCanonicalTrajectoryTypes) {
    if (raw.find(type) != std::string::npos) {
      return type;
    }
  }
  return {};
}

MetricReport make_metric_report(
  const std::string & validator_name, const std::string & validator_category,
  const std::string & metric_name, double value, uint8_t level)
{
  return autoware_trajectory_validator::build<MetricReport>()
    .validator_name(validator_name)
    .validator_category(validator_category)
    .metric_name(metric_name)
    .metric_value(value)
    .level(level);
}

struct PetWorst
{
  double pet{std::numeric_limits<double>::infinity()};
  bool has_finding{false};

  double metric_value() const
  {
    return has_finding ? pet : std::numeric_limits<double>::infinity();
  }

  uint8_t metric_level(const PetParams & params) const
  {
    if (!has_finding) {
      return MetricReport::OK;
    }
    const bool is_error = pet <= params.error_threshold.ego_first_passing_time_gap &&
                          pet >= -params.error_threshold.object_first_passing_time_gap;
    if (is_error) {
      return MetricReport::ERROR;
    }
    const bool is_warn = pet <= params.warn_threshold.ego_first_passing_time_gap &&
                         pet >= -params.warn_threshold.object_first_passing_time_gap;
    return is_warn ? MetricReport::WARN : MetricReport::OK;
  }
};

struct DracWorst
{
  bool known{false};
  std::optional<double> drac{0.0};

  double metric_value() const
  {
    if (!known) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    if (!drac.has_value()) {
      return std::numeric_limits<double>::max();
    }
    return std::max(0.0, -drac.value());
  }

  uint8_t metric_level(const DracParams & params) const
  {
    if (!known) {
      return MetricReport::OK;
    }
    if (!drac.has_value()) {
      return MetricReport::ERROR;
    }
    const double v = drac.value();
    if (v >= 0.0) {
      return MetricReport::OK;
    }
    if (v < params.error_threshold.ego_acceleration) {
      return MetricReport::ERROR;
    }
    if (v < params.warn_threshold.ego_acceleration) {
      return MetricReport::WARN;
    }
    return MetricReport::OK;
  }
};

struct RssWorst
{
  double required_deceleration{0.0};
  bool has_violation{false};

  double metric_value() const { return required_deceleration; }
  uint8_t metric_level() const { return has_violation ? MetricReport::ERROR : MetricReport::OK; }
};

std::map<std::string, PetWorst> compute_pet_worst(const PetArtifact & pet_artifact)
{
  std::map<std::string, PetWorst> worst;
  for (const auto & ev : pet_artifact.object_evaluations) {
    const auto type = canonical_trajectory_type(ev.detail.object_identification.trajectory_type);
    if (type.empty()) {
      continue;
    }
    auto & w = worst[type];
    if (!w.has_finding || std::abs(ev.detail.pet) < std::abs(w.pet)) {
      w.pet = ev.detail.pet;
      w.has_finding = true;
    }
  }
  return worst;
}

bool is_type_enabled_in_settings(const DracParamMap & m, const std::string & type)
{
  const auto & at = m.at(kCollisionCheckParamBaseKey).assessment_trajectories;
  if (type == "map_based_predicted_path") return at.map_based;
  if (type == "constant_curvature_path") return at.constant_curvature;
  if (type == "diffusion_based_trajectory") return at.diffusion_based;
  return false;
}

std::map<std::string, DracWorst> compute_drac_per_type(
  const DracArtifact & drac_artifact, const DracParamMap & drac_param_map)
{
  std::map<std::string, DracWorst> result;
  for (const auto * type : kCanonicalTrajectoryTypes) {
    result[type] = DracWorst{false, std::nullopt};
  }
  for (const auto & ev : drac_artifact.object_evaluations) {
    const auto t = canonical_trajectory_type(ev.detail.object_identification.trajectory_type);
    if (t.empty()) continue;
    if (!is_type_enabled_in_settings(drac_param_map, t)) continue;
    result[t].known = true;
    result[t].drac = drac_artifact.required_acceleration;
  }
  return result;
}

RssWorst compute_rss_worst(const RssArtifact & rss_artifact)
{
  RssWorst w;
  for (const auto & ev : rss_artifact.object_evaluations) {
    if (ev.detail.rss_acceleration > w.required_deceleration) {
      w.required_deceleration = ev.detail.rss_acceleration;
    }
    if (ev.risk == RiskLevel::ERROR) {
      w.has_violation = true;
    }
  }
  return w;
}

}  // namespace

std::vector<MetricReport> build_metric_reports(
  const std::string & validator_name, const std::string & validator_category,
  const DracArtifact & drac_artifact, const PetArtifact & pet_artifact,
  const RssArtifact & rss_artifact, const DracParamMap & drac_param_map,
  const PetParamMap & pet_param_map)
{
  std::vector<MetricReport> reports;
  reports.reserve(7);

  const auto & pet_params_base = pet_param_map.at(kCollisionCheckParamBaseKey);
  const auto & drac_params_base = drac_param_map.at(kCollisionCheckParamBaseKey);

  const auto pet_worst = compute_pet_worst(pet_artifact);
  for (const auto * type : kCanonicalTrajectoryTypes) {
    const auto it = pet_worst.find(type);
    const auto w = it != pet_worst.end() ? it->second : PetWorst{};
    reports.push_back(make_metric_report(
      validator_name, validator_category, fmt::format("check_PET_{}", type), w.metric_value(),
      w.metric_level(pet_params_base)));
  }

  const auto drac_per_type = compute_drac_per_type(drac_artifact, drac_param_map);
  for (const auto * type : kCanonicalTrajectoryTypes) {
    const auto & w = drac_per_type.at(type);
    reports.push_back(make_metric_report(
      validator_name, validator_category, fmt::format("check_DRAC_{}", type), w.metric_value(),
      w.metric_level(drac_params_base)));
  }

  const auto rss_worst = compute_rss_worst(rss_artifact);
  reports.push_back(make_metric_report(
    validator_name, validator_category, "check_RSS", rss_worst.metric_value(),
    rss_worst.metric_level()));

  return reports;
}

}  // namespace autoware::trajectory_validator::plugin::safety::metric
