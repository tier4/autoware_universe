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

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <utility>

namespace autoware::trajectory_validator::plugin::safety::metric
{
std::string canonical_trajectory_type(const std::string & raw)
{
  for (const auto * type : kAggregatedTrajectoryTypes) {
    if (raw.find(type) != std::string::npos) {
      return type;
    }
  }
  return {};
}

DracParamMap isolate_drac_param_map(const DracParamMap & m, const std::string & target_type)
{
  auto isolated = m;
  for (auto & [_, p] : isolated) {
    auto & at = p.assessment_trajectories;
    at.map_based = at.map_based && target_type == "map_based_predicted_path";
    at.constant_curvature = at.constant_curvature && target_type == "constant_curvature_path";
    at.diffusion_based = at.diffusion_based && target_type == "diffusion_based_trajectory";
  }
  return isolated;
}

std::optional<double> combine_per_type_drac(
  const std::map<std::string, std::optional<double>> & per_type)
{
  std::optional<double> overall{0.0};
  for (const auto & [_, opt] : per_type) {
    if (!opt.has_value()) return std::nullopt;
    overall = std::max(overall.value(), opt.value());
  }
  return overall;
}

double PetWorst::metric_value() const
{
  return has_finding ? pet : std::numeric_limits<double>::infinity();
}

uint8_t PetWorst::metric_level(const PetParams & params) const
{
  if (!has_finding) {
    return MetricReport::OK;
  }
  const bool is_error = pet <= params.error_threshold.ego_first_passing_time_gap &&
                        pet >= -params.error_threshold.object_first_passing_time_gap;
  return is_error ? MetricReport::ERROR : MetricReport::WARN;
}

double DracWorst::metric_value() const
{
  return drac.has_value() ? drac.value() : std::numeric_limits<double>::max();
}

uint8_t DracWorst::metric_level(const DracParams & params) const
{
  if (!drac.has_value() || drac.value() < params.error_threshold.ego_acceleration) {
    return MetricReport::ERROR;
  }
  if (drac.value() < params.warn_threshold.ego_acceleration) {
    return MetricReport::WARN;
  }
  return MetricReport::OK;
}

double RssWorst::metric_value() const
{
  return required_deceleration;
}

uint8_t RssWorst::metric_level() const
{
  return has_violation ? MetricReport::ERROR : MetricReport::OK;
}

std::map<std::string, PetWorst> compute_pet_worst(const PetArtifact & artifact)
{
  std::map<std::string, PetWorst> worst;
  for (const auto * type : kAggregatedTrajectoryTypes) {
    worst[type] = PetWorst{};
  }
  for (const auto & evaluation : artifact.object_evaluations) {
    const auto type =
      canonical_trajectory_type(evaluation.detail.object_identification.trajectory_type);
    if (type.empty()) {
      continue;
    }
    auto & w = worst[type];
    if (!w.has_finding || std::abs(evaluation.detail.pet) < std::abs(w.pet)) {
      w.pet = evaluation.detail.pet;
      w.has_finding = true;
    }
  }
  return worst;
}

std::map<std::string, DracWorst> compute_drac_worst(
  const std::map<std::string, std::optional<double>> & required_acceleration_by_type)
{
  std::map<std::string, DracWorst> worst;
  for (const auto * type : kAggregatedTrajectoryTypes) {
    const auto it = required_acceleration_by_type.find(type);
    worst[type] = DracWorst{
      it != required_acceleration_by_type.end() ? it->second : std::optional<double>{0.0}};
  }
  return worst;
}

RssWorst compute_rss_worst(const RssArtifact & artifact)
{
  double worst = 0.0;
  for (const auto & evaluation : artifact.object_evaluations) {
    if (evaluation.detail.rss_acceleration > worst) {
      worst = evaluation.detail.rss_acceleration;
    }
  }
  return RssWorst{worst, artifact.risk != RiskLevel::SAFE};
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
}  // namespace autoware::trajectory_validator::plugin::safety::metric
