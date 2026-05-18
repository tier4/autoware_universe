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
#include "metric.hpp"

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
    metric::build_metric_reports(
      get_name(), category(), drac_artifact, pet_artifact, rss_artifact, drac_param_map_,
      pet_param_map_),
    std::move(planning_factors)};
}

}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::CollisionCheckFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
