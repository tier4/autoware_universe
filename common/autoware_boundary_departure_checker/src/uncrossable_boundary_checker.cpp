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

#include "autoware/boundary_departure_checker/uncrossable_boundary_checker.hpp"

#include "autoware/boundary_departure_checker/detail/debug.hpp"
#include "autoware/boundary_departure_checker/detail/footprints_generator.hpp"
#include "autoware/boundary_departure_checker/detail/severity_evaluator.hpp"

#include <autoware_utils_system/stop_watch.hpp>

#include <algorithm>
#include <memory>

namespace autoware::boundary_departure_checker
{
UncrossableBoundaryChecker::UncrossableBoundaryChecker(
  const lanelet::LaneletMapPtr & map, const UncrossableBoundaryDepartureParam & param,
  const VehicleInfo & vehicle_info)
: param_(param), vehicle_info_(vehicle_info)
{
  evaluator_ptr_ = std::make_unique<BoundaryDepartureEvaluator>(map, param, vehicle_info);
}

void UncrossableBoundaryChecker::update_parameters(const UncrossableBoundaryDepartureParam & param)
{
  param_ = param;
  evaluator_ptr_->update_parameters(param);
}

std::optional<UncrossableBoundaryChecker::TrajectoryEvaluation>
UncrossableBoundaryChecker::evaluate_trajectory(
  const TrajectoryPoints & predicted_traj, const EgoDynamicState & ego_state) const
{
  if (predicted_traj.empty()) {
    return std::nullopt;
  }

  TrajectoryEvaluation evaluated;

  // An align distance of zero leaves the trajectory that the generator gives.
  const auto align_dist_m =
    param_.enable_align_to_ego_pose ? vehicle_info_.max_longitudinal_offset_m : 0.0;
  const auto expected_alignment =
    footprints::count_points_within_distance(predicted_traj, align_dist_m);
  const auto aligned_traj =
    footprints::align_to_ego_pose(predicted_traj, ego_state.pose_with_cov.pose, expected_alignment);

  evaluated.footprints = footprints::generate(aligned_traj, vehicle_info_, ego_state.pose_with_cov);
  evaluated.align_count = expected_alignment.align_count;
  const auto footprints_sides = footprints::get_sides_from_footprints(evaluated.footprints);

  evaluated.evaluation_result = evaluator_ptr_->evaluate(aligned_traj, footprints_sides, ego_state);
  return evaluated;
}

std::optional<double> UncrossableBoundaryChecker::get_lat_dist_to_bound(
  const TrajectoryEvaluation & evaluated)
{
  if (!evaluated.evaluation_result) {
    return std::nullopt;
  }
  return severity_evaluator::get_min_lateral_distance_to_bound(*evaluated.evaluation_result);
}

visualization_msgs::msg::MarkerArray UncrossableBoundaryChecker::create_debug_markers(
  const HysteresisState & critical_state, const TrajectoryEvaluation & evaluated,
  const EgoDynamicState & ego_state) const
{
  return debug::create_debug_markers(
    critical_state, evaluated.footprints, evaluated.align_count, ego_state,
    param_.enable_developer_marker);
}

DepartureResult UncrossableBoundaryChecker::update_departure_status(
  const TrajectoryPoints & predicted_traj, const EgoDynamicState & ego_state,
  HysteresisState & state)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  DepartureResult result;
  const auto evaluated = evaluate_trajectory(predicted_traj, ego_state);
  if (!evaluated) {
    return result;
  }
  const auto & evaluation_result = evaluated->evaluation_result;

  const auto hysteresis_result =
    update_and_judge(state, evaluation_result, param_, ego_state.current_time_s);

  state = hysteresis_result.updated_state;

  // CRITICAL is hysteresis filtered, whereas NEAR_BOUNDARY is a non-latching advisory taken
  // directly from the current evaluation. A CRITICAL projection that the hysteresis buffer still
  // suppresses is reported as NEAR_BOUNDARY, so the reported severity never dips back to NONE while
  // ego closes on the boundary.
  const auto footprint_is_close_to_bound =
    evaluation_result && (severity_evaluator::is_near_boundary(*evaluation_result) ||
                          severity_evaluator::is_critical(*evaluation_result));
  result.status = hysteresis_result.status;
  if (result.status == DepartureType::NONE && footprint_is_close_to_bound) {
    result.status = DepartureType::NEAR_BOUNDARY;
  }

  if (const auto lat_dist = get_lat_dist_to_bound(*evaluated)) {
    result.lat_dist_to_uncrossable_bound = *lat_dist;
  }

  result.debug_markers = create_debug_markers(state, *evaluated, ego_state);
  return result;
}

DepartureResult UncrossableBoundaryChecker::update_departure_status(
  const TrajectoryPoints & predicted_traj, const EgoDynamicState & ego_state,
  HysteresisStatePair & states)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  DepartureResult result;
  const auto evaluated = evaluate_trajectory(predicted_traj, ego_state);
  if (!evaluated) {
    return result;
  }
  const auto & evaluation_result = evaluated->evaluation_result;

  const auto critical_result =
    update_and_judge(states.critical, evaluation_result, param_, ego_state.current_time_s);
  states.critical = critical_result.updated_state;

  DepartureType caution_status = DepartureType::NONE;
  if (param_.enable_caution) {
    const auto caution_result = update_and_judge(
      states.caution, evaluation_result, ego_state.current_time_s, DepartureType::CAUTION,
      param_.caution_on_time_buffer_s, param_.caution_off_time_buffer_s);
    states.caution = caution_result.updated_state;
    caution_status = caution_result.status;
  }

  if (critical_result.status == DepartureType::CRITICAL) {
    result.status = DepartureType::CRITICAL;
  } else if (caution_status == DepartureType::CAUTION) {
    result.status = DepartureType::CAUTION;
  } else {
    // Same non-latching NEAR_BOUNDARY promotion as the single-state overload. A suppressed
    // CAUTION point counts as close only when it is also inside the near-boundary band.
    const auto th_near_boundary =
      std::max(param_.near_boundary_lateral_th_m, param_.critical_departure_lateral_th_m);
    const auto footprint_is_close_to_bound =
      evaluation_result && (severity_evaluator::is_near_boundary(*evaluation_result) ||
                            severity_evaluator::is_critical(*evaluation_result) ||
                            (severity_evaluator::is_caution_or_above(*evaluation_result) &&
                             severity_evaluator::get_min_lateral_distance_to_bound(
                               *evaluation_result) <= th_near_boundary));
    result.status =
      footprint_is_close_to_bound ? DepartureType::NEAR_BOUNDARY : DepartureType::NONE;
  }

  if (const auto lat_dist = get_lat_dist_to_bound(*evaluated)) {
    result.lat_dist_to_uncrossable_bound = *lat_dist;
  }

  result.debug_markers = create_debug_markers(states.critical, *evaluated, ego_state);
  return result;
}

}  // namespace autoware::boundary_departure_checker
