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

#include "autoware/boundary_departure_checker/detail/hysteresis_logic.hpp"

#include "autoware/boundary_departure_checker/detail/severity_evaluator.hpp"

namespace autoware::boundary_departure_checker
{
HysteresisResult update_and_judge(
  const HysteresisState & state,
  const std::optional<Side<std::optional<DeparturePointPair>>> & evaluation_result,
  const UncrossableBoundaryDepartureParam & param, const double current_time_s)
{
  return update_and_judge(
    state, evaluation_result, current_time_s, DepartureType::CRITICAL, param.on_time_buffer_s,
    param.off_time_buffer_s);
}

HysteresisResult update_and_judge(
  const HysteresisState & state,
  const std::optional<Side<std::optional<DeparturePointPair>>> & evaluation_result,
  const double current_time_s, const DepartureType target, const double on_time_buffer_s,
  const double off_time_buffer_s)
{
  const auto matches = [target](const ProjectionToBound & pt) {
    return target == DepartureType::CRITICAL ? pt.is_critical() : pt.is_caution_or_above();
  };

  HysteresisResult result;
  result.updated_state = state;
  result.status = DepartureType::NONE;

  if (!evaluation_result.has_value()) {
    result.updated_state.last_no_critical_dpt_time = current_time_s;
    result.updated_state.critical_departure_history.for_each_side(
      [](auto & side) { side.clear(); });
    return result;
  }

  const bool current_is_critical = evaluation_result->any_of_side([&](const auto & side_value) {
    return side_value.has_value() && matches(side_value->physical_departure_point);
  });

  if (current_is_critical) {
    const bool is_imminent_critical = evaluation_result->any_of_side([&](const auto & side_value) {
      return side_value.has_value() && matches(side_value->physical_departure_point) &&
             side_value->physical_departure_point.time_from_start < on_time_buffer_s;
    });

    if (
      is_imminent_critical ||
      current_time_s - state.last_no_critical_dpt_time >= on_time_buffer_s) {
      result.updated_state.last_found_critical_dpt_time = current_time_s;
      result.updated_state.critical_departure_history.for_each_side(
        [](auto & side) { side.clear(); });
      evaluation_result->for_each([&](auto key_constant, auto & side_value) {
        if (side_value.has_value() && matches(side_value->safety_buffer_start)) {
          result.updated_state.critical_departure_history[key_constant.value].push_back(
            side_value->physical_departure_point);
        }
      });
      result.status = target;
      return result;
    }
    return result;
  }

  result.updated_state.last_no_critical_dpt_time = current_time_s;

  if (!state.critical_departure_history.all_empty()) {
    if (current_time_s - state.last_found_critical_dpt_time < off_time_buffer_s) {
      result.status = target;
      return result;
    }
    result.updated_state.critical_departure_history.for_each_side(
      [](auto & side) { side.clear(); });
  }

  return result;
}

}  // namespace autoware::boundary_departure_checker
