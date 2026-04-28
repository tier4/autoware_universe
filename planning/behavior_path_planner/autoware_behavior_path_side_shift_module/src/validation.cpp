// Copyright 2023 TIER IV, Inc.
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

#include "autoware/behavior_path_side_shift_module/validation.hpp"

#include "autoware/behavior_path_side_shift_module/manager.hpp"
#include "autoware/behavior_path_side_shift_module/utils.hpp"

#include <autoware_common_msgs/msg/response_status.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

namespace autoware::behavior_path_planner
{
using ResponseStatus = autoware_common_msgs::msg::ResponseStatus;

std::pair<uint16_t, double> validateAndComputeLateralOffset(
  const SetLateralOffset::Request & request, const double current_inserted_lateral_offset,
  const std::shared_ptr<SideShiftParameters> & parameters,
  const std::pair<double, double> & lanelet_limits)
{
  constexpr double epsilon = 1.0e-4;

  const double right_limit = std::max(lanelet_limits.first, -parameters->max_shift_magnitude);
  const double left_limit = std::min(lanelet_limits.second, parameters->max_shift_magnitude);

  // RESET is always permitted
  if (
    request.shift_mode == SetLateralOffset::Request::LATERAL_OFFSET_DIRECTION &&
    request.shift_direction_value == SetLateralOffset::Request::RESET) {
    return {SetLateralOffset::Response::SUCCESS, 0.0};
  }

  // validate raw value first
  if (request.shift_mode == SetLateralOffset::Request::EXPLICIT_LATERAL_OFFSET_AMOUNT) {
    return validateRawValue(
      request.shift_value, current_inserted_lateral_offset, left_limit, right_limit,
      parameters->min_shift_gap);
  }

  // check invalid mode is requested
  if (request.shift_mode != SetLateralOffset::Request::LATERAL_OFFSET_DIRECTION) {
    return {SetLateralOffset::Response::ERROR_INVALID_MODE, 0.0};
  }

  // The remaining validation is about direction values
  if (
    request.shift_direction_value != SetLateralOffset::Request::LEFT &&
    request.shift_direction_value != SetLateralOffset::Request::RIGHT) {
    return {SetLateralOffset::Response::ERROR_INVALID_DIRECTION, 0.0};
  }

  double requested_offset = current_inserted_lateral_offset;
  if (request.shift_direction_value == SetLateralOffset::Request::LEFT) {
    if (current_inserted_lateral_offset > left_limit - epsilon) {
      return {SetLateralOffset::Response::ERROR_EXCEEDED_LIMIT, current_inserted_lateral_offset};
    }

    // If the vehicle is going left from right, return to zero first
    requested_offset = (current_inserted_lateral_offset < 0 &&
                        current_inserted_lateral_offset + parameters->unit_shift_amount > 0)
                         ? 0.0
                         : requested_offset + parameters->unit_shift_amount;

    if (requested_offset > left_limit) {
      requested_offset = left_limit;
      return {SetLateralOffset::Response::WARN_EXCEEDED_LIMIT, requested_offset};
    }
  }

  if (request.shift_direction_value == SetLateralOffset::Request::RIGHT) {
    if (current_inserted_lateral_offset < right_limit + epsilon) {
      return {SetLateralOffset::Response::ERROR_EXCEEDED_LIMIT, current_inserted_lateral_offset};
    }

    // If the vehicle is going right from left, return to zero first
    requested_offset = (current_inserted_lateral_offset > 0 &&
                        current_inserted_lateral_offset - parameters->unit_shift_amount < 0)
                         ? 0.0
                         : requested_offset - parameters->unit_shift_amount;

    if (requested_offset < right_limit) {
      requested_offset = right_limit;
      return {SetLateralOffset::Response::WARN_EXCEEDED_LIMIT, requested_offset};
    }
  }

  // This should only happen when the unit_shift_amount is smaller than the min_shift_gap
  if (std::fabs(requested_offset - current_inserted_lateral_offset) < parameters->min_shift_gap) {
    return {SetLateralOffset::Response::ERROR_SHIFT_GAP_TOO_SMALL, current_inserted_lateral_offset};
  }

  return {SetLateralOffset::Response::SUCCESS, requested_offset};
}

std::pair<uint16_t, double> validateRawValue(
  const double lateral_offset, const double current_inserted_lateral_offset,
  const double left_limit, const double right_limit, const double min_shift_gap)
{
  constexpr double epsilon = 1.0e-4;

  // Check if the current inserted lateral offset is already at limit
  if (lateral_offset < right_limit && current_inserted_lateral_offset < right_limit + epsilon) {
    return {SetLateralOffset::Response::ERROR_EXCEEDED_LIMIT, current_inserted_lateral_offset};
  }
  if (lateral_offset > left_limit && current_inserted_lateral_offset > left_limit - epsilon) {
    return {SetLateralOffset::Response::ERROR_EXCEEDED_LIMIT, current_inserted_lateral_offset};
  }

  // Clamp the lateral offset to the limit if request is out of limit
  if (lateral_offset < right_limit) {
    return {SetLateralOffset::Response::WARN_EXCEEDED_LIMIT, right_limit};
  }
  if (lateral_offset > left_limit) {
    return {SetLateralOffset::Response::WARN_EXCEEDED_LIMIT, left_limit};
  }

  // Check if the requested shift is too small
  if (std::fabs(lateral_offset - current_inserted_lateral_offset) < min_shift_gap) {
    return {SetLateralOffset::Response::ERROR_SHIFT_GAP_TOO_SMALL, current_inserted_lateral_offset};
  }

  return {SetLateralOffset::Response::SUCCESS, lateral_offset};
}

const char * getStatusMessage(uint16_t status_code)
{
  switch (status_code) {
    case SetLateralOffset::Response::UNKNOWN:
      return "Unknown or uninitialized status";
    case SetLateralOffset::Response::SUCCESS:
      return "Updated successfully";
    case SetLateralOffset::Response::ERROR_UNKNOWN:
      return "Unknown error occurred";
    case SetLateralOffset::Response::ERROR_INVALID_MODE:
      return "Invalid mode have been requested";
    case SetLateralOffset::Response::ERROR_INVALID_DIRECTION:
      return "Invalid direction have been requested";
    case SetLateralOffset::Response::ERROR_EXCEEDED_LIMIT:
      return "The vehicle cannot shift further than request";
    case SetLateralOffset::Response::ERROR_SHIFT_GAP_TOO_SMALL:
      return "The requested shift length differs too small from the current shift length";
    case SetLateralOffset::Response::ERROR_MODULES_CONFLICTING:
      return "Other modules are running in the behavior_path_planner";
    case SetLateralOffset::Response::WARN_UNKNOWN:
      return "Unknown warning occurred";
    case SetLateralOffset::Response::WARN_EXCEEDED_LIMIT:
      return "The shift length reached limit. Shifted to the possible end.";
    case ResponseStatus::UNKNOWN:
      return "Unknown response status";
    case ResponseStatus::SERVICE_UNREADY:
      return "The side_shift module is not ready";
    case ResponseStatus::PARAMETER_ERROR:
      return "Invalid parameters defined in the side_shift module";
    default:
      return "Unknown response_status";
  }
}

}  // namespace autoware::behavior_path_planner
