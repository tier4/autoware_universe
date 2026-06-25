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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__ERROR_HYSTERESIS_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__ERROR_HYSTERESIS_HPP_

namespace autoware::trajectory_validator
{
/**
 * @brief Class for error hysteresis.
 */
class ErrorHysteresis
{
public:
  bool isErrorHysteresisActive(const double current_time_s) const;

private:
  double on_time_buffer_s_{0.15};   //!< Buffer time for activating error hysteresis.
  double off_time_buffer_s_{0.15};  //!< Buffer time for deactivating error hysteresis.
};
}  // namespace autoware::trajectory_validator
#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__ERROR_HYSTERESIS_HPP_
