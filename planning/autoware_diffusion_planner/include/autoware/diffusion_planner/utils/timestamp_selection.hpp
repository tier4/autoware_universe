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

#ifndef AUTOWARE__DIFFUSION_PLANNER__UTILS__TIMESTAMP_SELECTION_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__UTILS__TIMESTAMP_SELECTION_HPP_

#include <cstdint>
#include <stdexcept>

namespace autoware::diffusion_planner::utils
{

struct TimestampSelection
{
  int64_t time_ns{0};
  bool used_header_fallback{false};
};

/**
 * @brief Select the timestamp in the clock domain used by the planner.
 *
 * The receive timestamp is preferred when it matches the planner clock because the training
 * converter uses recorder/arrival time. A matching source timestamp is the next best option when
 * middleware reports receive time in another domain. The payload header is only used when neither
 * transport timestamp is usable in the planner clock domain.
 */
inline TimestampSelection select_timestamp(
  const int64_t received_timestamp, const int64_t source_timestamp,
  const int64_t header_timestamp, const int64_t reference_time_ns,
  const int64_t clock_domain_mismatch_ns)
{
  const auto is_near_reference = [reference_time_ns, clock_domain_mismatch_ns](
                                   const int64_t timestamp_ns) {
    if (timestamp_ns <= 0 || reference_time_ns <= 0) {
      return true;
    }
    const int64_t difference = timestamp_ns > reference_time_ns
                                 ? timestamp_ns - reference_time_ns
                                 : reference_time_ns - timestamp_ns;
    return difference <= clock_domain_mismatch_ns;
  };

  if (received_timestamp > 0 && is_near_reference(received_timestamp)) {
    return TimestampSelection{received_timestamp, false};
  }
  if (source_timestamp > 0 && is_near_reference(source_timestamp)) {
    return TimestampSelection{source_timestamp, false};
  }
  if (header_timestamp > 0 && is_near_reference(header_timestamp)) {
    return TimestampSelection{header_timestamp, true};
  }

  // No candidate is in the current clock domain. Preserve deterministic stale-data handling by
  // keeping the first available transport timestamp; the caller's age gate will reject it. If no
  // transport timestamp exists, the header is still the only useful fallback.
  if (received_timestamp > 0) {
    return TimestampSelection{received_timestamp, false};
  }
  if (source_timestamp > 0) {
    return TimestampSelection{source_timestamp, false};
  }
  if (header_timestamp > 0) {
    return TimestampSelection{header_timestamp, true};
  }

  throw std::runtime_error("No usable received, source, or header timestamp");
}

}  // namespace autoware::diffusion_planner::utils

#endif  // AUTOWARE__DIFFUSION_PLANNER__UTILS__TIMESTAMP_SELECTION_HPP_
