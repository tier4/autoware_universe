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

#ifndef AUTOWARE__TENSORRT_E2E__DERIVED_CONTRACT_HPP_
#define AUTOWARE__TENSORRT_E2E__DERIVED_CONTRACT_HPP_

#include <array>
#include <string>
#include <string_view>

namespace autoware::tensorrt_e2e
{

/**
 * @brief The derived-data conventions (e2e-data-producer ``format_version``) whose inputs
 * this node feeds a model trained with recorded ego dynamics; see initialize_pipeline().
 *
 * derived-v10: the ego state of 9 (measured steering_tire_angle, unmodified yaw rate, twist
 * at or before and pose at the LiDAR time) and the producer's map conversion
 * (MapConversionOptions::oneplanner_derived_v10).
 *
 * derived-v12 feeds the same. 11 added the real frame timestamps and interpolates the
 * native-rate ego poses at them, as this node interpolates its ego history on the model's
 * 0.1 s grid (context.use_time_interpolation); 12 added each lanes / route row's lanelet ID,
 * which no model reads. Built from v10 and from v12, the same training rows give every
 * model input bit-identical except the ego history, within 3.2 mm.
 */
inline constexpr std::array<std::string_view, 2> DERIVED_CONTRACTS{"derived-v10", "derived-v12"};

inline bool feeds_derived_contract(std::string_view contract)
{
  for (const auto fed : DERIVED_CONTRACTS) {
    if (contract == fed) {
      return true;
    }
  }
  return false;
}

//! The conventions as a message lists them: 'derived-v10', 'derived-v12'.
inline std::string derived_contracts_text()
{
  std::string text;
  for (const auto fed : DERIVED_CONTRACTS) {
    text += (text.empty() ? "'" : ", '") + std::string(fed) + "'";
  }
  return text;
}

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__DERIVED_CONTRACT_HPP_
