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

#ifndef AUTOWARE__MPPI_OPTIMIZER__DETAIL__LOCAL_FRAME_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__DETAIL__LOCAL_FRAME_HPP_

#include <cmath>
#include <stdexcept>
#include <string>

namespace autoware::mppi_optimizer::detail
{

/** Translation-only frame for float GPU positions. Subtraction always happens in double. */
struct LocalFrame
{
  double origin_x{0.0};
  double origin_y{0.0};
  std::string frame_id;
  bool valid{false};

  [[nodiscard]] bool needsRecenter(
    const double x, const double y, const std::string & candidate_frame_id) const
  {
    constexpr double kRecenterDistanceM = 32.0;
    return !valid || frame_id != candidate_frame_id ||
           std::abs(x - origin_x) > kRecenterDistanceM ||
           std::abs(y - origin_y) > kRecenterDistanceM;
  }

  void reset(const double x, const double y, const std::string & candidate_frame_id)
  {
    if (!std::isfinite(x) || !std::isfinite(y)) {
      throw std::invalid_argument("MPPI local-frame origin must be finite");
    }
    origin_x = x;
    origin_y = y;
    frame_id = candidate_frame_id;
    valid = true;
  }

  [[nodiscard]] float x(const double world_x) const
  {
    if (!valid || !std::isfinite(world_x)) {
      throw std::invalid_argument("MPPI local-frame x must be finite");
    }
    const float local_x = static_cast<float>(world_x - origin_x);
    if (!std::isfinite(local_x)) {
      throw std::invalid_argument("MPPI local-frame x exceeds float range");
    }
    return local_x;
  }

  [[nodiscard]] float y(const double world_y) const
  {
    if (!valid || !std::isfinite(world_y)) {
      throw std::invalid_argument("MPPI local-frame y must be finite");
    }
    const float local_y = static_cast<float>(world_y - origin_y);
    if (!std::isfinite(local_y)) {
      throw std::invalid_argument("MPPI local-frame y exceeds float range");
    }
    return local_y;
  }

  [[nodiscard]] double worldX(const float local_x) const
  {
    return origin_x + static_cast<double>(local_x);
  }

  [[nodiscard]] double worldY(const float local_y) const
  {
    return origin_y + static_cast<double>(local_y);
  }
};

}  // namespace autoware::mppi_optimizer::detail

#endif  // AUTOWARE__MPPI_OPTIMIZER__DETAIL__LOCAL_FRAME_HPP_
