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

#ifndef AUTOWARE__ML_PLANNER__PREPROCESSING__ROUTE_SIGNAL_RELEASE_HPP_
#define AUTOWARE__ML_PLANNER__PREPROCESSING__ROUTE_SIGNAL_RELEASE_HPP_

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>

#include <cstdint>
#include <optional>

namespace autoware::ml_planner::preprocess
{

/**
 * @brief True for a color that can change the route signal memory.
 *
 * Unknown is a detection gap, not a new command, so it neither releases the
 * longitudinal memory nor erases the last definite color.
 */
inline bool is_definite_signal_color(const uint8_t color)
{
  using Element = autoware_perception_msgs::msg::TrafficLightElement;
  return color == Element::RED || color == Element::AMBER || color == Element::GREEN ||
         color == Element::WHITE;
}

/**
 * @brief Remember the route signal color and report the cycle it changes.
 *
 * The caller passes the color of the first traffic light on the route, already
 * reduced to the element that matches that lane's turn. Lights elsewhere are
 * not observed. The first definite color only arms the memory. A later change
 * among red, amber, green and white, in either direction, returns true for
 * that one observation. A missing light or an unknown color returns false and
 * leaves the memory in place, so the next definite color is compared with the
 * last one rather than treated as a first sighting.
 */
class RouteSignalRelease
{
public:
  bool observe(const bool on_route, const uint8_t color)
  {
    if (!on_route || !is_definite_signal_color(color)) {
      return false;
    }
    const bool changed = last_color_.has_value() && *last_color_ != color;
    last_color_ = color;
    return changed;
  }

private:
  std::optional<uint8_t> last_color_;
};

}  // namespace autoware::ml_planner::preprocess

#endif  // AUTOWARE__ML_PLANNER__PREPROCESSING__ROUTE_SIGNAL_RELEASE_HPP_
