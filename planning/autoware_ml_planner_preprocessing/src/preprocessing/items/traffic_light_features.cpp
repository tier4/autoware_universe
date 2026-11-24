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

#include "autoware/ml_planner/preprocessing/items/traffic_light_features.hpp"

#include "autoware/ml_planner/dimensions.hpp"
#include "autoware/ml_planner/preprocessing/items/map.hpp"

#include <xtensor/xbuilder.hpp>

#include <algorithm>
#include <tuple>

namespace autoware::ml_planner::preprocess
{
namespace
{
TrafficLightShape shape_for_direction(const int64_t direction)
{
  switch (direction) {
    case LaneSegment::TURN_DIRECTION_STRAIGHT:
      return TrafficLightShape::Up;
    case LaneSegment::TURN_DIRECTION_LEFT:
      return TrafficLightShape::Left;
    case LaneSegment::TURN_DIRECTION_RIGHT:
      return TrafficLightShape::Right;
    default:
      return TrafficLightShape::Unknown;
  }
}

bool is_arrow(const TrafficLightShape shape)
{
  return shape == TrafficLightShape::Left || shape == TrafficLightShape::Right ||
         shape == TrafficLightShape::Up || shape == TrafficLightShape::Down ||
         shape == TrafficLightShape::DownLeft || shape == TrafficLightShape::DownRight;
}
}  // namespace

TrafficLightFeature select_traffic_light(
  const int64_t turn_direction, const std::vector<TrafficLightObservation> & elements)
{
  const auto target = shape_for_direction(turn_direction);
  const auto priority = [target](const TrafficLightObservation & element) {
    if (element.color == TrafficLightColor::Unknown) {
      return 0;
    }
    if (element.shape == target) {
      return 3;
    }
    return element.shape == TrafficLightShape::Circle ? 2 : 1;
  };

  const auto best = std::max_element(
    elements.begin(), elements.end(), [&](const auto & a, const auto & b) {
      return std::make_tuple(priority(a), a.confidence) < std::make_tuple(priority(b), b.confidence);
    });
  if (best == elements.end() || best->color == TrafficLightColor::Unknown) {
    return {};
  }
  return {best->color, is_arrow(best->shape)};
}

xt::xarray<float> infer_traffic_light_future(const xt::xarray<float> & past)
{
  const size_t segments = past.shape().at(0);
  xt::xarray<float> future = xt::zeros<float>(
    {segments, static_cast<size_t>(OUTPUT_T), static_cast<size_t>(TRAFFIC_LIGHT_ONE_HOT_DIM)});
  constexpr size_t amber_index = 1;
  constexpr size_t red_index = 2;
  constexpr size_t amber_duration_steps = 30;

  for (size_t segment = 0; segment < segments; ++segment) {
    const size_t current_t = past.shape().at(1) - 1;
    bool has_state = false;
    for (size_t d = 0; d < static_cast<size_t>(TRAFFIC_LIGHT_ONE_HOT_DIM); ++d) {
      has_state = has_state || past(segment, current_t, d) != 0.0F;
    }
    if (!has_state) {
      continue;
    }

    size_t remaining_amber_steps = 0;
    if (past(segment, current_t, amber_index) > 0.5F) {
      size_t elapsed_steps = 0;
      for (size_t t = current_t + 1; t > 0; --t) {
        if (past(segment, t - 1, amber_index) <= 0.5F) {
          break;
        }
        ++elapsed_steps;
      }
      remaining_amber_steps =
        amber_duration_steps > elapsed_steps ? amber_duration_steps - elapsed_steps : 0;
    }

    for (size_t t = 0; t < static_cast<size_t>(OUTPUT_T); ++t) {
      if (past(segment, current_t, amber_index) > 0.5F && t >= remaining_amber_steps) {
        future(segment, t, red_index) = 1.0F;
      } else {
        for (size_t d = 0; d < static_cast<size_t>(TRAFFIC_LIGHT_ONE_HOT_DIM); ++d) {
          future(segment, t, d) = past(segment, current_t, d);
        }
      }
    }
  }
  return future;
}

}  // namespace autoware::ml_planner::preprocess
