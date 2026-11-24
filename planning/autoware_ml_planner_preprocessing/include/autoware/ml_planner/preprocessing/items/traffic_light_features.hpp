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

#ifndef AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__TRAFFIC_LIGHT_FEATURES_HPP_
#define AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__TRAFFIC_LIGHT_FEATURES_HPP_

#include <xtensor/xarray.hpp>

#include <cstdint>
#include <vector>

namespace autoware::ml_planner::preprocess
{

// Numeric values match T4Scene v0.0.0 and Autoware's TrafficLightElement.
enum class TrafficLightColor : int64_t { Unknown = 0, Red = 1, Amber = 2, Green = 3, White = 4 };
enum class TrafficLightShape : int64_t {
  Unknown = 0,
  Circle = 1,
  Left = 2,
  Right = 3,
  Up = 4,
  Down = 7,
  DownLeft = 8,
  DownRight = 9,
};

struct TrafficLightObservation
{
  TrafficLightColor color;
  TrafficLightShape shape;
  float confidence;
};

struct TrafficLightFeature
{
  TrafficLightColor color{TrafficLightColor::Unknown};
  bool is_arrow{false};
};

/// Select the effective light for the given lane direction.
TrafficLightFeature select_traffic_light(
  int64_t turn_direction, const std::vector<TrafficLightObservation> & elements);

/// Infer eight seconds from the observed light history with a three-second amber cycle.
xt::xarray<float> infer_traffic_light_future(const xt::xarray<float> & past);

}  // namespace autoware::ml_planner::preprocess

#endif  // AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__TRAFFIC_LIGHT_FEATURES_HPP_
