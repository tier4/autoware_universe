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

#ifndef FILTERS__SAFETY__COLLISION_CHECK_FILTER__PARAMETER_HPP_
#define FILTERS__SAFETY__COLLISION_CHECK_FILTER__PARAMETER_HPP_

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_trajectory_validator/autoware_trajectory_validator_param.hpp>

#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
using autoware_perception_msgs::msg::ObjectClassification;

inline constexpr const char * kCollisionCheckParamBaseKey = "base";
// Keep in sync with `parameter_struct.yaml`.
inline constexpr std::pair<uint8_t, std::string_view> kObjectClassifications[] = {
  {ObjectClassification::CAR, "car"},
  {ObjectClassification::TRUCK, "truck"},
  {ObjectClassification::BUS, "bus"},
  {ObjectClassification::TRAILER, "trailer"},
  {ObjectClassification::MOTORCYCLE, "motorcycle"},
  {ObjectClassification::BICYCLE, "bicycle"},
  {ObjectClassification::PEDESTRIAN, "pedestrian"},
  {ObjectClassification::UNKNOWN, "unknown"}};
// DO NOT accept "animal", "hazard", "over_drivable", "under_drivable" class

constexpr std::string_view to_type_string(const uint8_t label)
{
  for (const auto & [label_val, class_name] : kObjectClassifications) {
    if (label_val == label) {
      return class_name;
    }
  }
  throw std::invalid_argument("Unsupported label: " + std::to_string(label));
}

constexpr std::string_view to_type_string(const ObjectClassification & obj)
{
  return to_type_string(obj.label);
}

inline std::string_view to_type_string(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & obj)
{
  return to_type_string(autoware::object_recognition_utils::getHighestProbLabel(obj));
}

struct GlobalParams
{
  double time_resolution{0.1};

  GlobalParams() = default;
  explicit GlobalParams(const validator::Params::CollisionCheck::GlobalSetting & params)
  {
    time_resolution = params.time_resolution;
  }
};

struct PetThreshold
{
  double ego_first_passing_time_gap{1.0};
  double object_first_passing_time_gap{1.0};
};

using DracParams = validator::Params::CollisionCheck::Drac;
using PetParams = validator::Params::CollisionCheck::PetCollision;
using RssParams = validator::Params::CollisionCheck::Rss;

using DracParamMap = std::map<std::string_view, DracParams>;
using PetParamMap = std::map<std::string_view, PetParams>;
using RssParamMap = std::map<std::string_view, RssParams>;

template <typename ParamMap>
ParamMap make_param_map_from_base(const typename ParamMap::mapped_type & base)
{
  ParamMap param_map;
  for (const auto & [label, class_name] : kObjectClassifications) {
    (void)label;
    param_map[class_name] = base;
  }
  param_map[kCollisionCheckParamBaseKey] = base;
  return param_map;
}

}  // namespace autoware::trajectory_validator::plugin::safety

#endif  // FILTERS__SAFETY__COLLISION_CHECK_FILTER__PARAMETER_HPP_
