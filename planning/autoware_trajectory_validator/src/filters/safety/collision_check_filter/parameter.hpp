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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER__PARAMETER_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER__PARAMETER_HPP_

#include <autoware_trajectory_validator/autoware_trajectory_validator_param.hpp>

#include <array>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>


namespace autoware::trajectory_validator::plugin::safety
{
inline constexpr const char * kCollisionCheckParamBaseKey = "base";
// Keep in sync with `parameter_struct.yaml`.
inline constexpr const char * kCollisionCheckObjectClassKeys[]{
  "car", "truck", "bus", "trailer", "motorcycle", "bicycle", "pedestrian", "unknown",
};
// DO NOT accept "animal", "hazard", "over_drivable", "under_drivable" class

template <typename OutT, typename ParamStruct>
OutT extract_labeled_param(const ParamStruct & params_struct, const std::string & key)
{
  if constexpr (std::is_aggregate_v<ParamStruct>) {
    if (key == kCollisionCheckParamBaseKey) {
      return static_cast<OutT>(params_struct.base);
    }

    using MemberPtr = OutT ParamStruct::*;

    static const std::unordered_map<std::string, MemberPtr> mappings = {
      {"car", &ParamStruct::car},
      {"truck", &ParamStruct::truck},
      {"bus", &ParamStruct::bus},
      {"trailer", &ParamStruct::trailer},
      {"motorcycle", &ParamStruct::motorcycle},
      {"bicycle", &ParamStruct::bicycle},
      {"pedestrian", &ParamStruct::pedestrian},
      {"animal", &ParamStruct::animal},
      {"hazard", &ParamStruct::hazard},
      {"over_drivable", &ParamStruct::over_drivable},
      {"under_drivable", &ParamStruct::under_drivable},
      {"unknown", &ParamStruct::unknown}};

    auto it = mappings.find(key);
    if (it == mappings.end()) {
      throw std::invalid_argument("Unknown label key: " + key);
    }

    auto label_value = params_struct.*(it->second);
    if constexpr (std::is_floating_point_v<OutT>) {
      return static_cast<OutT>(std::isnan(label_value) ? params_struct.base : label_value);
    } else if constexpr (std::is_same_v<OutT, std::string>) {
      return static_cast<OutT>(label_value.empty() ? params_struct.base : label_value);
    } else {
      return static_cast<OutT>(label_value);
    }

  } else {
    return static_cast<OutT>(params_struct);
  }
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

struct DracParams
{
  struct AssessmentTrajectories
  {
    bool map_based{true};
    bool constant_curvature{true};
    bool diffusion_based{true};
  };

  struct Threshold
  {
    double ego_acceleration{-4.0};
  };

  DracParams() = default;
  DracParams(const validator::Params & node_params, const std::string & key)
  {
    const auto & drac = node_params.collision_check.drac;
    enable_assessment = extract_labeled_param<bool>(drac.enable_assessment, key);
    assessment_trajectories.map_based =
      extract_labeled_param<bool>(drac.assessment_trajectories.map_based, key);
    assessment_trajectories.constant_curvature =
      extract_labeled_param<bool>(drac.assessment_trajectories.constant_curvature, key);
    assessment_trajectories.diffusion_based =
      extract_labeled_param<bool>(drac.assessment_trajectories.diffusion_based, key);
    ego_total_braking_delay = extract_labeled_param<double>(drac.ego_total_braking_delay, key);
    warn_threshold.ego_acceleration =
      extract_labeled_param<double>(drac.warn_threshold.ego_acceleration, key);
    error_threshold.ego_acceleration =
      extract_labeled_param<double>(drac.error_threshold.ego_acceleration, key);
  }

  bool enable_assessment{false};
  AssessmentTrajectories assessment_trajectories{};
  double ego_total_braking_delay{0.4};
  Threshold warn_threshold{-2.0};
  Threshold error_threshold{};
};

struct PetCollisionParams
{
  struct AssessmentTrajectories
  {
    bool map_based{true};
    bool constant_curvature{true};
    bool diffusion_based{true};
  };

  struct Threshold
  {
    double ego_first_passing_time_gap{1.0};
    double object_first_passing_time_gap{1.0};
  };

  bool enable_assessment{true};
  AssessmentTrajectories assessment_trajectories{};
  double ego_total_braking_delay{0.4};
  double ego_assumed_acceleration{-5.0};
  Threshold warn_threshold{};
  Threshold error_threshold{0.6, 0.3};

  PetCollisionParams() = default;
  PetCollisionParams(const validator::Params & node_params, const std::string & key)
  {
    const auto & pet = node_params.collision_check.pet_collision;
    enable_assessment = extract_labeled_param<bool>(pet.enable_assessment, key);
    assessment_trajectories.map_based =
      extract_labeled_param<bool>(pet.assessment_trajectories.map_based, key);
    assessment_trajectories.constant_curvature =
      extract_labeled_param<bool>(pet.assessment_trajectories.constant_curvature, key);
    assessment_trajectories.diffusion_based =
      extract_labeled_param<bool>(pet.assessment_trajectories.diffusion_based, key);
    ego_total_braking_delay = extract_labeled_param<double>(pet.ego_total_braking_delay, key);
    ego_assumed_acceleration = extract_labeled_param<double>(pet.ego_assumed_acceleration, key);

    warn_threshold.ego_first_passing_time_gap =
      extract_labeled_param<double>(pet.warn_threshold.ego_first_passing_time_gap, key);
    warn_threshold.object_first_passing_time_gap =
      extract_labeled_param<double>(pet.warn_threshold.object_first_passing_time_gap, key);
    error_threshold.ego_first_passing_time_gap =
      extract_labeled_param<double>(pet.error_threshold.ego_first_passing_time_gap, key);
    error_threshold.object_first_passing_time_gap =
      extract_labeled_param<double>(pet.error_threshold.object_first_passing_time_gap, key);
  }
};

struct RssParams
{
  struct ErrorThreshold
  {
    double ego_acceleration{-4.0};
  };

  RssParams() = default;
  RssParams(const validator::Params & node_params, const std::string & key)
  {
    const auto & rss = node_params.collision_check.rss;
    enable_assessment = extract_labeled_param<bool>(rss.enable_assessment, key);
    stop_distance_margin = extract_labeled_param<double>(rss.stop_distance_margin, key);
    ego_total_braking_delay = extract_labeled_param<double>(rss.ego_total_braking_delay, key);
    object_assumed_acceleration = extract_labeled_param<double>(rss.object_assumed_acceleration, key);
    error_threshold.ego_acceleration =
      extract_labeled_param<double>(rss.error_threshold.ego_acceleration, key);
  }

  bool enable_assessment{true};
  double stop_distance_margin{2.0};
  double ego_total_braking_delay{0.4};
  double object_assumed_acceleration{-1.0};
  ErrorThreshold error_threshold{};
};

// struct CollisionCheckParameters
// {
//   DracParams drac{};
//   PetCollisionParams pet_collision{};
//   RssParams rss{};
// };

template <typename PluginParam>
std::map<std::string, PluginParam> create_param_map_per_object(
  const validator::Params & node_params)
{
  std::map<std::string, PluginParam> param_map;
  for (const char * class_key : kCollisionCheckObjectClassKeys) {
    param_map[class_key] = PluginParam(node_params, class_key);
  }

  param_map[kCollisionCheckParamBaseKey] = PluginParam(node_params, kCollisionCheckParamBaseKey);

  return param_map;
}

using DracParamMap = std::map<std::string, DracParams>;
using PetCollisionParamMap = std::map<std::string, PetCollisionParams>;
using RssParamMap = std::map<std::string, RssParams>;

}  // namespace autoware::trajectory_validator::plugin::safety

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER__PARAMETER_HPP_
