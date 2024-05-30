// Copyright 2024 TIER IV, Inc.
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

#include "filter_predicted_objects.hpp"

#include <motion_utils/trajectory/trajectory.hpp>
#include <traffic_light_utils/traffic_light_utils.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>

namespace autoware::motion_velocity_planner::out_of_lane
{
autoware_auto_perception_msgs::msg::PredictedObjects filter_predicted_objects(
  const std::shared_ptr<const PlannerData> planner_data, const EgoData & ego_data,
  const PlannerParam & params)
{
  autoware_auto_perception_msgs::msg::PredictedObjects filtered_objects;
  filtered_objects.header = planner_data->predicted_objects->header;
  for (const auto & object : planner_data->predicted_objects->objects) {
    const auto is_pedestrian =
      std::find_if(object.classification.begin(), object.classification.end(), [](const auto & c) {
        return c.label == autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
      }) != object.classification.end();
    if (is_pedestrian) continue;

    auto filtered_object = object;
    const auto is_invalid_predicted_path = [&](const auto & predicted_path) {
      const auto is_low_confidence = predicted_path.confidence < params.objects_min_confidence;
      const auto no_overlap_path = motion_utils::removeOverlapPoints(predicted_path.path);
      if (no_overlap_path.size() <= 1) return true;
      const auto lat_offset_to_current_ego =
        std::abs(motion_utils::calcLateralOffset(no_overlap_path, ego_data.pose.position));
      const auto is_crossing_ego =
        lat_offset_to_current_ego <=
        object.shape.dimensions.y / 2.0 + std::max(
                                            params.left_offset + params.extra_left_offset,
                                            params.right_offset + params.extra_right_offset);
      return is_low_confidence || is_crossing_ego;
    };
    if (params.objects_use_predicted_paths) {
      auto & predicted_paths = filtered_object.kinematics.predicted_paths;
      const auto new_end =
        std::remove_if(predicted_paths.begin(), predicted_paths.end(), is_invalid_predicted_path);
      predicted_paths.erase(new_end, predicted_paths.end());
      predicted_paths.erase(
        std::remove_if(
          predicted_paths.begin(), predicted_paths.end(),
          [](const auto & p) { return p.path.empty(); }),
        predicted_paths.end());
    }

    if (!params.objects_use_predicted_paths || !filtered_object.kinematics.predicted_paths.empty())
      filtered_objects.objects.push_back(filtered_object);
  }
  return filtered_objects;
}

}  // namespace autoware::motion_velocity_planner::out_of_lane
