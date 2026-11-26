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

#ifndef AUTOWARE__ML_PLANNER__UTILS__OBJECT_REMAP_HPP_
#define AUTOWARE__ML_PLANNER__UTILS__OBJECT_REMAP_HPP_

#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace autoware::ml_planner::utils
{
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;

/// @brief True for a label the model has no class for but that can still obstruct driving.
///
/// The preprocessing maps CAR, TRUCK, BUS, MOTORCYCLE, TRAILER, BICYCLE and PEDESTRIAN to model
/// classes and ignores everything else, so UNKNOWN, HAZARD and any class added to
/// ObjectClassification later are dropped before the planner sees them. ANIMAL, OVER_DRIVABLE and
/// UNDER_DRIVABLE are deliberate exceptions: not obstacles, or not drivable-space hazards worth
/// braking for. Listing the exceptions rather than the unsupported classes keeps a future class
/// fail-safe: remapped, not silently ignored.
bool is_unsupported_obstacle_label(uint8_t label);

/// @brief Rewrite one unsupported object to PEDESTRIAN, the most conservative supported class.
///
/// Objects of a supported class, and objects with an empty classification (no label to rewrite),
/// are returned unchanged. A non-BOX shape is replaced with a 0.5 m bounding box: perception
/// reports these classes as polygons, the preprocessing drops POLYGON objects, and the model reads
/// `shape.dimensions.x/y` as length and width, which a polygon does not fill in.
TrackedObject remap_unsupported_to_pedestrian(const TrackedObject & object);

/// @brief Apply remap_unsupported_to_pedestrian() to every object of a message.
TrackedObjects remap_unsupported_objects_to_pedestrian(const TrackedObjects & objects);

/// @brief Apply the remap to a batch of messages taken from a polling subscriber.
///
/// A message with nothing to rewrite is passed through by pointer, so the common case copies
/// nothing. Null pointers are preserved, because the caller skips them.
std::vector<std::shared_ptr<const TrackedObjects>> remap_unsupported_objects_to_pedestrian(
  const std::vector<std::shared_ptr<const TrackedObjects>> & messages);

}  // namespace autoware::ml_planner::utils

#endif  // AUTOWARE__ML_PLANNER__UTILS__OBJECT_REMAP_HPP_
