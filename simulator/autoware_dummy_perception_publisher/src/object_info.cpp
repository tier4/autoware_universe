// Copyright 2025 Tier IV, Inc.
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

#include "autoware/dummy_perception_publisher/object_info.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace autoware::dummy_perception_publisher
{
using autoware_perception_msgs::msg::TrackedObject;
using tier4_simulation_msgs::msg::DummyObject;

ObjectInfo ObjectInfo::fromDummyObject(const DummyObject & object)
{
  ObjectInfo object_info;
  object_info.length = object.shape.dimensions.x;
  object_info.width = object.shape.dimensions.y;
  object_info.height = object.shape.dimensions.z;
  object_info.std_dev_x = std::sqrt(object.initial_state.pose_covariance.covariance[0]);
  object_info.std_dev_y = std::sqrt(object.initial_state.pose_covariance.covariance[7]);
  object_info.std_dev_z = std::sqrt(object.initial_state.pose_covariance.covariance[14]);
  object_info.std_dev_yaw = std::sqrt(object.initial_state.pose_covariance.covariance[35]);
  object_info.pose_covariance_ = object.initial_state.pose_covariance;
  object_info.twist_covariance_ = object.initial_state.twist_covariance;
  tf2::fromMsg(object.initial_state.pose_covariance.pose, object_info.tf_map2moved_object);
  return object_info;
}

// Implementation of toTrackedObject method
TrackedObject ObjectInfo::toTrackedObject(const DummyObject & object) const
{
  TrackedObject tracked_object;
  tracked_object.kinematics.pose_with_covariance = pose_covariance_;
  tracked_object.kinematics.twist_with_covariance = twist_covariance_;
  tracked_object.classification.push_back(object.classification);
  tracked_object.shape.type = object.shape.type;
  tracked_object.shape.dimensions.x = length;
  tracked_object.shape.dimensions.y = width;
  tracked_object.shape.dimensions.z = height;
  tracked_object.object_id = object.id;
  tracked_object.kinematics.orientation_availability =
    autoware_perception_msgs::msg::TrackedObjectKinematics::SIGN_UNKNOWN;
  return tracked_object;
}

}  // namespace autoware::dummy_perception_publisher
