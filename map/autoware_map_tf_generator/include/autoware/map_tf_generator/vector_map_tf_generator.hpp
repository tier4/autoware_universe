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

#ifndef AUTOWARE__MAP_TF_GENERATOR__VECTOR_MAP_TF_GENERATOR_HPP_
#define AUTOWARE__MAP_TF_GENERATOR__VECTOR_MAP_TF_GENERATOR_HPP_

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/time.hpp>

#include <string>

namespace autoware::map_tf_generator
{

geometry_msgs::msg::TransformStamped create_viewer_transform(
  const autoware_map_msgs::msg::LaneletMapBin & map_bin_msg, const std::string & map_frame,
  const std::string & viewer_frame, const rclcpp::Time & stamp);

}  // namespace autoware::map_tf_generator

#endif  // AUTOWARE__MAP_TF_GENERATOR__VECTOR_MAP_TF_GENERATOR_HPP_
