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

#ifndef LOW_INTENSITY_CLUSTER_FILTER_NODE_HPP_
#define LOW_INTENSITY_CLUSTER_FILTER_NODE_HPP_

#include "autoware/detected_object_validation/utils/utils.hpp"
#include "autoware_utils/system/stop_watch.hpp"

#include <Eigen/Eigen>
#include <agnocast/agnocast.hpp>
#include <agnocast/node/tf2/buffer.hpp>
#include <agnocast/node/tf2/transform_listener.hpp>
#include <autoware_utils_debug/debug_publisher.hpp>

#include "tier4_perception_msgs/msg/detected_objects_with_feature.hpp"

#include <memory>
#include <string>

namespace autoware::low_intensity_cluster_filter
{

class LowIntensityClusterFilter : public agnocast::Node
{
public:
  explicit LowIntensityClusterFilter(const rclcpp::NodeOptions & node_options);

private:
  void objectCallback(
    agnocast::ipc_shared_ptr<tier4_perception_msgs::msg::DetectedObjectsWithFeature> && input_msg);
  bool isValidatedCluster(const sensor_msgs::msg::PointCloud2 & cluster);

  agnocast::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr
    object_pub_;
  agnocast::Subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr
    object_sub_;

  agnocast::Buffer tf_buffer_;
  std::unique_ptr<agnocast::TransformListener> tf_listener_;
  double intensity_threshold_;
  double existence_probability_threshold_;
  double max_x_;
  double min_x_;
  double max_y_;
  double min_y_;

  // Eigen::Vector4f min_boundary_transformed_;
  // Eigen::Vector4f max_boundary_transformed_;
  const std::string base_link_frame_id_ = "base_link";
  autoware::detected_object_validation::utils::FilterTargetLabel filter_target_;

  // debugger
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{nullptr};
  std::unique_ptr<autoware_utils_debug::BasicDebugPublisher<agnocast::Node>> debug_publisher_ptr_{
    nullptr};
};

}  // namespace autoware::low_intensity_cluster_filter

#endif  // LOW_INTENSITY_CLUSTER_FILTER_NODE_HPP_
