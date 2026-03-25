// Copyright 2021 TIER IV, Inc.
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

#ifndef AUTOWARE__LIDAR_CENTERPOINT__NODE_HPP_
#define AUTOWARE__LIDAR_CENTERPOINT__NODE_HPP_

#include "autoware/lidar_centerpoint/centerpoint_trt.hpp"
#include "autoware/lidar_centerpoint/detection_class_remapper.hpp"
#include "autoware/lidar_centerpoint/postprocess/non_maximum_suppression.hpp"

#include <agnocast/agnocast.hpp>
#include <agnocast/cuda/types.hpp>
#include <agnocast/node/tf2/tf2.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::lidar_centerpoint
{

class LidarCenterPointNode : public agnocast::Node
{
public:
  explicit LidarCenterPointNode(const rclcpp::NodeOptions & node_options);

private:
  void pointCloudCallback(
    agnocast::ipc_shared_ptr<const agnocast::cuda::PointCloud2> input_pointcloud_msg);

  agnocast::Buffer tf_buffer_;
  std::unique_ptr<agnocast::TransformListener> tf_listener_;

  agnocast::Subscription<agnocast::cuda::PointCloud2>::SharedPtr pointcloud_sub_;
  agnocast::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;

  std::vector<std::string> class_names_;
  bool has_variance_{false};
  bool has_twist_{false};

  // for diagnostics (diagnostic_updater::Updater is incompatible with agnocast::Node)
  // double max_allowed_processing_time_ms_;
  // double max_acceptable_consecutive_delay_ms_;
  // std::optional<double> last_processing_time_ms_;
  // std::optional<rclcpp::Time> last_in_time_processing_timestamp_;

  NonMaximumSuppression iou_bev_nms_;
  DetectionClassRemapper detection_class_remapper_;

  std::unique_ptr<CenterPointTRT> detector_ptr_{nullptr};
  std::unique_ptr<autoware_utils::BasicDiagnosticsInterface<agnocast::Node>>
    diagnostics_centerpoint_trt_;

  // debugger
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{nullptr};
  std::unique_ptr<autoware_utils::BasicDebugPublisher<agnocast::Node>> debug_publisher_ptr_{
    nullptr};

  std::unique_ptr<autoware_utils::BasicPublishedTimePublisher<agnocast::Node>>
    published_time_publisher_;
  std::string logger_name_{"lidar_centerpoint"};
};

}  // namespace autoware::lidar_centerpoint

#endif  // AUTOWARE__LIDAR_CENTERPOINT__NODE_HPP_
