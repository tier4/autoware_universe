// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__OBJECT_MERGER__OBJECT_ASSOCIATION_MERGER_NODE_HPP_
#define AUTOWARE__OBJECT_MERGER__OBJECT_ASSOCIATION_MERGER_NODE_HPP_

#include "autoware/object_merger/association/data_association.hpp"
#include "autoware_utils/ros/debug_publisher.hpp"
#include "autoware_utils/ros/published_time_publisher.hpp"
#include "autoware_utils/system/stop_watch.hpp"

#include <agnocast/agnocast_timer.hpp>
#include <agnocast/message_filters/subscriber.hpp>
#include <agnocast/message_filters/sync_policies/approximate_time.hpp>
#include <agnocast/message_filters/synchronizer.hpp>
#include <agnocast/node/agnocast_node.hpp>
#include <agnocast/node/tf2/buffer.hpp>
#include <agnocast/node/tf2/transform_listener.hpp>
#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/detected_objects.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::object_merger
{
class ObjectAssociationMergerNode : public agnocast::Node
{
  using Label = autoware_perception_msgs::msg::ObjectClassification;

public:
  explicit ObjectAssociationMergerNode(const rclcpp::NodeOptions & node_options);
  enum class PriorityMode : int { Object0 = 0, Object1 = 1, Confidence = 2, ClassBased = 3 };

private:
  void objectsCallback(
    const agnocast::ipc_shared_ptr<autoware_perception_msgs::msg::DetectedObjects const> &
      input_objects0_msg,
    const agnocast::ipc_shared_ptr<autoware_perception_msgs::msg::DetectedObjects const> &
      input_objects1_msg);

  void diagCallback();

  agnocast::Buffer tf_buffer_;
  std::unique_ptr<agnocast::TransformListener> tf_listener_;
  agnocast::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr merged_object_pub_;
  agnocast::message_filters::Subscriber<autoware_perception_msgs::msg::DetectedObjects,
    agnocast::Node> object0_sub_{};
  agnocast::message_filters::Subscriber<autoware_perception_msgs::msg::DetectedObjects,
    agnocast::Node> object1_sub_{};

  using SyncPolicy = agnocast::message_filters::sync_policies::ApproximateTime<
    autoware_perception_msgs::msg::DetectedObjects, autoware_perception_msgs::msg::DetectedObjects>;
  using Sync = agnocast::message_filters::Synchronizer<SyncPolicy>;
  typename std::shared_ptr<Sync> sync_ptr_;

  int sync_queue_size_;
  std::unique_ptr<DataAssociation> data_association_;
  std::string base_link_frame_id_;  // associated with the base_link frame

  // Timeout Related
  double message_timeout_sec_;
  double initialization_timeout_sec_;
  std::optional<rclcpp::Time> last_sync_time_;
  std::optional<double> message_interval_;
  agnocast::WallTimer<std::function<void()>>::SharedPtr timeout_timer_;
  std::unique_ptr<autoware_utils::BasicDiagnosticsInterface<agnocast::Node>>
    diagnostics_interface_ptr_;

  PriorityMode priority_mode_;
  std::vector<int64_t> class_based_priority_matrix_;

  int NUMBER_OF_CLASSES_;

  bool remove_overlapped_unknown_objects_;
  struct
  {
    double precision_threshold;
    double recall_threshold;
    std::map<int, double> generalized_iou_threshold;
    std::map<int /*class label*/, double /*distance_threshold*/> distance_threshold_map;
  } overlapped_judge_param_;

  // debug publisher
  std::unique_ptr<autoware_utils_debug::BasicDebugPublisher<agnocast::Node>>
    processing_time_publisher_;
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;

  std::unique_ptr<autoware_utils::BasicPublishedTimePublisher<agnocast::Node>>
    published_time_publisher_;
};
}  // namespace autoware::object_merger

#endif  // AUTOWARE__OBJECT_MERGER__OBJECT_ASSOCIATION_MERGER_NODE_HPP_
