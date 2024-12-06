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

// create a node to subscribe traffic yolox/rois and traffic expect/rois to match the traffic light
// id and publish the matched traffic light id

#ifndef TRAFFIC_LIGHT_ID_MATCHER_NODE_HPP_
#define TRAFFIC_LIGHT_ID_MATCHER_NODE_HPP_

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/time_synchronizer.h"
#include "tf2_ros/buffer.h"
#include "autoware/universe_utils/ros/transform_listener.hpp"

#include <rclcpp/rclcpp.hpp>

#include "tier4_perception_msgs/msg/detected_object_with_feature.hpp"
#include "tier4_perception_msgs/msg/detected_objects_with_feature.hpp"
#include "tier4_perception_msgs/msg/traffic_light_roi_array.hpp"

namespace autoware::traffic_light
{

using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;
using tier4_perception_msgs::msg::TrafficLightRoi;
using tier4_perception_msgs::msg::TrafficLightRoiArray;

class AutowareTrafficLightIdMatcherNode : public rclcpp::Node
{
public:
  AutowareTrafficLightIdMatcherNode(const rclcpp::NodeOptions & options);

private:

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  void callback(
    const DetectedObjectsWithFeature::SharedPtr rois,
    const TrafficLightRoiArray::SharedPtr expect_rois);
  // Syncronized subscriber
  message_filters::Subscriber<DetectedObjectsWithFeature> rois_sub_;
  message_filters::Subscriber<TrafficLightRoiArray> expect_rois_sub_;
  typedef message_filters::sync_policies::ApproximateTime<
    DetectedObjectsWithFeature, TrafficLightRoiArray>
    ApproximateSyncPolicy;

  typedef message_filters::Synchronizer<ApproximateSyncPolicy> ApproximateSync;
  ApproximateSync approximate_sync_;
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_ID_MATCHER_NODE_HPP_
