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

#include "traffic_light_id_matcher_node.hpp"
namespace autoware::traffic_light
{
AutowareTrafficLightIdMatcherNode::AutowareTrafficLightIdMatcherNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("traffic_light_id_matcher_node", options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  rois_sub_(this, "input/rois", rclcpp::QoS{1}.get_rmw_qos_profile()),
  expect_rois_sub_(this, "input/expect_rois", rclcpp::QoS{1}.get_rmw_qos_profile()),
  approximate_sync_(ApproximateSyncPolicy(10), rois_sub_, expect_rois_sub_)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  approximate_sync_.registerCallback(
    std::bind(&AutowareTrafficLightIdMatcherNode::callback, this, _1, _2));
}

void AutowareTrafficLightIdMatcherNode::callback(
  const DetectedObjectsWithFeature::SharedPtr rois_msg,
  const TrafficLightRoiArray::SharedPtr roughs_msg)
{
  (void)rois_msg;
  (void)roughs_msg;

  // // match the rois with expect_rois by calculating max_score between 2 rois
  // // and publish the matched traffic light id
  // TrafficLightRoiArray matched_rois;
  // for(const auto & roi : rois_msg->feature_objects->feature.rois){
  //   for(const auto & rough_roi : roughs_msg->rois){
  //     if (isInsideRough(roi, rough_roi)){
  //       TrafficLightRoi matched_roi;
  //       matched_roi.roi = roi;
  //       matched_roi.traffic_light_id = rough_roi.traffic_light_id;
  //       matched_rois.rois.push_back(matched_roi);
  //     }
  //   }
  // }
  // matched_rois.header = rois_msg->header;
  // output_pub_->publish(matched_rois);
}


}  // namespace autoware::traffic_light

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::AutowareTrafficLightIdMatcherNode)
