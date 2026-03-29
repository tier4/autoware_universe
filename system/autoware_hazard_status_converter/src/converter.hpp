// Copyright 2023 The Autoware Contributors
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

#ifndef CONVERTER_HPP_
#define CONVERTER_HPP_

#include "autoware/diagnostic_graph_utils/graph.hpp"

#include <agnocast/agnocast.hpp>

#include <autoware_system_msgs/msg/hazard_status_stamped.hpp>
#include <tier4_system_msgs/msg/diag_graph_status.hpp>
#include <tier4_system_msgs/msg/diag_graph_struct.hpp>
#include <tier4_system_msgs/msg/emergency_holding_state.hpp>

#include <unordered_set>

namespace autoware::hazard_status_converter
{

class Converter : public agnocast::Node
{
public:
  explicit Converter(const rclcpp::NodeOptions & options);

private:
  using HazardStatusStamped = autoware_system_msgs::msg::HazardStatusStamped;
  using DiagGraph = autoware::diagnostic_graph_utils::DiagGraph;
  using DiagUnit = autoware::diagnostic_graph_utils::DiagUnit;
  using DiagNode = autoware::diagnostic_graph_utils::DiagNode;
  void on_create(DiagGraph::SharedPtr graph);
  void on_update(DiagGraph::SharedPtr graph);
  void on_struct(
    const agnocast::ipc_shared_ptr<const tier4_system_msgs::msg::DiagGraphStruct> & msg);
  void on_status(
    const agnocast::ipc_shared_ptr<const tier4_system_msgs::msg::DiagGraphStatus> & msg);
  agnocast::Subscription<tier4_system_msgs::msg::DiagGraphStruct>::SharedPtr sub_struct_;
  agnocast::Subscription<tier4_system_msgs::msg::DiagGraphStatus>::SharedPtr sub_status_;
  DiagGraph::SharedPtr graph_;
  agnocast::Publisher<HazardStatusStamped>::SharedPtr pub_hazard_;
  agnocast::PollingSubscriber<tier4_system_msgs::msg::EmergencyHoldingState>::SharedPtr
    sub_emergency_holding_;

  DiagNode * auto_mode_root_;
  std::unordered_set<DiagUnit *> auto_mode_tree_;
};

}  // namespace autoware::hazard_status_converter

#endif  // CONVERTER_HPP_
