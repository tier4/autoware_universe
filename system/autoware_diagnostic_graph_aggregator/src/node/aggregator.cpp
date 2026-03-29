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

#include "aggregator.hpp"

#include <memory>
#include <sstream>
#include <string>

namespace autoware::diagnostic_graph_aggregator
{

AggregatorNode::AggregatorNode(const rclcpp::NodeOptions & options)
: agnocast::Node("aggregator", options)
{
  const auto stamp = now();

  // Init diagnostics graph.
  {
    const auto graph_file = declare_parameter<std::string>("graph_file");
    std::ostringstream id;
    id << std::hex << stamp.nanoseconds();
    graph_ = std::make_unique<Graph>(graph_file, id.str(), nullptr);
  }

  // Init plugins.
  if (declare_parameter<bool>("use_command_mode_mappings")) {
    availability_ = std::make_unique<CommandModeMapping>(*this, *graph_);
  }

  // Init ros interface.
  {
    const auto qos_input = rclcpp::QoS(declare_parameter<int64_t>("input_qos_depth"));
    const auto qos_unknown = rclcpp::QoS(1);
    const auto qos_struct = rclcpp::QoS(1).transient_local();
    const auto qos_status = rclcpp::QoS(declare_parameter<int64_t>("graph_qos_depth"));
    const auto callback = std::bind(&AggregatorNode::on_diag, this, std::placeholders::_1);
    sub_input_ = create_subscription<DiagnosticArray>("/diagnostics", qos_input, callback);
    pub_struct_ = create_publisher<DiagGraphStruct>("~/struct", qos_struct);
    pub_status_ = create_publisher<DiagGraphStatus>("~/status", qos_status);
    pub_unknown_ = create_publisher<DiagnosticArray>("~/unknowns", qos_unknown);
    srv_reset_ = create_service<ResetDiagGraph>(
      "~/reset",
      [this](
        const agnocast::ipc_shared_ptr<agnocast::Service<ResetDiagGraph>::RequestT> & req,
        agnocast::ipc_shared_ptr<agnocast::Service<ResetDiagGraph>::ResponseT> & res) {
        on_reset(req, res);
      });
    srv_set_initializing_ = create_service<SetBool>(
      "~/set_initializing",
      [this](
        const agnocast::ipc_shared_ptr<agnocast::Service<SetBool>::RequestT> & req,
        agnocast::ipc_shared_ptr<agnocast::Service<SetBool>::ResponseT> & res) {
        on_set_initializing(req, res);
      });

    const auto rate = rclcpp::Rate(declare_parameter<double>("rate"));
    timer_ = create_timer(rate.period(), [this]() { on_timer(); });
  }

  // Send structure topic once.
  {
    auto struct_msg = pub_struct_->borrow_loaned_message();
    *struct_msg = graph_->create_struct_msg(stamp);
    pub_struct_->publish(std::move(struct_msg));
  }
}

AggregatorNode::~AggregatorNode()
{
  // For unique_ptr members.
}

void AggregatorNode::on_timer()
{
  // Check timeout of diag units.
  const auto stamp = now();
  graph_->update(stamp);

  // Publish status.
  {
    auto status_msg = pub_status_->borrow_loaned_message();
    *status_msg = graph_->create_status_msg(stamp);
    pub_status_->publish(std::move(status_msg));
  }
  {
    auto unknown_msg = pub_unknown_->borrow_loaned_message();
    *unknown_msg = graph_->create_unknown_msg(stamp);
    pub_unknown_->publish(std::move(unknown_msg));
  }

  // Update plugins.
  if (availability_) availability_->update(stamp);
}

void AggregatorNode::on_diag(const agnocast::ipc_shared_ptr<const DiagnosticArray> & msg)
{
  graph_->update(now(), *msg);
}

void AggregatorNode::on_reset(
  const agnocast::ipc_shared_ptr<agnocast::Service<ResetDiagGraph>::RequestT> &,
  agnocast::ipc_shared_ptr<agnocast::Service<ResetDiagGraph>::ResponseT> & response)
{
  graph_->reset();
  response->status.success = true;
}

void AggregatorNode::on_set_initializing(
  const agnocast::ipc_shared_ptr<agnocast::Service<SetBool>::RequestT> & request,
  agnocast::ipc_shared_ptr<agnocast::Service<SetBool>::ResponseT> & response)
{
  graph_->set_initializing(request->data);
  response->success = true;
}

}  // namespace autoware::diagnostic_graph_aggregator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::diagnostic_graph_aggregator::AggregatorNode)
