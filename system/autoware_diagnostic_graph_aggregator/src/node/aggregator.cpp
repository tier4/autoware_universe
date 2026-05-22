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

#include <yaml-cpp/yaml.h>

#include <memory>
#include <optional>
#include <sstream>
#include <set>
#include <string>
#include <unordered_map>

namespace autoware::diagnostic_graph_aggregator
{

AggregatorNode::AggregatorNode(const rclcpp::NodeOptions & options) : Node("aggregator", options)
{
  const auto stamp = now();

  // Init diagnostics graph.
  {
    const auto graph_file = declare_parameter<std::string>("graph_file");

    // Parse graph variables from YAML map format (e.g., "{key1: value1, key2: value2}").
    auto variables = std::make_shared<std::unordered_map<std::string, std::string>>();
    const auto vars_text = declare_parameter<std::string>("graph_vars", "");
    if (!vars_text.empty()) {
      const auto vars_yaml = YAML::Load(vars_text);
      for (const auto & var : vars_yaml) {
        variables->emplace(var.first.as<std::string>(), var.second.as<std::string>());
      }
    }

    std::ostringstream id;
    id << std::hex << stamp.nanoseconds();
    graph_ = std::make_unique<Graph>(graph_file, id.str(), nullptr, variables);
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
    sub_input_ = create_subscription<DiagnosticArray>("~/diagnostics", qos_input, callback);
    pub_struct_ = create_publisher<DiagGraphStruct>("~/struct", qos_struct);
    pub_status_ = create_publisher<DiagGraphStatus>("~/status", qos_status);
    pub_unknown_ = create_publisher<DiagnosticArray>("~/unknowns", qos_unknown);
    srv_reset_ = create_service<ResetDiagGraph>(
      "~/reset",
      std::bind(&AggregatorNode::on_reset, this, std::placeholders::_1, std::placeholders::_2));
    srv_set_initializing_ = create_service<SetBool>(
      "~/set_initializing",
      std::bind(
        &AggregatorNode::on_set_initializing, this, std::placeholders::_1, std::placeholders::_2));

    const auto rate = rclcpp::Rate(declare_parameter<double>("rate"));
    timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() { on_timer(); });
  }

  // Send structure topic once.
  pub_struct_->publish(graph_->create_struct_msg(stamp));
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
  pub_status_->publish(graph_->create_status_msg(stamp));
  pub_unknown_->publish(graph_->create_unknown_msg(stamp));

  // Update plugins.
  if (availability_) availability_->update(stamp);
}

void AggregatorNode::on_diag(const DiagnosticArray & msg)
{
  check_sequence_gaps(msg);
  graph_->update(now(), msg);
}

std::optional<uint64_t> AggregatorNode::find_sequence_id(const DiagnosticStatus & status)
{
  const auto iter = std::find_if(status.values.begin(), status.values.end(), [](const auto & kv) {
    return kv.key == "sequence_id";
  });
  if (iter == status.values.end()) {
    return std::nullopt;
  }

  try {
    return std::stoull(iter->value);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

void AggregatorNode::check_sequence_gaps(const DiagnosticArray & msg)
{
  std::unordered_map<std::string, std::set<uint64_t>> sequences_by_name;

  for (const auto & status : msg.status) {
    const auto sequence_id = find_sequence_id(status);
    if (!sequence_id || status.name.empty()) {
      continue;
    }
    sequences_by_name[status.name].insert(*sequence_id);
  }

  for (const auto & [name, sequence_ids] : sequences_by_name) {
    if (sequence_ids.size() > 1) {
      std::ostringstream oss;
      bool first = true;
      for (const auto sequence_id : sequence_ids) {
        if (!first) {
          oss << ", ";
        }
        oss << sequence_id;
        first = false;
      }
      RCLCPP_WARN(
        get_logger(),
        "diagnostics with name '%s' contain inconsistent sequence_id values in one "
        "DiagnosticArray: [%s]",
        name.c_str(), oss.str().c_str());
    }

    const auto current_sequence_id = *sequence_ids.begin();
    const auto iter = last_sequence_by_name_.find(name);
    if (iter != last_sequence_by_name_.end()) {
      const auto last_sequence_id = iter->second;
      if (current_sequence_id > last_sequence_id + 1) {
        RCLCPP_ERROR(
          get_logger(),
          "diagnostics sequence_id gap detected for name '%s': last=%lu current=%lu missing=[%lu,%lu]",
          name.c_str(), last_sequence_id, current_sequence_id, last_sequence_id + 1,
          current_sequence_id - 1);
      } else if (current_sequence_id <= last_sequence_id) {
        RCLCPP_DEBUG(
          get_logger(),
          "diagnostics sequence_id went backward or duplicated for name '%s': last=%lu current=%lu",
          name.c_str(), last_sequence_id, current_sequence_id);
      }
    }
    last_sequence_by_name_[name] = current_sequence_id;
  }
}

void AggregatorNode::on_reset(
  const ResetDiagGraph::Request::SharedPtr, const ResetDiagGraph::Response::SharedPtr response)
{
  graph_->reset();
  response->status.success = true;
}

void AggregatorNode::on_set_initializing(
  const SetBool::Request::SharedPtr request, const SetBool::Response::SharedPtr response)
{
  graph_->set_initializing(request->data);
  response->success = true;
}

}  // namespace autoware::diagnostic_graph_aggregator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::diagnostic_graph_aggregator::AggregatorNode)
