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

#include "graph/graph.hpp"

#include "config/entity.hpp"
#include "config/loader.hpp"
#include "graph/links.hpp"
#include "graph/units.hpp"

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

//
#include <iostream>

namespace autoware::diagnostic_graph_aggregator
{

Graph::Graph(const std::string & path, const std::string & id, const Logger & logger)
{
  id_ = id;

  {
    const auto graph = load_config(path, logger);
    std::unordered_map<LinkConfig, UnitConfig> parent_unit;
    std::unordered_map<LinkConfig, UnitConfig> child_unit;
    std::unordered_map<UnitConfig, std::vector<UnitLink *>> parent_links;
    std::unordered_map<UnitConfig, std::vector<UnitLink *>> child_links;
    std::unordered_map<UnitConfig, BaseUnit *> units;

    for (const auto & parent : graph.units) {
      for (const auto & [link, child] : parent->links) {
        parent_unit[link] = parent;
        child_unit[link] = child;
        parent_links[child].push_back(link->link.get());
        child_links[parent].push_back(link->link.get());
      }
      if (parent->diag) {
        const auto & [link, child] = parent->diag.value();
        parent_unit[link] = parent;
        child_unit[link] = child;
        parent_links[child].push_back(link->link.get());
        child_links[parent].push_back(link->link.get());
      }
    }

    for (const auto & node : graph.units) {
      const auto parents = parent_links[node];  // Root units have no parents.
      const auto children = child_links[node];  // Leaf units have no children.
      nodes_.push_back(std::make_unique<NodeUnit>(parents, children, nodes_.size(), node));
      units[node] = nodes_.back().get();
    }
    for (const auto & diag : graph.diags) {
      const auto parents = parent_links.at(diag);
      diags_.push_back(std::make_unique<DiagUnit>(parents, diag->data));
      units[diag] = diags_.back().get();
    }

    for (const auto & link : graph.links) {
      const auto parent = units.at(parent_unit.at(link));
      const auto child = units.at(child_unit.at(link));
      link->link->init(parent, child);
      links_.push_back(std::move(link->link));
    }
  }

  for (const auto & diag : diags_) {
    diag_dict_[diag->name()] = diag.get();
  }
}

Graph::~Graph()
{
}

void Graph::dump() const
{
  // NOLINTBEGIN(build/namespaces, whitespace/line_length)
  // clang-format off
  std::cout << "| Unit | Address        | Level | Input | Latch | Type  | Details                                            |" << std::endl;
  std::cout << "|------------------------------------------------------------------------------------------------------------|" << std::endl;
  for (const auto & node : nodes_) node->dump();
  for (const auto & diag : diags_) diag->dump();
  std::cout << "|============================================================================================================|" << std::endl;
  // clang-format on
  // NOLINTEND
}

void Graph::update(const rclcpp::Time & stamp)
{
  // Update the graph from the leaves. Note that the nodes are topological sorted.
  std::for_each(diags_.rbegin(), diags_.rend(), [stamp](auto & diag) { diag->update(stamp); });
  std::for_each(nodes_.rbegin(), nodes_.rend(), [stamp](auto & node) { node->update(stamp); });
}

bool Graph::update(const rclcpp::Time & stamp, const DiagnosticArray & array)
{
  for (const auto & status : array.status) {
    const auto iter = diag_dict_.find(status.name);
    if (iter != diag_dict_.end()) {
      iter->second->update(stamp, array.header.stamp, status);
    } else {
      unknown_diags_[status.name] = status;
    }
  }
  return true;
}

DiagGraphStruct Graph::create_struct_msg(const rclcpp::Time & stamp) const
{
  DiagGraphStruct msg;
  msg.stamp = stamp;
  msg.id = id_;
  for (const auto & node : nodes_) msg.nodes.push_back(node->create_struct());
  for (const auto & diag : diags_) msg.diags.push_back(diag->create_struct());
  for (const auto & link : links_) {
    if (!link->is_diag()) {
      msg.links.push_back(link->create_struct());
    }
  }
  return msg;
}

DiagGraphStatus Graph::create_status_msg(const rclcpp::Time & stamp) const
{
  DiagGraphStatus msg;
  msg.stamp = stamp;
  msg.id = id_;
  return msg;
}

DiagnosticArray Graph::create_unknown_msg(const rclcpp::Time & stamp) const
{
  DiagnosticArray msg;
  msg.header.stamp = stamp;
  for (const auto & [name, diag] : unknown_diags_) msg.status.push_back(diag);
  return msg;
}

void Graph::reset()
{
  for (const auto & node : nodes_) node->reset();
}

}  // namespace autoware::diagnostic_graph_aggregator
