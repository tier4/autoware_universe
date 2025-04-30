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

Graph::Graph(const std::string & path, const Logger & logger)
{
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
      nodes_.push_back(std::make_unique<NodeUnit>(parents, children, std::move(node->logic)));
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

  /*
  for (const auto & unit : graph.units) {
    std::cout << unit->type << " " << unit->path << std::endl;
    for (const auto & [link, child] : unit->links) {
      std::cout << " - " << link.get() << " -> " << child.get() << std::endl;
    }
  }
  */
}

Graph::~Graph()
{
}

void Graph::dump() const
{
  std::cout << "Graph" << std::endl;

  for (const auto & node : nodes_) {
    std::cout << "Node[" << node.get() << "]: " << node->type() << " (" << node->path() << ")"
              << std::endl;
  }
  for (const auto & diag : diags_) {
    std::cout << "Diag[" << diag.get() << "]: " << diag->name() << std::endl;
  }
}

}  // namespace autoware::diagnostic_graph_aggregator
