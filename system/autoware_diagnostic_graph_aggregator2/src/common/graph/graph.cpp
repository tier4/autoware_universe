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

//
#include <iostream>

namespace autoware::diagnostic_graph_aggregator
{

Graph::Graph(const std::string & path, const Logger & logger)
{
  const auto graph = load_config(path, logger);
  (void)graph;
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
}

}  // namespace autoware::diagnostic_graph_aggregator
