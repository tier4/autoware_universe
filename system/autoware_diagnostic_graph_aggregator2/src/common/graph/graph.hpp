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

#ifndef COMMON__GRAPH__GRAPH_HPP_
#define COMMON__GRAPH__GRAPH_HPP_

#include "types/forward.hpp"
#include "utils/logger.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

class Graph
{
public:
  Graph(const std::string & path, const Logger & logger);
  ~Graph();

  void dump() const;

private:
  std::vector<std::unique_ptr<NodeUnit>> nodes_;
  std::vector<std::unique_ptr<DiagUnit>> diags_;
  std::vector<std::unique_ptr<UnitLink>> links_;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__GRAPH_HPP_
