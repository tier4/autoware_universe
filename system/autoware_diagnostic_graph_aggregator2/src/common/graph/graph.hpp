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

#include "types/diags.hpp"
#include "types/forward.hpp"
#include "utils/logger.hpp"

#include <rclcpp/time.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

class Graph
{
public:
  Graph(const std::string & path, const std::string & id, const Logger & logger);
  ~Graph();
  void dump() const;
  void update(const rclcpp::Time & stamp);
  bool update(const rclcpp::Time & stamp, const DiagnosticArray & array);
  DiagGraphStruct create_struct(const rclcpp::Time & stamp) const;
  DiagGraphStatus create_status(const rclcpp::Time & stamp) const;
  DiagnosticArray create_unknowns(const rclcpp::Time & stamp) const;

private:
  std::string id_;
  std::vector<std::unique_ptr<NodeUnit>> nodes_;
  std::vector<std::unique_ptr<DiagUnit>> diags_;
  std::vector<std::unique_ptr<UnitLink>> links_;

  std::unordered_map<std::string, DiagUnit *> diag_dict_;
  std::unordered_map<std::string, DiagnosticStatus> unknown_diags_;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__GRAPH_HPP_
