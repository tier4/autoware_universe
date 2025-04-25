// Copyright 2024 The Autoware Contributors
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

#include "config/context.hpp"

#include <memory>
#include <string>
#include <unordered_set>

namespace autoware::diagnostic_graph_aggregator
{

ParseContext::ParseContext(
  const std::string & parent, const std::shared_ptr<std::unordered_set<std::string>> & visited)
{
  parent_ = parent;
  visited_ = visited;
}

ParseContext ParseContext::child(const std::string & path)
{
  return ParseContext(path, visited_);
}

bool ParseContext::visit(const std::string & path)
{
  return visited_->insert(path).second;
}

std::string ParseContext::file() const
{
  return parent_;
}

}  // namespace autoware::diagnostic_graph_aggregator
