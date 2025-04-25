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

#ifndef COMMON__CONFIG__CONTEXT_HPP_
#define COMMON__CONFIG__CONTEXT_HPP_

#include <memory>
#include <string>
#include <unordered_set>

namespace autoware::diagnostic_graph_aggregator
{

struct ParseContext
{
  ParseContext(
    const std::string & parent, const std::shared_ptr<std::unordered_set<std::string>> & visited);

  ParseContext child(const std::string & path);

  bool visit(const std::string & path);

  std::string file() const;

private:
  std::string parent_;
  std::shared_ptr<std::unordered_set<std::string>> visited_;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__CONFIG__CONTEXT_HPP_
