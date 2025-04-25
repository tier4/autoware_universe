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

#include "graph/logic.hpp"

#include "config/errors.hpp"

#include <memory>
#include <string>

namespace autoware::diagnostic_graph_aggregator
{

std::unique_ptr<Logic> LogicFactory::Create(const std::string & type, const LogicConfig & config)
{
  const auto iter = logics_.find(type);
  if (iter != logics_.end()) {
    return iter->second(config);
  }
  throw UnknownLogic(type);
}

void LogicFactory::Register(const std::string & type, Function function)
{
  logics_[type] = function;
}

}  // namespace autoware::diagnostic_graph_aggregator
