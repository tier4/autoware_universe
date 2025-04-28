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

#ifndef COMMON__GRAPH__LOGIC_HPP_
#define COMMON__GRAPH__LOGIC_HPP_

#include "types/forward.hpp"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::diagnostic_graph_aggregator
{

class Logic
{
public:
  virtual ~Logic() = default;
};

class LogicFactory
{
public:
  using Function = std::function<std::unique_ptr<Logic>(const LogicConfig &)>;
  static std::unique_ptr<Logic> Create(const std::string & type, const LogicConfig & config);
  static void Register(const std::string & type, Function function);

private:
  static inline std::unordered_map<std::string, Function> logics_;
};

template <typename T>
class RegisterLogic
{
public:
  explicit RegisterLogic(const std::string & type)
  {
    LogicFactory::Register(
      type, [](const LogicConfig & config) { return std::make_unique<T>(config); });
  }
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__LOGIC_HPP_
