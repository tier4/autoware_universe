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

#ifndef COMMON__TYPES__FORWARD_HPP_
#define COMMON__TYPES__FORWARD_HPP_

#include <memory>

namespace autoware::diagnostic_graph_aggregator
{

class ConfigYaml;
class Graph;
class BaseUnit;
class NodeUnit;
class DiagUnit;
class UnitLink;
class Logic;
class LogicConfig;

class LatchLevel;
class HysteresisLevel;
class TimeoutLevel;

struct ParseContext;
struct GraphConfig;
struct FileConfigData;
struct UnitConfigData;
struct LinkConfigData;

using FileConfig = std::shared_ptr<FileConfigData>;
using UnitConfig = std::shared_ptr<UnitConfigData>;
using LinkConfig = std::shared_ptr<LinkConfigData>;

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__TYPES__FORWARD_HPP_
