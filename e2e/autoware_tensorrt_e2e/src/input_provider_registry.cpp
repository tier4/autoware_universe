// Copyright 2026 TIER IV, Inc.
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

#include "autoware/tensorrt_e2e/input_provider_registry.hpp"

#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace autoware::tensorrt_e2e
{

namespace
{
// A function-local static: registrars run during static initialization, in no defined
// order across translation units, and this is the one form guaranteed to be ready first.
std::map<std::string, InputProviderFactory> & registry()
{
  static std::map<std::string, InputProviderFactory> instance;
  return instance;
}
}  // namespace

void register_input_provider(const std::string & sensor, InputProviderFactory factory)
{
  registry()[sensor] = std::move(factory);
}

std::unique_ptr<InputProviderInterface> make_input_provider(
  const std::string & sensor, rclcpp::Node & node, tf2_ros::Buffer & tf_buffer)
{
  const auto it = registry().find(sensor);
  if (it == registry().end()) {
    std::ostringstream oss;
    oss << "Unknown sensor input '" << sensor << "' (registered:";
    for (const auto & name : registered_input_providers()) {
      oss << " \"" << name << "\"";
    }
    oss << ")";
    throw std::runtime_error(oss.str());
  }
  return it->second(node, tf_buffer);
}

std::vector<std::string> registered_input_providers()
{
  std::vector<std::string> names;
  for (const auto & [name, factory] : registry()) {
    names.push_back(name);
  }
  return names;
}

}  // namespace autoware::tensorrt_e2e
