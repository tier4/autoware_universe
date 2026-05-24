// Copyright 2025 Autoware Foundation
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

#include "autoware/localization_fusion_host/ekf_core_plugin.hpp"

namespace autoware::localization_fusion_host::plugin
{

void EkfCorePlugin::initialize(
  const std::string & name, rclcpp::Node * node_ptr, LocalizationFusionHost * host_ptr)
{
  LocalizationFusionPluginBase::initialize(name, node_ptr, host_ptr);
  ekf_core_ = std::make_unique<autoware::ekf_localizer::EKFLocalizerCore>(node_ptr);
}

}  // namespace autoware::localization_fusion_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::localization_fusion_host::plugin::EkfCorePlugin,
  autoware::localization_fusion_host::plugin::LocalizationFusionPluginBase)
