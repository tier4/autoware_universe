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

#ifndef AUTOWARE__LOCALIZATION_FUSION_HOST__EKF_CORE_PLUGIN_HPP_
#define AUTOWARE__LOCALIZATION_FUSION_HOST__EKF_CORE_PLUGIN_HPP_

#include "autoware/localization_fusion_host/localization_fusion_plugin_base.hpp"

#include <autoware/ekf_localizer/ekf_localizer_core.hpp>

#include <memory>

namespace autoware::localization_fusion_host::plugin
{

class EkfCorePlugin : public LocalizationFusionPluginBase
{
public:
  void initialize(
    const std::string & name, rclcpp::Node * node_ptr,
    LocalizationFusionHost * host_ptr) override;

  void process(FusionData & /*data*/) override {}

  std::unique_ptr<autoware::ekf_localizer::EKFLocalizerCore> & get_core() { return ekf_core_; }

private:
  std::unique_ptr<autoware::ekf_localizer::EKFLocalizerCore> ekf_core_;
};

}  // namespace autoware::localization_fusion_host::plugin

#endif  // AUTOWARE__LOCALIZATION_FUSION_HOST__EKF_CORE_PLUGIN_HPP_
